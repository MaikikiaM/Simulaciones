/**
 * charts.js — Gráficas en tiempo real de conservación (Energías Ec, Ep, Etot y Momentos P, Lz).
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory();
    } else {
        root.ConservationCharts = factory();
    }
}(typeof self !== 'undefined' ? self : this, function () {
    'use strict';

    function ChartSeries(canvas, title, seriesDefs) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.title = title;
        this.seriesDefs = seriesDefs;
        this.samples = [];
        this.maxSamples = 500;
    }

    ChartSeries.prototype.clear = function () {
        this.samples = [];
    };

    ChartSeries.prototype.push = function (t, valuesObj) {
        if (this.samples.length > 0 && t <= this.samples[this.samples.length - 1].t + 1e-6) return;
        this.samples.push({ t: t, v: valuesObj });
        if (this.samples.length > this.maxSamples) {
            // Downsample slightly or shift oldest
            this.samples.shift();
        }
    };

    ChartSeries.prototype.draw = function () {
        var canvas = this.canvas;
        var rect = canvas.getBoundingClientRect();
        if (rect.width < 20 || rect.height < 20) return;

        var dpr = Math.min(window.devicePixelRatio || 1, 2);
        var w = Math.round(rect.width * dpr);
        var h = Math.round(rect.height * dpr);
        if (canvas.width !== w || canvas.height !== h) {
            canvas.width = w;
            canvas.height = h;
        }

        var ctx = this.ctx;
        ctx.save();
        ctx.scale(dpr, dpr);
        var cw = rect.width;
        var ch = rect.height;

        ctx.fillStyle = '#090e18';
        ctx.fillRect(0, 0, cw, ch);

        var padL = 48, padR = 12, padT = 24, padB = 18;

        // Title & legend (responsive: wraps below title if narrow so items never clip off right edge)
        ctx.font = '600 11px "Segoe UI", sans-serif';
        ctx.fillStyle = '#cbd5e1';
        ctx.fillText(this.title, 8, 15);
        var titleW = ctx.measureText(this.title).width;

        var lastSample = this.samples.length > 0 ? this.samples[this.samples.length - 1] : null;
        ctx.font = '10px "Segoe UI", monospace';
        var legendItems = [];
        var totalLegW = 0;
        for (var s = 0; s < this.seriesDefs.length; s++) {
            var def = this.seriesDefs[s];
            var valStr = lastSample && isFinite(lastSample.v[def.key]) ? (': ' + this._compactNum(lastSample.v[def.key])) : '';
            var txt = def.label + valStr;
            var itemW = 12 + ctx.measureText(txt).width;
            legendItems.push({ color: def.color, txt: txt, w: itemW });
            totalLegW += itemW;
        }

        var minGap = 6;
        var singleRowMinW = (titleW + 14) + totalLegW + minGap * Math.max(0, legendItems.length - 1) + 8;
        if (singleRowMinW <= cw) {
            var gap = legendItems.length > 1 ? Math.max(minGap, Math.min(14, (cw - 14 - titleW - 12 - totalLegW) / (legendItems.length - 1))) : 0;
            var lx = Math.max(titleW + 14, cw - 8 - (totalLegW + gap * Math.max(0, legendItems.length - 1)));
            for (var s = 0; s < legendItems.length; s++) {
                var it = legendItems[s];
                ctx.fillStyle = it.color;
                ctx.fillRect(lx, 8, 8, 8);
                ctx.fillStyle = '#94a3b8';
                ctx.fillText(it.txt, lx + 11, 15);
                lx += it.w + gap;
            }
        } else {
            // Wrap legend onto row(s) below the title
            var legGap = 10;
            var curX = 8;
            var curY = 29;
            for (var s = 0; s < legendItems.length; s++) {
                var it = legendItems[s];
                if (s > 0 && curX + it.w > cw - 8) {
                    curX = 8;
                    curY += 13;
                }
                ctx.fillStyle = it.color;
                ctx.fillRect(curX, curY - 7, 8, 8);
                ctx.fillStyle = '#94a3b8';
                ctx.fillText(it.txt, curX + 11, curY);
                curX += it.w + legGap;
            }
            padT = curY + 7;
        }

        var pw = Math.max(10, cw - padL - padR);
        var ph = Math.max(10, ch - padT - padB);

        // Plot frame
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.16)';
        ctx.lineWidth = 1;
        ctx.strokeRect(padL, padT, pw, ph);

        if (this.samples.length < 2) {
            ctx.fillStyle = 'rgba(148, 163, 184, 0.45)';
            ctx.font = '11px "Segoe UI", sans-serif';
            ctx.fillText('Pulsa Iniciar para registrar evolución temporal...', padL + 14, padT + ph * 0.55);
            ctx.restore();
            return;
        }

        // Bounds
        var minT = this.samples[0].t;
        var maxT = this.samples[this.samples.length - 1].t;
        if (maxT <= minT) maxT = minT + 1;

        var minY = Infinity, maxY = -Infinity;
        for (var i = 0; i < this.samples.length; i++) {
            for (var k = 0; k < this.seriesDefs.length; k++) {
                var val = this.samples[i].v[this.seriesDefs[k].key];
                if (typeof val === 'number' && isFinite(val)) {
                    if (val < minY) minY = val;
                    if (val > maxY) maxY = val;
                }
            }
        }
        if (!isFinite(minY)) { minY = 0; maxY = 1; }
        if (Math.abs(maxY - minY) < 1e-6) {
            var padVal = Math.max(1, Math.abs(maxY) * 0.25);
            minY -= padVal;
            maxY += padVal;
        } else {
            var margin = (maxY - minY) * 0.12;
            minY -= margin;
            maxY += margin;
        }
        if (minY > 0 && minY < maxY * 0.15) minY = 0;

        // Horizontal ticks
        ctx.font = '9.5px "Segoe UI", monospace';
        ctx.fillStyle = 'rgba(148, 163, 184, 0.65)';
        for (var g = 0; g <= 2; g++) {
            var frac = g / 2;
            var yVal = maxY - frac * (maxY - minY);
            var py = padT + frac * ph;
            ctx.strokeStyle = 'rgba(148, 163, 184, 0.08)';
            ctx.beginPath();
            ctx.moveTo(padL, py);
            ctx.lineTo(padL + pw, py);
            ctx.stroke();
            ctx.fillText(this._compactNum(yVal), 4, py + 3);
        }

        // Draw series
        for (var k = 0; k < this.seriesDefs.length; k++) {
            var def = this.seriesDefs[k];
            ctx.save();
            ctx.strokeStyle = def.color;
            ctx.lineWidth = def.width || 2;
            if (def.dash) ctx.setLineDash(def.dash);
            ctx.beginPath();
            var started = false;
            for (var i = 0; i < this.samples.length; i++) {
                var val = this.samples[i].v[def.key];
                if (typeof val !== 'number' || !isFinite(val)) continue;
                var px = padL + ((this.samples[i].t - minT) / (maxT - minT)) * pw;
                var py = padT + (1 - (val - minY) / (maxY - minY)) * ph;
                if (!started) { ctx.moveTo(px, py); started = true; }
                else ctx.lineTo(px, py);
            }
            ctx.stroke();
            ctx.restore();
        }

        // Time axis labels
        ctx.fillStyle = 'rgba(148, 163, 184, 0.65)';
        ctx.fillText(minT.toFixed(1) + 's', padL, ch - 4);
        ctx.fillText(maxT.toFixed(1) + 's', padL + pw - 26, ch - 4);

        ctx.restore();
    };

    ChartSeries.prototype._compactNum = function (v) {
        var av = Math.abs(v);
        if (av >= 1e6) return (v / 1e6).toFixed(2) + 'M';
        if (av >= 1e4) return (v / 1e3).toFixed(1) + 'k';
        if (av >= 100) return v.toFixed(0);
        if (av >= 1) return v.toFixed(2);
        if (av < 1e-4) return '0';
        return v.toFixed(3);
    };

    return {
        ChartSeries: ChartSeries
    };
}));
