/**
 * renderer2d.js — Motor gráfico 2D de alta precisión (HiDPI) con cámara pan/zoom,
 * rejilla métrica, ruedas con radios giratorios, trayectorias y vectores etiquetados.
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory();
    } else {
        root.Renderer2D = factory();
    }
}(typeof self !== 'undefined' ? self : this, function () {
    'use strict';

    function Renderer2D(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.zoom = 1.0;
        this.panX = 0;
        this.panY = 0;
        this.dragging = false;
        this.lastMouse = { x: 0, y: 0 };
        this.lastBoundsKey = '';

        this._bindEvents();
    }

    Renderer2D.prototype._bindEvents = function () {
        var self = this;
        this.canvas.addEventListener('wheel', function (e) {
            e.preventDefault();
            var factor = e.deltaY < 0 ? 1.12 : 0.89;
            self.zoom = Math.max(0.2, Math.min(12.0, self.zoom * factor));
        }, { passive: false });

        this.canvas.addEventListener('mousedown', function (e) {
            self.dragging = true;
            self.lastMouse = { x: e.clientX, y: e.clientY };
        });
        window.addEventListener('mousemove', function (e) {
            if (!self.dragging) return;
            self.panX += e.clientX - self.lastMouse.x;
            self.panY += e.clientY - self.lastMouse.y;
            self.lastMouse = { x: e.clientX, y: e.clientY };
        });
        window.addEventListener('mouseup', function () {
            self.dragging = false;
        });
    };

    Renderer2D.prototype.resetView = function () {
        this.zoom = 1.0;
        this.panX = 0;
        this.panY = 0;
    };

    Renderer2D.prototype._niceStep = function (range) {
        if (range <= 0) return 1;
        var rough = range / 7;
        var pow10 = Math.pow(10, Math.floor(Math.log10(rough)));
        var frac = rough / pow10;
        var nice = frac < 1.5 ? 1 : (frac < 3.5 ? 2 : (frac < 7.5 ? 5 : 10));
        return nice * pow10;
    };

    Renderer2D.prototype.render = function (renderState, options) {
        var canvas = this.canvas;
        var rect = canvas.getBoundingClientRect();
        if (rect.width < 10 || rect.height < 10) return;

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

        // Background
        var grad = ctx.createRadialGradient(cw * 0.5, ch * 0.45, 30, cw * 0.5, ch * 0.5, Math.max(cw, ch) * 0.8);
        grad.addColorStop(0, '#0f172a');
        grad.addColorStop(1, '#060911');
        ctx.fillStyle = grad;
        ctx.fillRect(0, 0, cw, ch);

        var b = renderState.bounds || { minX: -5, maxX: 5, minY: -5, maxY: 5 };
        var worldW = Math.max(0.5, b.maxX - b.minX);
        var worldH = Math.max(0.5, b.maxY - b.minY);
        var pad = 42;
        var baseScale = Math.min((cw - 2 * pad) / worldW, (ch - 2 * pad) / worldH);
        var scale = baseScale * this.zoom;

        var cxWorld = 0.5 * (b.minX + b.maxX);
        var cyWorld = 0.5 * (b.minY + b.maxY);

        function toScreenX(wx) {
            return cw * 0.5 + (wx - cxWorld) * scale + this.panX;
        }
        function toScreenY(wy) {
            return ch * 0.5 - (wy - cyWorld) * scale + this.panY;
        }
        var sx = toScreenX.bind(this);
        var sy = toScreenY.bind(this);

        // Visible world bounds for grid
        var visMinX = cxWorld - (cw * 0.5 + this.panX) / scale;
        var visMaxX = cxWorld + (cw * 0.5 - this.panX) / scale;
        var visMinY = cyWorld - (ch * 0.5 - this.panY) / scale;
        var visMaxY = cyWorld + (ch * 0.5 + this.panY) / scale;

        // Grid
        var toolbarEl = canvas.parentElement && canvas.parentElement.querySelector ? canvas.parentElement.querySelector('.viewport-toolbar') : null;
        var bottomClear = toolbarEl && toolbarEl.offsetHeight ? Math.max(48, toolbarEl.offsetHeight + 18) : 48;

        var step = this._niceStep(Math.max(visMaxX - visMinX, visMaxY - visMinY));
        ctx.lineWidth = 1;
        ctx.font = '10px "Segoe UI", Roboto, monospace';
        ctx.fillStyle = 'rgba(148, 163, 184, 0.55)';

        var xStart = Math.floor(visMinX / step) * step;
        for (var gx = xStart; gx <= visMaxX; gx += step) {
            var px = sx(gx);
            ctx.strokeStyle = Math.abs(gx) < step * 1e-4 ? 'rgba(148, 163, 184, 0.35)' : 'rgba(148, 163, 184, 0.08)';
            ctx.beginPath();
            ctx.moveTo(px, 0);
            ctx.lineTo(px, ch);
            ctx.stroke();
            if (px > 24 && px < cw - 24) {
                ctx.fillText(Number(gx.toFixed(2)) + 'm', px + 3, ch - bottomClear);
            }
        }

        var yStart = Math.floor(visMinY / step) * step;
        for (var gy = yStart; gy <= visMaxY; gy += step) {
            var py = sy(gy);
            ctx.strokeStyle = Math.abs(gy) < step * 1e-4 ? 'rgba(148, 163, 184, 0.35)' : 'rgba(148, 163, 184, 0.08)';
            ctx.beginPath();
            ctx.moveTo(0, py);
            ctx.lineTo(cw, py);
            ctx.stroke();
            if (py > 46 && py < ch - bottomClear - 8) {
                ctx.fillText(Number(gy.toFixed(2)) + 'm', 8, py - 3);
            }
        }

        // Static / Structural Lines
        if (renderState.lines) {
            renderState.lines.forEach(function (ln) {
                ctx.save();
                ctx.strokeStyle = ln.color || '#64748b';
                ctx.lineWidth = ln.width || 2;
                if (ln.dash) ctx.setLineDash(ln.dash);
                ctx.beginPath();
                ctx.moveTo(sx(ln.x1), sy(ln.y1));
                ctx.lineTo(sx(ln.x2), sy(ln.y2));
                ctx.stroke();
                ctx.restore();
            });
        }

        // Trails
        if (options.showTrails && renderState.trails) {
            renderState.trails.forEach(function (tr) {
                if (!tr.pts || tr.pts.length < 2) return;
                ctx.save();
                ctx.strokeStyle = tr.color || '#38bdf8';
                ctx.lineWidth = tr.width || 2;
                if (tr.dash) ctx.setLineDash(tr.dash);
                ctx.beginPath();
                ctx.moveTo(sx(tr.pts[0].x), sy(tr.pts[0].y));
                for (var i = 1; i < tr.pts.length; i++) {
                    ctx.lineTo(sx(tr.pts[i].x), sy(tr.pts[i].y));
                }
                ctx.stroke();
                ctx.restore();
            });
        }

        // Rods
        if (renderState.rods) {
            renderState.rods.forEach(function (rd) {
                ctx.save();
                ctx.strokeStyle = rd.color || '#cbd5e1';
                ctx.lineWidth = rd.width || 4;
                ctx.lineCap = 'round';
                ctx.beginPath();
                ctx.moveTo(sx(rd.x1), sy(rd.y1));
                ctx.lineTo(sx(rd.x2), sy(rd.y2));
                ctx.stroke();
                ctx.restore();
            });
        }

        // Boxes
        if (renderState.boxes) {
            renderState.boxes.forEach(function (bx) {
                var cx = sx(bx.x), cy = sy(bx.y);
                var bw = bx.w * scale, bh = bx.h * scale;
                ctx.save();
                ctx.fillStyle = bx.color || '#fb7185';
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 1.5;
                ctx.fillRect(cx - bw * 0.5, cy - bh * 0.5, bw, bh);
                ctx.strokeRect(cx - bw * 0.5, cy - bh * 0.5, bw, bh);
                if (bx.label) {
                    ctx.font = 'bold 11px "Segoe UI", sans-serif';
                    ctx.fillStyle = '#f8fafc';
                    ctx.fillText(bx.label, cx + bw * 0.5 + 8, cy + 4);
                }
                ctx.restore();
            });
        }

        // Wheels / Cylinders (with rotating spokes so rolling vs sliding is unmistakable!)
        if (renderState.wheels) {
            renderState.wheels.forEach(function (wh) {
                var cx = sx(wh.x), cy = sy(wh.y);
                var pr = Math.max(8, wh.r * scale);
                ctx.save();
                ctx.translate(cx, cy);
                ctx.rotate(wh.phi); // clockwise positive in screen coords!

                // Body
                ctx.beginPath();
                ctx.arc(0, 0, pr, 0, Math.PI * 2);
                if (wh.hollow) {
                    ctx.fillStyle = 'rgba(251, 191, 36, 0.14)';
                    ctx.fill();
                    ctx.lineWidth = Math.max(4, pr * 0.22);
                    ctx.strokeStyle = wh.color;
                    ctx.stroke();
                } else {
                    ctx.fillStyle = 'rgba(56, 189, 248, 0.25)';
                    ctx.fill();
                    ctx.lineWidth = 2.5;
                    ctx.strokeStyle = wh.color;
                    ctx.stroke();
                }

                // Spokes (4 spokes + bright red marker dot on rim)
                ctx.strokeStyle = wh.color;
                ctx.lineWidth = 1.8;
                for (var k = 0; k < 4; k++) {
                    var a = k * Math.PI * 0.5;
                    ctx.beginPath();
                    ctx.moveTo(0, 0);
                    ctx.lineTo(Math.cos(a) * pr * 0.9, Math.sin(a) * pr * 0.9);
                    ctx.stroke();
                }
                ctx.beginPath();
                ctx.arc(pr * 0.72, 0, Math.max(3, pr * 0.12), 0, Math.PI * 2);
                ctx.fillStyle = '#fb7185';
                ctx.fill();

                ctx.restore();

                if (wh.label) {
                    ctx.font = 'bold 11px "Segoe UI", sans-serif';
                    ctx.fillStyle = wh.color;
                    ctx.fillText(wh.label, cx + pr + 8, cy - pr * 0.4);
                }
            });
        }

        // Masses
        if (renderState.masses) {
            renderState.masses.forEach(function (ms) {
                var cx = sx(ms.x), cy = sy(ms.y);
                var pr = Math.max(6, Math.min(26, ms.r * scale));
                ctx.save();
                ctx.shadowColor = ms.color;
                ctx.shadowBlur = 12;
                ctx.beginPath();
                ctx.arc(cx, cy, pr, 0, Math.PI * 2);
                ctx.fillStyle = ms.color;
                ctx.fill();
                ctx.shadowBlur = 0;
                ctx.lineWidth = 1.5;
                ctx.strokeStyle = 'rgba(255,255,255,0.75)';
                ctx.stroke();
                if (ms.label) {
                    ctx.font = 'bold 11px "Segoe UI", sans-serif';
                    var mtw = ctx.measureText(ms.label).width;
                    var mlx = cx + pr + 6, mly = cy - 4;
                    ctx.fillStyle = 'rgba(9, 13, 22, 0.72)';
                    ctx.fillRect(mlx - 3, mly - 10, mtw + 6, 13);
                    ctx.fillStyle = '#f1f5f9';
                    ctx.fillText(ms.label, mlx, mly);
                }
                ctx.restore();
            });
        }

        // CM Marker (Gold Diamond + Crosshair)
        if (options.showCM && renderState.cm) {
            var cmx = sx(renderState.cm.x), cmy = sy(renderState.cm.y);
            ctx.save();
            ctx.strokeStyle = '#fbbf24';
            ctx.fillStyle = '#fbbf24';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(cmx, cmy - 8);
            ctx.lineTo(cmx + 8, cmy);
            ctx.lineTo(cmx, cmy + 8);
            ctx.lineTo(cmx - 8, cmy);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = '#0f172a';
            ctx.lineWidth = 1.5;
            ctx.stroke();
            if (renderState.cm.label) {
                ctx.font = 'bold 11px "Segoe UI", sans-serif';
                ctx.fillStyle = '#fbbf24';
                ctx.fillText(renderState.cm.label, cmx + 11, cmy + 4);
            }
            ctx.restore();
        }

        // Vectors
        if (renderState.vectors) {
            renderState.vectors.forEach(function (vec) {
                if (vec.type === 'velocity' && !options.showVelocity) return;
                if (vec.type === 'momentum' && !options.showMomentum) return;
                if (vec.type === 'force' && !options.showForce) return;
                if (vec.type === 'tension' && !options.showForce) return;

                var x0 = sx(vec.x), y0 = sy(vec.y);
                var dx = vec.vx * scale;
                var dy = -vec.vy * scale;
                var len = Math.hypot(dx, dy);
                if (len < 4) return;
                // Cap screen arrow length to keep layout legible
                var maxLen = 140;
                if (len > maxLen) {
                    dx *= maxLen / len;
                    dy *= maxLen / len;
                    len = maxLen;
                } else if (len < 22) {
                    dx *= 22 / len;
                    dy *= 22 / len;
                    len = 22;
                }
                var x1 = x0 + dx, y1 = y0 + dy;
                var angle = Math.atan2(dy, dx);
                var head = 9;

                ctx.save();
                ctx.strokeStyle = vec.color;
                ctx.fillStyle = vec.color;
                ctx.lineWidth = 2.4;
                ctx.beginPath();
                ctx.moveTo(x0, y0);
                ctx.lineTo(x1, y1);
                ctx.stroke();

                ctx.beginPath();
                ctx.moveTo(x1, y1);
                ctx.lineTo(x1 - head * Math.cos(angle - 0.42), y1 - head * Math.sin(angle - 0.42));
                ctx.lineTo(x1 - head * Math.cos(angle + 0.42), y1 - head * Math.sin(angle + 0.42));
                ctx.closePath();
                ctx.fill();

                if (vec.label) {
                    ctx.font = 'bold 11px "Segoe UI", sans-serif';
                    var tw = ctx.measureText(vec.label).width;
                    var lx = x1 + 6, ly = y1 - 4;
                    ctx.fillStyle = 'rgba(9, 13, 22, 0.78)';
                    ctx.fillRect(lx - 3, ly - 10, tw + 6, 13);
                    ctx.fillStyle = vec.color;
                    ctx.fillText(vec.label, lx, ly);
                }
                ctx.restore();
            });
        }

        // Top HUD Banner (wraps cleanly on narrow viewports)
        if (renderState.banner) {
            ctx.save();
            ctx.font = '600 12px "Segoe UI", Roboto, sans-serif';
            var bx = 14, by = 14;
            var maxBoxW = Math.max(120, cw - 28);
            var maxTextW = maxBoxW - 22;
            var words = String(renderState.banner).split(/\s+/);
            var bLines = [];
            var curLine = '';
            for (var wi = 0; wi < words.length; wi++) {
                var candidate = curLine ? (curLine + ' ' + words[wi]) : words[wi];
                if (ctx.measureText(candidate).width <= maxTextW || !curLine) {
                    curLine = candidate;
                } else {
                    bLines.push(curLine);
                    curLine = words[wi];
                }
            }
            if (curLine) bLines.push(curLine);

            var maxLineW = 0;
            for (var li = 0; li < bLines.length; li++) {
                var lw = ctx.measureText(bLines[li]).width;
                if (lw > maxLineW) maxLineW = lw;
            }
            var boxW = Math.min(maxBoxW, maxLineW + 22);
            var lineH = 15;
            var boxH = 11 + bLines.length * lineH;

            ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
            ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.roundRect ? ctx.roundRect(bx, by, boxW, boxH, 6) : ctx.rect(bx, by, boxW, boxH);
            ctx.fill();
            ctx.stroke();
            ctx.fillStyle = '#e2e8f0';
            for (var li = 0; li < bLines.length; li++) {
                ctx.fillText(bLines[li], bx + 11, by + 17 + li * lineH);
            }
            ctx.restore();
        }

        ctx.restore();
    };

    return Renderer2D;
}));
