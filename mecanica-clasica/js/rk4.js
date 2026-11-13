/**
 * rk4.js — Integrador numérico Runge-Kutta de orden 4 (RK4),
 * detección exacta de eventos por bisección y utilidades vectoriales 2D/3D.
 * Compatible con navegador (sin módulos ES por CORS en file://) y Node.js (CommonJS).
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory();
    } else {
        root.PhysicsRK4 = factory();
    }
}(typeof self !== 'undefined' ? self : this, function () {
    'use strict';

    /**
     * Un paso clásico RK4 sobre un vector de estado y(t).
     * @param {number[]} y - Vector de estado actual.
     * @param {number} t - Tiempo actual.
     * @param {number} dt - Paso temporal.
     * @param {function(number[], number): number[]} derivs - Función dy/dt = f(y, t).
     * @returns {number[]} Nuevo vector de estado y(t + dt).
     */
    function rk4Step(y, t, dt, derivs) {
        var n = y.length;
        if (n === 0 || dt === 0) return y.slice();

        var k1 = derivs(y, t);
        var y2 = new Array(n);
        for (var i = 0; i < n; i++) y2[i] = y[i] + 0.5 * dt * k1[i];

        var k2 = derivs(y2, t + 0.5 * dt);
        var y3 = new Array(n);
        for (var i = 0; i < n; i++) y3[i] = y[i] + 0.5 * dt * k2[i];

        var k3 = derivs(y3, t + 0.5 * dt);
        var y4 = new Array(n);
        for (var i = 0; i < n; i++) y4[i] = y[i] + dt * k3[i];

        var k4 = derivs(y4, t + dt);
        var out = new Array(n);
        var dt6 = dt / 6.0;
        for (var i = 0; i < n; i++) {
            out[i] = y[i] + dt6 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        return out;
    }

    /**
     * Integración RK4 subdividida en subpasos <= maxSubStep.
     */
    function rk4Integrate(y, t, dt, maxSubStep, derivs) {
        if (dt === 0) return y.slice();
        var absDt = Math.abs(dt);
        var steps = (maxSubStep > 0 && absDt > maxSubStep) ? Math.ceil(absDt / maxSubStep) : 1;
        var h = dt / steps;
        var cur = y.slice();
        var curT = t;
        for (var s = 0; s < steps; s++) {
            cur = rk4Step(cur, curT, h, derivs);
            curT += h;
        }
        return cur;
    }

    /**
     * Localiza por bisección sobre pasos RK4 el instante exacto dentro de [t0, t0+dt]
     * donde eventFn(y, t) cruza 0 (de f0 <= 0 a f1 >= 0 o viceversa).
     * @returns {{ tEvent: number, dtEvent: number, yEvent: number[] }}
     */
    function bisectEvent(y0, t0, dt, derivs, eventFn, iterations) {
        var iters = iterations || 28;
        var f0 = eventFn(y0, t0);
        var lo = 0.0;
        var hi = dt;
        var yHi = rk4Step(y0, t0, dt, derivs);
        var yBest = yHi;

        for (var k = 0; k < iters; k++) {
            var mid = 0.5 * (lo + hi);
            var yMid = rk4Step(y0, t0, mid, derivs);
            var fMid = eventFn(yMid, t0 + mid);
            yBest = yMid;
            if (fMid === 0) {
                lo = hi = mid;
                break;
            }
            if ((f0 <= 0 && fMid <= 0) || (f0 >= 0 && fMid >= 0)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        var dtEv = 0.5 * (lo + hi);
        yBest = rk4Step(y0, t0, dtEv, derivs);
        return {
            tEvent: t0 + dtEv,
            dtEvent: dtEv,
            yEvent: yBest
        };
    }

    function clamp(x, minVal, maxVal) {
        return x < minVal ? minVal : (x > maxVal ? maxVal : x);
    }

    function degToRad(deg) {
        return deg * Math.PI / 180.0;
    }

    function radToDeg(rad) {
        return rad * 180.0 / Math.PI;
    }

    function fmt(val, decimals) {
        if (typeof val !== 'number' || !isFinite(val)) return '—';
        var d = (decimals === undefined) ? 3 : decimals;
        if (Math.abs(val) < Math.pow(10, -d - 2)) val = 0;
        return val.toFixed(d);
    }

    return {
        rk4Step: rk4Step,
        rk4Integrate: rk4Integrate,
        bisectEvent: bisectEvent,
        clamp: clamp,
        degToRad: degToRad,
        radToDeg: radToDeg,
        fmt: fmt
    };
}));
