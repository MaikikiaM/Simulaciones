/**
 * problems_fundamentals.js — 4 simulaciones de Fundamentos de Mecánica Clásica (T1–T4)
 * para aprender desde 0 antes de abordar Sistemas de Partículas y Sólido Rígido:
 *  - F1: Tiro Parabólico y Descomposición Vectorial (Cinemática 2D)
 *  - F2: Plano Inclinado con Rozamiento Estático y Cinético (Dinámica)
 *  - F3: Oscilador Armónico Masa-Muelle con Amortiguamiento (Trabajo, Energía y M.A.S.)
 *  - F4: Órbita Gravitatoria de Kepler (Fuerzas Centrales y Conservación de L y E)
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory(require('./rk4.js'));
    } else {
        root.FundamentalProblems = factory(root.PhysicsRK4);
    }
}(typeof self !== 'undefined' ? self : this, function (RK4) {
    'use strict';

    var rk4Integrate = RK4.rk4Integrate;
    var bisectEvent = RK4.bisectEvent;
    var degToRad = RK4.degToRad;
    var radToDeg = RK4.radToDeg;
    var fmt = RK4.fmt;

    // =========================================================================
    // F1 — Cinemática 2D: Tiro Parabólico y Vectores
    // =========================================================================
    var f_p1 = {
        id: 'f_p1',
        topic: 'fundamentals',
        level: 1,
        code: 'T1 · F1',
        title: 'Tiro Parabólico y Descomposición Vectorial',
        subtitle: 'MRU horizontal + MRUA vertical independientes bajo gravedad',
        renderMode: '2d',
        learningGoal: 'Entender el principio de independencia de movimientos: en X no hay fuerza (vₓ constante) y en Y actúa el peso constante (aᵧ = −g).',
        guideFromZero: [
            '<strong>1. Descomposición inicial:</strong> <code>v₀ₓ = v₀ cos θ</code> y <code>v₀ᵧ = v₀ sin θ</code>.',
            '<strong>2. Ecuaciones horarias:</strong> <code>x(t) = v₀ₓ·t</code> ; <code>y(t) = h₀ + v₀ᵧ·t − ½ g t²</code>.',
            '<strong>3. Alcance y altura máxima:</strong> Con h₀ = 0, el alcance máximo ocurre en <code>θ = 45°</code> y la altura máxima es <code>H = v₀ᵧ² / (2g)</code>.'
        ],
        statement: 'Se lanza un proyectil de masa m = 2 kg desde altura h₀ = 0 m con velocidad v₀ = 25 m/s y ángulo θ = 45°. Determinar el tiempo de vuelo, la altura máxima y el alcance horizontal comparando la solución analítica con RK4.',
        paramsSchema: [
            { key: 'v0', label: 'Velocidad inicial v₀', unit: 'm/s', min: 5, max: 50, step: 1, def: 25 },
            { key: 'thetaDeg', label: 'Ángulo lanzamiento θ', unit: '°', min: 10, max: 85, step: 1, def: 45 },
            { key: 'h0', label: 'Altura inicial h₀', unit: 'm', min: 0, max: 30, step: 1, def: 0 },
            { key: 'm', label: 'Masa m', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 2 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'Ángulo óptimo 45°', apply: function (p) { p.thetaDeg = 45; p.h0 = 0; } },
            { label: 'Ángulos complementarios 30°', apply: function (p) { p.thetaDeg = 30; p.h0 = 0; } },
            { label: 'Ángulos complementarios 60°', apply: function (p) { p.thetaDeg = 60; p.h0 = 0; } }
        ],
        defaultDuration: 5.0,

        solveAnalytical: function (p) {
            var th = degToRad(p.thetaDeg);
            var vx0 = p.v0 * Math.cos(th);
            var vy0 = p.v0 * Math.sin(th);
            var tApex = vy0 / p.g;
            var yMax = p.h0 + 0.5 * vy0 * tApex;
            var tFlight = (vy0 + Math.sqrt(vy0 * vy0 + 2 * p.g * p.h0)) / p.g;
            var range = vx0 * tFlight;
            var Etot = 0.5 * p.m * p.v0 * p.v0 + p.m * p.g * p.h0;
            return {
                steps: [
                    {
                        title: '1. Componentes iniciales de velocidad',
                        formula: 'v₀ₓ = v₀ cos θ  ;  v₀ᵧ = v₀ sin θ',
                        detail: 'v₀ₓ = ' + fmt(vx0, 3) + ' m/s (constante)  ;  v₀ᵧ = ' + fmt(vy0, 3) + ' m/s'
                    },
                    {
                        title: '2. Altura máxima (donde vᵧ = 0)',
                        formula: 't_cima = v₀ᵧ / g  ;  y_max = h₀ + v₀ᵧ² / (2g)',
                        detail: 't_cima = ' + fmt(tApex, 3) + ' s  →  y_max = ' + fmt(yMax, 3) + ' m'
                    },
                    {
                        title: '3. Tiempo de vuelo y alcance horizontal',
                        formula: 't_vuelo = [v₀ᵧ + √(v₀ᵧ² + 2gh₀)] / g  ;  R = v₀ₓ · t_vuelo',
                        detail: 't_vuelo = ' + fmt(tFlight, 3) + ' s  →  Alcance R = ' + fmt(range, 3) + ' m'
                    }
                ],
                values: { vx0: vx0, vy0: vy0, tApex: tApex, yMax: yMax, tFlight: tFlight, range: range, Etot: Etot }
            };
        },

        createSimulation: function (p) {
            var ana = f_p1.solveAnalytical(p).values;
            var y = [0, p.h0, ana.vx0, ana.vy0];
            var t = 0;
            var landed = false;
            var simYMax = p.h0;
            var simTime = NaN;
            var trails = [{ x: 0, y: p.h0 }];

            function derivs(st) { return [st[2], st[3], 0, -p.g]; }

            return {
                step: function (dt) {
                    if (landed) return;
                    var yNext = rk4Integrate(y, t, dt, 0.005, derivs);
                    if (y[3] > 0 && yNext[3] <= 0) {
                        var evTop = bisectEvent(y, t, dt, derivs, function (st) { return st[3]; }, 22);
                        simYMax = Math.max(simYMax, evTop.yEvent[1]);
                    }
                    if (yNext[1] <= 0 && (t > 0 || p.h0 === 0 && yNext[3] < 0)) {
                        var evLand = bisectEvent(y, t, dt, derivs, function (st) { return st[1]; }, 24);
                        y = evLand.yEvent;
                        y[1] = 0;
                        t = evLand.tEvent;
                        simTime = t;
                        landed = true;
                    } else {
                        y = yNext;
                        t += dt;
                    }
                    simYMax = Math.max(simYMax, y[1]);
                    if (trails.length < 1000) trails.push({ x: y[0], y: y[1] });
                },
                getObservables: function () {
                    var Ec = 0.5 * p.m * (y[2] * y[2] + y[3] * y[3]);
                    var Ep = p.m * p.g * Math.max(0, y[1]);
                    return {
                        t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                        Px: p.m * y[2], Py: p.m * y[3], P: p.m * Math.hypot(y[2], y[3]),
                        Lz: p.m * (y[0] * y[3] - y[1] * y[2]),
                        cm: { x: y[0], y: y[1] },
                        finished: landed,
                        extra: [
                            { label: 'Posición (x, y)', value: '(' + fmt(y[0], 2) + ', ' + fmt(y[1], 2) + ') m' },
                            { label: 'vₓ / vᵧ', value: fmt(y[2], 2) + ' / ' + fmt(y[3], 2) + ' m/s' },
                            { label: 'Altura máx', value: fmt(simYMax, 2) + ' m' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var Ec = 0.5 * p.m * (y[2] * y[2] + y[3] * y[3]);
                    var Ep = p.m * p.g * Math.max(0, y[1]);
                    return [
                        { label: 'Altura máxima', unit: 'm', theo: ana.yMax, sim: (t >= ana.tApex || landed) ? simYMax : NaN },
                        { label: 'Tiempo de vuelo', unit: 's', theo: ana.tFlight, sim: simTime },
                        { label: 'Alcance R', unit: 'm', theo: ana.range, sim: landed ? y[0] : NaN },
                        { label: 'E_mecánica', unit: 'J', theo: ana.Etot, sim: Ec + Ep }
                    ];
                },
                getRenderState: function () {
                    var maxX = Math.max(20, ana.range * 1.1, y[0] + 3);
                    var maxY = Math.max(12, ana.yMax * 1.25);
                    return {
                        bounds: { minX: -2, maxX: maxX, minY: -2, maxY: maxY },
                        lines: [
                            { x1: -2, y1: 0, x2: maxX, y2: 0, color: '#22c55e', width: 3 },
                            { x1: 0, y1: 0, x2: 0, y2: p.h0, color: '#64748b', width: 4 }
                        ],
                        trails: [
                            { pts: trails, color: '#38bdf8', width: 2.5 }
                        ],
                        masses: [
                            { x: y[0], y: y[1], r: maxX * 0.015, color: '#38bdf8', label: 'm (' + p.m + ' kg)' }
                        ],
                        vectors: landed ? [] : [
                            { type: 'velocity', x: y[0], y: y[1], vx: y[2] * 0.25, vy: y[3] * 0.25, color: '#22d3ee', label: 'v' },
                            { type: 'velocity', x: y[0], y: y[1], vx: y[2] * 0.25, vy: 0, color: '#94a3b8', label: 'vₓ' },
                            { type: 'velocity', x: y[0], y: y[1], vx: 0, vy: y[3] * 0.25, color: '#a78bfa', label: 'vᵧ' },
                            { type: 'force', x: y[0], y: y[1], vx: 0, vy: -p.m * p.g * 0.15, color: '#fb7185', label: 'mg' }
                        ],
                        banner: landed
                            ? 'Impacto en x = ' + fmt(y[0], 2) + ' m (t = ' + fmt(t, 2) + ' s)'
                            : 'En vuelo: vₓ = ' + fmt(y[2], 2) + ' m/s constante | vᵧ = ' + fmt(y[3], 2) + ' m/s'
                    };
                }
            };
        }
    };

    // =========================================================================
    // F2 — Dinámica: Plano Inclinado con Rozamiento Estático y Cinético
    // =========================================================================
    var f_p2 = {
        id: 'f_p2',
        topic: 'fundamentals',
        level: 2,
        code: 'T2 · F2',
        title: 'Bloque en Plano Inclinado con Rozamiento',
        subtitle: 'Umbral estático tan θ > μ_e frente a deslizamiento con μ_d',
        renderMode: '2d',
        learningGoal: 'Distinguir la fuerza de rozamiento estática autoajustable (f_r ≤ μ_e N) de la cinética (f_r = μ_d N) al variar el ángulo θ del plano.',
        guideFromZero: [
            '<strong>1. Descomposición del peso:</strong> Paralela al plano <code>mg sin θ</code> y perpendicular <code>N = mg cos θ</code>.',
            '<strong>2. Condición de reposo:</strong> El bloque permanece quieto si <code>mg sin θ ≤ μ_e N</code>, es decir <code>tan θ ≤ μ_e</code>.',
            '<strong>3. Si desliza:</strong> Actúa el rozamiento cinético opuesto al movimiento: <code>a = g (sin θ − μ_d cos θ)</code>.'
        ],
        statement: 'Un bloque de masa m = 5 kg se coloca sobre una rampa de longitud L = 8 m inclinada un ángulo θ = 32° con la horizontal (μ_e = 0.50, μ_d = 0.35). Determinar si desliza, su aceleración y el tiempo hasta llegar a la base.',
        paramsSchema: [
            { key: 'm', label: 'Masa del bloque m', unit: 'kg', min: 1, max: 20, step: 0.5, def: 5 },
            { key: 'L', label: 'Longitud de la rampa L', unit: 'm', min: 3, max: 15, step: 0.5, def: 8 },
            { key: 'thetaDeg', label: 'Ángulo plano θ', unit: '°', min: 5, max: 60, step: 1, def: 32 },
            { key: 'mu_e', label: 'Coef. estático μ_e', unit: '', min: 0.05, max: 1.0, step: 0.05, def: 0.50 },
            { key: 'mu_d', label: 'Coef. cinético μ_d', unit: '', min: 0.0, max: 0.9, step: 0.05, def: 0.35 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'Desliza (θ=32° > 26.6°)', apply: function (p) { p.thetaDeg = 32; p.mu_e = 0.50; p.mu_d = 0.35; } },
            { label: 'Equilibrio estático (θ=22°)', apply: function (p) { p.thetaDeg = 22; p.mu_e = 0.50; } }
        ],
        defaultDuration: 4.0,

        solveAnalytical: function (p) {
            var th = degToRad(p.thetaDeg);
            var sinT = Math.sin(th), cosT = Math.cos(th);
            var N = p.m * p.g * cosT;
            var Fpar = p.m * p.g * sinT;
            var frMax = p.mu_e * N;
            var slips = Fpar > frMax + 1e-9;
            var muEff = Math.min(p.mu_d, p.mu_e);
            var fr = slips ? muEff * N : Fpar;
            var a = slips ? Math.max(0, p.g * (sinT - muEff * cosT)) : 0;
            var tEnd = (slips && a > 1e-9) ? Math.sqrt(2 * p.L / a) : NaN;
            var vEnd = (slips && a > 1e-9) ? a * tEnd : 0;
            var thetaCrit = radToDeg(Math.atan(p.mu_e));
            return {
                steps: [
                    {
                        title: '1. Fuerza normal y umbral estático',
                        formula: 'N = mg cos θ = ' + fmt(N, 2) + ' N  ;  f_r,máx = μ_e N = ' + fmt(frMax, 2) + ' N',
                        detail: 'Ángulo crítico θ_crít = atan(μ_e) = ' + fmt(thetaCrit, 2) + '°  |  mg sin θ = ' + fmt(Fpar, 2) + ' N'
                    },
                    {
                        title: slips ? '2. ¡Vence el estático! Aceleración cinética' : '2. Equilibrio estático (no desliza)',
                        formula: slips ? 'a = g (sin θ − μ_d cos θ)' : 'a = 0  ;  f_r = mg sin θ ≤ μ_e N',
                        detail: slips ? ('a = ' + fmt(a, 4) + ' m/s²  ;  f_r,cin = ' + fmt(fr, 2) + ' N') : ('Permanece quieto con f_r = ' + fmt(fr, 2) + ' N')
                    },
                    {
                        title: '3. Tiempo y velocidad al pie de la rampa',
                        formula: slips ? 't = √(2L/a)  ;  v_final = a·t' : '—',
                        detail: slips ? ('t_llegada = ' + fmt(tEnd, 3) + ' s  ;  v_final = ' + fmt(vEnd, 3) + ' m/s') : 'El bloque no llega a deslizar.'
                    }
                ],
                values: { N: N, fr: fr, a: a, tEnd: tEnd, vEnd: vEnd, slips: slips, thetaCrit: thetaCrit }
            };
        },

        createSimulation: function (p) {
            var ana = f_p2.solveAnalytical(p).values;
            var th = degToRad(p.thetaDeg);
            var sinT = Math.sin(th), cosT = Math.cos(th);
            var h = p.L * sinT;
            var xBase = p.L * cosT;
            var y = [0, 0]; // [s, v]
            var t = 0;
            var finished = false;
            var tArrived = NaN;

            function derivs(st) { return [ana.slips ? st[1] : 0, ana.a]; }

            return {
                step: function (dt) {
                    if (finished || !ana.slips) { t += dt; return; }
                    var yNext = rk4Integrate(y, t, dt, 0.005, derivs);
                    if (yNext[0] >= p.L) {
                        var ev = bisectEvent(y, t, dt, derivs, function (st) { return st[0] - p.L; }, 22);
                        y = ev.yEvent;
                        y[0] = p.L;
                        t = ev.tEvent;
                        tArrived = t;
                        finished = true;
                    } else {
                        y = yNext;
                        t += dt;
                    }
                },
                getObservables: function () {
                    var height = Math.max(0, (p.L - y[0]) * sinT);
                    var Ec = 0.5 * p.m * y[1] * y[1];
                    var Ep = p.m * p.g * height;
                    return {
                        t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                        Px: p.m * y[1] * cosT, Py: -p.m * y[1] * sinT, P: p.m * y[1], Lz: 0,
                        cm: { x: y[0] * cosT, y: height },
                        finished: finished,
                        extra: [
                            { label: 'Estado', value: ana.slips ? (finished ? 'En la base' : 'Deslizando') : 'Reposo estático' },
                            { label: 'Recorrido s', value: fmt(y[0], 2) + ' / ' + fmt(p.L, 1) + ' m' },
                            { label: 'f_r actual', value: fmt(ana.fr, 2) + ' N' }
                        ]
                    };
                },
                compareWithSim: function () {
                    return [
                        { label: 'Aceleración a', unit: 'm/s²', theo: ana.a, sim: ana.slips ? ana.a : 0 },
                        { label: 't_llegada', unit: 's', theo: ana.tEnd, sim: tArrived },
                        { label: 'v_final', unit: 'm/s', theo: ana.vEnd, sim: finished ? y[1] : NaN },
                        { label: 'Trabajo f_r', unit: 'J', theo: -ana.fr * y[0], sim: (0.5 * p.m * y[1] * y[1] + p.m * p.g * Math.max(0, (p.L - y[0]) * sinT)) - p.m * p.g * h }
                    ];
                },
                getRenderState: function () {
                    var bw = 0.9, bh = 0.55;
                    var nx = sinT, ny = cosT; // outward normal from ramp
                    var tx = cosT, ty = -sinT; // down-slope tangent
                    var cx = y[0] * tx + (bh * 0.5) * nx;
                    var cy = h + y[0] * ty + (bh * 0.5) * ny;
                    var fScale = 0.04;
                    return {
                        bounds: { minX: -1, maxX: xBase + 1.5, minY: -0.8, maxY: Math.max(3, h + 1.8) },
                        polys: [
                            { pts: [{ x: 0, y: 0 }, { x: xBase, y: 0 }, { x: 0, y: h }], fill: 'rgba(51, 65, 85, 0.35)', stroke: '#475569', lineWidth: 2 }
                        ],
                        lines: [
                            { x1: -1, y1: 0, x2: xBase + 1.5, y2: 0, color: '#334155', width: 3 }
                        ],
                        boxes: [
                            { x: cx, y: cy, w: bw, h: bh, angle: -th, color: ana.slips ? '#38bdf8' : '#4ade80', label: 'm=' + p.m + 'kg' }
                        ],
                        vectors: [
                            { type: 'force', x: cx, y: cy, vx: 0, vy: -p.m * p.g * fScale, color: '#ef4444', label: 'mg' },
                            { type: 'force', x: cx, y: cy, vx: ana.N * nx * fScale, vy: ana.N * ny * fScale, color: '#a78bfa', label: 'N' },
                            { type: 'tension', x: cx - (bh * 0.5) * nx, y: cy - (bh * 0.5) * ny, vx: -ana.fr * tx * fScale, vy: -ana.fr * ty * fScale, color: '#fbbf24', label: 'f_r' },
                            { type: 'velocity', x: cx, y: cy, vx: y[1] * tx * 0.25, vy: y[1] * ty * 0.25, color: '#22d3ee', label: 'v' }
                        ],
                        banner: ana.slips
                            ? 'Desliza (θ=' + p.thetaDeg + '° > θ_crít=' + fmt(ana.thetaCrit, 1) + '°): f_r disipa energía mecánica'
                            : 'Equilibrio estático (θ=' + p.thetaDeg + '° ≤ θ_crít=' + fmt(ana.thetaCrit, 1) + '°): f_r = mg sin θ'
                    };
                }
            };
        }
    };

    // =========================================================================
    // F3 — Oscilaciones y Energía: Oscilador Armónico Masa-Muelle
    // =========================================================================
    var f_p3 = {
        id: 'f_p3',
        topic: 'fundamentals',
        level: 3,
        code: 'T3 · F3',
        title: 'Oscilador Armónico Masa-Muelle (M.A.S.)',
        subtitle: 'Intercambio continuo Ec ↔ Ep elástica y efecto del amortiguamiento',
        renderMode: '2d',
        learningGoal: 'Dominar la frecuencia propia ω₀ = √(k/m), el periodo T = 2π√(m/k) y la conservación de E = ½kA² frente al amortiguamiento viscoso.',
        guideFromZero: [
            '<strong>1. Ley de Hooke:</strong> La fuerza recuperadora es <code>F = −k x</code>, que da la EDO <code>m ẍ + c ẋ + k x = 0</code>.',
            '<strong>2. Sin amortiguamiento (c = 0):</strong> Oscila con <code>x(t) = A cos(ω₀ t)</code>, donde <code>ω₀ = √(k/m)</code> y <code>v_máx = ω₀ A</code>.',
            '<strong>3. Balance energético:</strong> Cuando pasa por <code>x = 0</code>, toda la energía es cinética; en los extremos <code>±A</code>, toda es potencial elástica.'
        ],
        statement: 'Un bloque de m = 2 kg unido a un muelle horizontal de constante k = 18 N/m se separa A = 1.2 m del equilibrio y se suelta sin rozamiento (c = 0). Calcular ω₀, periodo T, velocidad máxima y energía mecánica total.',
        paramsSchema: [
            { key: 'm', label: 'Masa m', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 2 },
            { key: 'k', label: 'Constante muelle k', unit: 'N/m', min: 2, max: 60, step: 1, def: 18 },
            { key: 'A', label: 'Elongación inicial A', unit: 'm', min: 0.2, max: 2.0, step: 0.1, def: 1.2 },
            { key: 'c', label: 'Amortiguamiento c', unit: 'N·s/m', min: 0, max: 4.0, step: 0.1, def: 0 }
        ],
        quickActions: [
            { label: 'M.A.S. ideal (c = 0)', apply: function (p) { p.c = 0; } },
            { label: 'Subamortiguado (c = 0.8)', apply: function (p) { p.c = 0.8; } }
        ],
        defaultDuration: 8.0,

        solveAnalytical: function (p) {
            var w0 = Math.sqrt(p.k / p.m);
            var T = (2 * Math.PI) / w0;
            var vMax = w0 * p.A;
            var E0 = 0.5 * p.k * p.A * p.A;
            var gamma = p.c / (2 * p.m);
            var wd = w0 > gamma ? Math.sqrt(w0 * w0 - gamma * gamma) : 0;
            return {
                steps: [
                    {
                        title: '1. Pulsación propia y periodo',
                        formula: 'ω₀ = √(k / m)  ;  T = 2π / ω₀',
                        detail: 'ω₀ = ' + fmt(w0, 4) + ' rad/s  ;  T = ' + fmt(T, 4) + ' s' + (p.c > 0 ? ('  (ω_amort = ' + fmt(wd, 3) + ' rad/s)') : '')
                    },
                    {
                        title: '2. Velocidad máxima (en x = 0 sin rozamiento)',
                        formula: 'v_máx = ω₀ · A',
                        detail: 'v_máx = ' + fmt(vMax, 4) + ' m/s  (aceleración máxima: a_máx = ω₀² A = ' + fmt(w0 * w0 * p.A, 2) + ' m/s²)'
                    },
                    {
                        title: '3. Energía mecánica inicial',
                        formula: 'E₀ = ½ k A²',
                        detail: 'E₀ = ' + fmt(E0, 3) + ' J' + (p.c === 0 ? ' (conservada exactamente)' : ' (decae exponencialmente por amortiguamiento)')
                    }
                ],
                values: { w0: w0, T: T, vMax: vMax, E0: E0, gamma: gamma, wd: wd }
            };
        },

        createSimulation: function (p) {
            var ana = f_p3.solveAnalytical(p).values;
            var y = [p.A, 0]; // [x, v]
            var t = 0;
            var simVMax = 0;
            var simT = NaN;
            var eqAnchor = -2.4;

            function derivs(st) {
                return [st[1], (-p.k * st[0] - p.c * st[1]) / p.m];
            }

            return {
                step: function (dt) {
                    var yPrev = y;
                    y = rk4Integrate(y, t, dt, 0.004, derivs);
                    t += dt;
                    simVMax = Math.max(simVMax, Math.abs(y[1]));
                    if (isNaN(simT) && t > 0.25 * ana.T && yPrev[1] > 0 && y[1] <= 0) {
                        var ev = bisectEvent(yPrev, t - dt, dt, derivs, function (st) { return st[1]; }, 22);
                        simT = ev.tEvent;
                    }
                },
                getObservables: function () {
                    var Ec = 0.5 * p.m * y[1] * y[1];
                    var Ep = 0.5 * p.k * y[0] * y[0];
                    return {
                        t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                        Px: p.m * y[1], Py: 0, P: Math.abs(p.m * y[1]), Lz: 0,
                        cm: { x: y[0], y: 0 },
                        finished: false,
                        extra: [
                            { label: 'Elongación x(t)', value: fmt(y[0], 3) + ' m' },
                            { label: 'Velocidad v(t)', value: fmt(y[1], 3) + ' m/s' },
                            { label: 'Fuerza F = −kx', value: fmt(-p.k * y[0], 2) + ' N' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var Ec = 0.5 * p.m * y[1] * y[1];
                    var Ep = 0.5 * p.k * y[0] * y[0];
                    var xTheo = p.c === 0 ? p.A * Math.cos(ana.w0 * t)
                        : (ana.wd > 0 ? p.A * Math.exp(-ana.gamma * t) * (Math.cos(ana.wd * t) + (ana.gamma / ana.wd) * Math.sin(ana.wd * t)) : NaN);
                    return [
                        { label: 'x(t)', unit: 'm', theo: xTheo, sim: y[0] },
                        { label: 'Periodo T', unit: 's', theo: p.c === 0 ? ana.T : (ana.wd > 0 ? 2 * Math.PI / ana.wd : NaN), sim: simT },
                        { label: 'v_máx', unit: 'm/s', theo: p.c === 0 ? ana.vMax : NaN, sim: t >= 0.3 * ana.T ? simVMax : NaN },
                        { label: 'E_total', unit: 'J', theo: p.c === 0 ? ana.E0 : NaN, sim: Ec + Ep }
                    ];
                },
                getRenderState: function () {
                    var wallX = eqAnchor - p.A - 0.6;
                    var span = Math.max(2.6, p.A * 1.45);
                    var fk = -p.k * y[0];
                    return {
                        bounds: { minX: wallX - 0.4, maxX: span + 1.0, minY: -1.4, maxY: 1.8 },
                        lines: [
                            { x1: wallX, y1: -0.6, x2: wallX, y2: 1.2, color: '#64748b', width: 5 },
                            { x1: wallX, y1: -0.35, x2: span + 0.8, y2: -0.35, color: '#334155', width: 3 },
                            { x1: 0, y1: -0.5, x2: 0, y2: 0.9, color: 'rgba(148,163,184,0.45)', width: 1.5, dash: [4, 4] }
                        ],
                        springs: [
                            { x1: wallX, y1: 0, x2: y[0] - 0.35, y2: 0, coils: 12, ampPx: 9, color: '#a78bfa' }
                        ],
                        boxes: [
                            { x: y[0], y: 0, w: 0.7, h: 0.7, color: '#38bdf8', label: 'm=' + p.m + 'kg' }
                        ],
                        labels: [
                            { x: 0, y: 1.02, text: 'x = 0 (Equilibrio)', color: '#94a3b8', align: 'center' }
                        ],
                        vectors: [
                            { type: 'force', x: y[0], y: 0.42, vx: fk * 0.04, vy: 0, color: '#fb7185', label: 'F_k' },
                            { type: 'velocity', x: y[0], y: 0, vx: y[1] * 0.18, vy: 0, color: '#22d3ee', label: 'v' }
                        ],
                        banner: p.c === 0
                            ? 'M.A.S. ideal: T = ' + fmt(ana.T, 3) + ' s | E = ½kA² = ' + fmt(ana.E0, 2) + ' J constante'
                            : 'Oscilador amortiguado (c = ' + p.c + ' N·s/m): E disipándose'
                    };
                }
            };
        }
    };

    // =========================================================================
    // F4 — Gravitación: Órbita de Kepler (Fuerza Central y Conservación de L)
    // =========================================================================
    var f_p4 = {
        id: 'f_p4',
        topic: 'fundamentals',
        level: 4,
        code: 'T4 · F4',
        title: 'Órbita Gravitatoria (Leyes de Kepler)',
        subtitle: 'Fuerza central inversa al cuadrado: conservación de L y E con RK4',
        renderMode: '2d',
        learningGoal: 'Comprobar que una fuerza central (τ = r × F = 0) conserva el momento angular L = r × p exacto en toda la órbita (2ª Ley de Kepler: áreas iguales en tiempos iguales).',
        guideFromZero: [
            '<strong>1. Ley de Gravitación:</strong> En unidades astronómicas normalizadas (<code>μ = GM = 4π² AU³/año²</code>), <code>a = −μ r / r³</code>.',
            '<strong>2. Conservación de L:</strong> Al ser fuerza radial, el par es nulo y <code>L_z = m (x vᵧ − y vₓ) = constante</code>.',
            '<strong>3. Tipo de órbita:</strong> Depende de la velocidad inicial <code>v₀</code> en el perihelio r₀: <code>v_circ = √(μ/r₀)</code> (circular), <code>v₀ &lt; √2 v_circ</code> (elipse).'
        ],
        statement: 'Un planeta de masa m = 1 (unidad normalizada) parte del perihelio r₀ = 1.0 AU con velocidad tangencial v₀ = 7.2 AU/año (GM = 4π² ≈ 39.478 AU³/año²). Integrar la órbita mediante RK4 y verificar la conservación de L_z y E.',
        paramsSchema: [
            { key: 'r0', label: 'Distancia inicial r₀', unit: 'AU', min: 0.5, max: 2.0, step: 0.1, def: 1.0 },
            { key: 'vFactor', label: 'Factor velocidad v₀ / v_circ', unit: '', min: 0.65, max: 1.32, step: 0.02, def: 1.15 }
        ],
        quickActions: [
            { label: 'Órbita circular (1.00)', apply: function (p) { p.vFactor = 1.0; p.r0 = 1.0; } },
            { label: 'Elipse excéntrica (1.20)', apply: function (p) { p.vFactor = 1.20; p.r0 = 1.0; } },
            { label: 'Cometa rápido (1.30)', apply: function (p) { p.vFactor = 1.30; p.r0 = 0.8; } }
        ],
        defaultDuration: 6.0,

        solveAnalytical: function (p) {
            var mu = 4 * Math.PI * Math.PI; // AU^3 / yr^2
            var vCirc = Math.sqrt(mu / p.r0);
            var v0 = p.vFactor * vCirc;
            var eps = 0.5 * v0 * v0 - mu / p.r0; // specific energy (< 0 bound)
            var a = -mu / (2 * eps); // semi-major axis
            var Lz = p.r0 * v0;
            var e = Math.sqrt(Math.max(0, 1 + (2 * eps * Lz * Lz) / (mu * mu)));
            var T = a > 0 ? Math.sqrt(a * a * a) : NaN; // Kepler 3rd law in AU/yr
            var rMax = e < 1 ? a * (1 + e) : NaN;
            return {
                steps: [
                    {
                        title: '1. Velocidad circular y de lanzamiento',
                        formula: 'v_circ = √(GM / r₀)  ;  v₀ = ' + fmt(p.vFactor, 2) + ' v_circ',
                        detail: 'v_circ = ' + fmt(vCirc, 3) + ' AU/año  →  v₀ = ' + fmt(v0, 3) + ' AU/año'
                    },
                    {
                        title: '2. Energía específica y excentricidad',
                        formula: 'E/m = ½v₀² − GM/r₀ = −GM/(2a)  ;  L_z/m = r₀ v₀',
                        detail: 'E/m = ' + fmt(eps, 3) + ' AU²/año²  ;  L_z/m = ' + fmt(Lz, 3) + ' AU²/año  ;  e = ' + fmt(e, 4)
                    },
                    {
                        title: '3. 3ª Ley de Kepler (periodo orbital)',
                        formula: 'T = a^(3/2) años  ;  r_máx = a(1 + e)',
                        detail: 'a = ' + fmt(a, 3) + ' AU  ;  T = ' + fmt(T, 3) + ' años  ;  r_afelio = ' + fmt(rMax, 3) + ' AU'
                    }
                ],
                values: { mu: mu, vCirc: vCirc, v0: v0, eps: eps, a: a, Lz: Lz, e: e, T: T, rMax: rMax }
            };
        },

        createSimulation: function (p) {
            var ana = f_p4.solveAnalytical(p).values;
            var y = [p.r0, 0, 0, ana.v0]; // [x, y, vx, vy]
            var t = 0;
            var simT = NaN;
            var trails = [{ x: p.r0, y: 0 }];

            function derivs(st) {
                var r2 = st[0] * st[0] + st[1] * st[1];
                var r3 = r2 * Math.sqrt(r2);
                var inv = r3 > 1e-12 ? -ana.mu / r3 : 0;
                return [st[2], st[3], inv * st[0], inv * st[1]];
            }

            return {
                step: function (dt) {
                    var yPrev = y;
                    y = rk4Integrate(y, t, dt, 0.002, derivs);
                    t += dt;
                    if (isNaN(simT) && t > 0.25 * ana.T && yPrev[1] < 0 && y[1] >= 0 && y[0] > 0) {
                        var ev = bisectEvent(yPrev, t - dt, dt, derivs, function (st) { return st[1]; }, 22);
                        simT = ev.tEvent;
                    }
                    if (trails.length < 1500) trails.push({ x: y[0], y: y[1] });
                },
                getObservables: function () {
                    var r = Math.hypot(y[0], y[1]);
                    var v = Math.hypot(y[2], y[3]);
                    var Ec = 0.5 * v * v;
                    var Ep = -ana.mu / Math.max(0.05, r);
                    var Lz = y[0] * y[3] - y[1] * y[2];
                    return {
                        t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                        Px: y[2], Py: y[3], P: v, Lz: Lz,
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'Radio r(t)', value: fmt(r, 3) + ' AU' },
                            { label: 'Rapidez v(t)', value: fmt(v, 3) + ' AU/año' },
                            { label: 'Excentricidad e', value: fmt(ana.e, 3) }
                        ]
                    };
                },
                compareWithSim: function () {
                    var r = Math.hypot(y[0], y[1]);
                    var v2 = y[2] * y[2] + y[3] * y[3];
                    var Etot = 0.5 * v2 - ana.mu / r;
                    var Lz = y[0] * y[3] - y[1] * y[2];
                    return [
                        { label: 'L_z (constante)', unit: 'AU²/año', theo: ana.Lz, sim: Lz },
                        { label: 'E_mecánica/m', unit: 'AU²/año²', theo: ana.eps, sim: Etot },
                        { label: 'Periodo T', unit: 'años', theo: ana.T, sim: simT }
                    ];
                },
                getRenderState: function () {
                    var rSpan = Math.max(1.8, isFinite(ana.rMax) ? ana.rMax * 1.15 : 3.5);
                    var r = Math.hypot(y[0], y[1]);
                    var r3 = Math.pow(Math.max(0.1, r), 3);
                    var ax = (-ana.mu * y[0]) / r3;
                    var ay = (-ana.mu * y[1]) / r3;
                    return {
                        bounds: { minX: -rSpan, maxX: rSpan, minY: -rSpan * 0.85, maxY: rSpan * 0.85 },
                        circles: [
                            { x: 0, y: 0, r: 0.14, fill: '#fbbf24', stroke: '#fde047', lineWidth: 2 }
                        ],
                        trails: [
                            { pts: trails, color: 'rgba(56, 189, 248, 0.7)', width: 2 }
                        ],
                        masses: [
                            { x: y[0], y: y[1], r: 0.08, color: '#38bdf8', label: 'Planeta' }
                        ],
                        labels: [
                            { x: 0, y: -0.26, text: 'Sol (foco)', color: '#fbbf24', align: 'center' }
                        ],
                        vectors: [
                            { type: 'velocity', x: y[0], y: y[1], vx: y[2] * 0.07, vy: y[3] * 0.07, color: '#22d3ee', label: 'v' },
                            { type: 'force', x: y[0], y: y[1], vx: ax * 0.015, vy: ay * 0.015, color: '#fb7185', label: 'F_g' }
                        ],
                        banner: 'Fuerza central F_g ∥ r → L_z = ' + fmt(y[0] * y[3] - y[1] * y[2], 3) + ' constante (e = ' + fmt(ana.e, 3) + ')'
                    };
                }
            };
        }
    };

    return [f_p1, f_p2, f_p3, f_p4];
}));
