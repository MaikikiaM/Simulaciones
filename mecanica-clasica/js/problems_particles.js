/**
 * problems_particles.js — 6 simulaciones progresivas de Dinámica de Sistemas de Partículas
 * Basado en "Problemas T5.pdf" (Fundamentos de Física I).
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory(require('./rk4.js'));
    } else {
        root.ParticleProblems = factory(root.PhysicsRK4);
    }
}(typeof self !== 'undefined' ? self : this, function (RK4) {
    'use strict';

    var rk4Integrate = RK4.rk4Integrate;
    var bisectEvent = RK4.bisectEvent;
    var degToRad = RK4.degToRad;
    var radToDeg = RK4.radToDeg;
    var fmt = RK4.fmt;

    // =========================================================================
    // NIVEL 1 — T5 · P3: Teorema del Centro de Masas bajo Fuerzas Externas
    // =========================================================================
    var t5_p3 = {
        id: 't5_p3',
        topic: 'particles',
        level: 1,
        code: 'T5 · P3',
        title: 'Centro de Masas bajo Fuerzas Externas',
        subtitle: 'Movimiento del CM vs rotación interna de la barra',
        renderMode: '2d',
        learningGoal: 'Comprender que las fuerzas internas (tensión de la barra) se anulan por pares y NO afectan al movimiento del centro de masas: M·a_CM = ΣF_ext.',
        guideFromZero: [
            '<strong>1. Posición del Centro de Masas (CM):</strong> Es el promedio ponderado por masa de las posiciones: <code>R_CM = (m₁ r₁ + m₂ r₂) / (m₁ + m₂)</code>.',
            '<strong>2. Teorema del CM:</strong> Al derivar dos veces y aplicar la 3ª Ley de Newton a la barra (<code>T₁ + T₂ = 0</code>), solo sobreviven las fuerzas externas: <code>M · a_CM = F₁ + F₂</code>.',
            '<strong>3. Qué observar en el simulador:</strong> Aunque la barra gira y cada masa describe una trayectoria compleja, el diamante dorado (CM) acelera en línea recta exacta bajo <code>F_ext = F₁ + F₂</code>.'
        ],
        statement: 'Las masas m₁ = 10 kg y m₂ = 6 kg están unidas por una barra de masa despreciable, situadas inicialmente en reposo en (0, 3) m y (4, 0) m. Son sometidas a las fuerzas F₁ = 8 i N sobre m₁ y F₂ = 6 j N sobre m₂. (a) Expresen las coordenadas del CM en función del tiempo. (b) Expresen el momento lineal total en función del tiempo.',
        paramsSchema: [
            { key: 'm1', label: 'Masa m₁', unit: 'kg', min: 1, max: 25, step: 0.5, def: 10 },
            { key: 'm2', label: 'Masa m₂', unit: 'kg', min: 1, max: 25, step: 0.5, def: 6 },
            { key: 'y1', label: 'Posición inicial y₁ de m₁', unit: 'm', min: 0.5, max: 8, step: 0.5, def: 3 },
            { key: 'x2', label: 'Posición inicial x₂ de m₂', unit: 'm', min: 0.5, max: 8, step: 0.5, def: 4 },
            { key: 'F1x', label: 'Fuerza F₁ₓ sobre m₁', unit: 'N', min: -20, max: 20, step: 0.5, def: 8 },
            { key: 'F2y', label: 'Fuerza F₂ᵧ sobre m₂', unit: 'N', min: -20, max: 20, step: 0.5, def: 6 }
        ],
        defaultDuration: 6.0,

        solveAnalytical: function (p) {
            var M = p.m1 + p.m2;
            var xcm0 = (p.m2 * p.x2) / M;
            var ycm0 = (p.m1 * p.y1) / M;
            var ax = p.F1x / M;
            var ay = p.F2y / M;
            var aMag = Math.hypot(ax, ay);
            var L = Math.hypot(p.x2, p.y1);
            var tRef = 4.0;
            var xcm4 = xcm0 + 0.5 * ax * tRef * tRef;
            var ycm4 = ycm0 + 0.5 * ay * tRef * tRef;
            var Px4 = p.F1x * tRef;
            var Py4 = p.F2y * tRef;

            return {
                steps: [
                    {
                        title: '1. Masa total y posición inicial del CM',
                        formula: 'M = m₁ + m₂ = ' + fmt(M, 2) + ' kg',
                        detail: 'x_CM(0) = (m₁·0 + m₂·x₂)/M = ' + fmt(xcm0, 4) + ' m ; y_CM(0) = (m₁·y₁ + m₂·0)/M = ' + fmt(ycm0, 4) + ' m'
                    },
                    {
                        title: '2. Aceleración del CM (las tensiones internas se anulan)',
                        formula: 'a_CM = (F₁ + F₂) / M = (' + fmt(p.F1x, 2) + ' i + ' + fmt(p.F2y, 2) + ' j) / ' + fmt(M, 2),
                        detail: 'a_CM = (' + fmt(ax, 4) + ' i + ' + fmt(ay, 4) + ' j) m/s²  (|a_CM| = ' + fmt(aMag, 4) + ' m/s²)'
                    },
                    {
                        title: '(a) Coordenadas del CM en función del tiempo',
                        formula: 'R_CM(t) = R_CM(0) + ½ a_CM t²',
                        detail: 'x_CM(t) = ' + fmt(xcm0, 3) + ' + ' + fmt(0.5 * ax, 4) + '·t² m  ;  y_CM(t) = ' + fmt(ycm0, 3) + ' + ' + fmt(0.5 * ay, 4) + '·t² m  (en t=4 s: (' + fmt(xcm4, 2) + ', ' + fmt(ycm4, 2) + ') m)'
                    },
                    {
                        title: '(b) Momento lineal total en función del tiempo',
                        formula: 'P(t) = M·v_CM(t) = F_ext · t',
                        detail: 'P(t) = (' + fmt(p.F1x, 2) + '·t) i + (' + fmt(p.F2y, 2) + '·t) j  kg·m/s  (en t=4 s: (' + fmt(Px4, 2) + ', ' + fmt(Py4, 2) + ') kg·m/s, longitud barra L = ' + fmt(L, 3) + ' m)'
                    }
                ],
                metricsAtTime: function (t) {
                    return {
                        xcm: xcm0 + 0.5 * ax * t * t,
                        ycm: ycm0 + 0.5 * ay * t * t,
                        Px: p.F1x * t,
                        Py: p.F2y * t,
                        P: Math.hypot(p.F1x, p.F2y) * t
                    };
                }
            };
        },

        createSimulation: function (p) {
            var M = p.m1 + p.m2;
            var L = Math.hypot(p.x2, p.y1);
            var d1 = (p.m2 / M) * L;
            var d2 = (p.m1 / M) * L;
            var mu = (p.m1 * p.m2) / M;
            var Icm = mu * L * L;
            var xcm0 = (p.m2 * p.x2) / M;
            var ycm0 = (p.m1 * p.y1) / M;
            var theta0 = Math.atan2(-p.y1, p.x2);

            // y = [Xcm, Ycm, Vxcm, Vycm, theta, omega]
            var y = [xcm0, ycm0, 0, 0, theta0, 0];
            var t = 0;
            var trails = { cm: [{ x: xcm0, y: ycm0 }], m1: [{ x: 0, y: p.y1 }], m2: [{ x: p.x2, y: 0 }] };

            function derivs(st) {
                var th = st[4], w = st[5];
                var ux = Math.cos(th), uy = Math.sin(th);
                var r1x = -d1 * ux, r1y = -d1 * uy;
                var r2x = d2 * ux, r2y = d2 * uy;
                var tau = (r1x * 0 - r1y * p.F1x) + (r2x * p.F2y - r2y * 0);
                var alpha = Icm > 1e-12 ? tau / Icm : 0;
                return [st[2], st[3], p.F1x / M, p.F2y / M, w, alpha];
            }

            function positions(st) {
                var ux = Math.cos(st[4]), uy = Math.sin(st[4]);
                return {
                    x1: st[0] - d1 * ux,
                    y1: st[1] - d1 * uy,
                    x2: st[0] + d2 * ux,
                    y2: st[1] + d2 * uy
                };
            }

            return {
                step: function (dt) {
                    y = rk4Integrate(y, t, dt, 0.005, derivs);
                    t += dt;
                    var pos = positions(y);
                    if (trails.cm.length < 1800) {
                        trails.cm.push({ x: y[0], y: y[1] });
                        trails.m1.push({ x: pos.x1, y: pos.y1 });
                        trails.m2.push({ x: pos.x2, y: pos.y2 });
                    }
                },
                getObservables: function () {
                    var EcTras = 0.5 * M * (y[2] * y[2] + y[3] * y[3]);
                    var EcRot = 0.5 * Icm * y[5] * y[5];
                    var Px = M * y[2], Py = M * y[3];
                    var Lcm = Icm * y[5];
                    return {
                        t: t,
                        Ec: EcTras + EcRot,
                        Etras: EcTras,
                        Erot: EcRot,
                        Ep: 0,
                        Etot: EcTras + EcRot,
                        Px: Px,
                        Py: Py,
                        P: Math.hypot(Px, Py),
                        Lz: Lcm,
                        cm: { x: y[0], y: y[1] },
                        finished: false,
                        extra: [
                            { label: 'x_CM(t)', value: fmt(y[0], 3) + ' m' },
                            { label: 'y_CM(t)', value: fmt(y[1], 3) + ' m' },
                            { label: 'ω barra', value: fmt(y[5], 3) + ' rad/s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t5_p3.solveAnalytical(p).metricsAtTime(t);
                    return [
                        { label: 'x_CM(t)', unit: 'm', theo: ana.xcm, sim: y[0] },
                        { label: 'y_CM(t)', unit: 'm', theo: ana.ycm, sim: y[1] },
                        { label: 'Pₓ(t)', unit: 'kg·m/s', theo: ana.Px, sim: M * y[2] },
                        { label: 'Pᵧ(t)', unit: 'kg·m/s', theo: ana.Py, sim: M * y[3] }
                    ];
                },
                getRenderState: function () {
                    var pos = positions(y);
                    var ux = Math.cos(y[4]), uy = Math.sin(y[4]);
                    // Velocities of m1 and m2
                    var v1x = y[2] - y[5] * (-d1 * uy);
                    var v1y = y[3] + y[5] * (-d1 * ux);
                    var v2x = y[2] - y[5] * (d2 * uy);
                    var v2y = y[3] + y[5] * (d2 * ux);
                    // Internal rod tension T (acting +T*u on m1, -T*u on m2)
                    var F1u = p.F1x * ux;
                    var F2u = p.F2y * uy;
                    var Tmag = mu * (L * y[5] * y[5] + F2u / p.m2 - F1u / p.m1);

                    return {
                        bounds: { minX: -2, maxX: Math.max(12, y[0] + 6), minY: -2, maxY: Math.max(10, y[1] + 5) },
                        trails: [
                            { pts: trails.m1, color: 'rgba(56, 189, 248, 0.45)', width: 1.5, dash: [4, 4] },
                            { pts: trails.m2, color: 'rgba(244, 114, 182, 0.45)', width: 1.5, dash: [4, 4] },
                            { pts: trails.cm, color: '#fbbf24', width: 2.5 }
                        ],
                        rods: [
                            { x1: pos.x1, y1: pos.y1, x2: pos.x2, y2: pos.y2, color: '#94a3b8', width: 4 }
                        ],
                        masses: [
                            { x: pos.x1, y: pos.y1, r: 0.25 + 0.012 * p.m1, color: '#38bdf8', label: 'm₁ (' + p.m1 + ' kg)' },
                            { x: pos.x2, y: pos.y2, r: 0.25 + 0.012 * p.m2, color: '#f472b6', label: 'm₂ (' + p.m2 + ' kg)' }
                        ],
                        cm: { x: y[0], y: y[1], label: 'CM' },
                        vectors: [
                            { type: 'force', x: pos.x1, y: pos.y1, vx: p.F1x * 0.25, vy: 0, color: '#fb7185', label: 'F₁' },
                            { type: 'force', x: pos.x2, y: pos.y2, vx: 0, vy: p.F2y * 0.25, color: '#fb7185', label: 'F₂' },
                            { type: 'tension', x: pos.x1, y: pos.y1, vx: Tmag * ux * 0.25, vy: Tmag * uy * 0.25, color: '#a78bfa', label: 'T₁' },
                            { type: 'tension', x: pos.x2, y: pos.y2, vx: -Tmag * ux * 0.25, vy: -Tmag * uy * 0.25, color: '#a78bfa', label: 'T₂' },
                            { type: 'velocity', x: pos.x1, y: pos.y1, vx: v1x * 0.4, vy: v1y * 0.4, color: '#22d3ee', label: 'v₁' },
                            { type: 'velocity', x: pos.x2, y: pos.y2, vx: v2x * 0.4, vy: v2y * 0.4, color: '#22d3ee', label: 'v₂' },
                            { type: 'momentum', x: y[0], y: y[1], vx: M * y[2] * 0.08, vy: M * y[3] * 0.08, color: '#fbbf24', label: 'P_total' }
                        ],
                        banner: 'F_ext = (' + fmt(p.F1x, 1) + ', ' + fmt(p.F2y, 1) + ') N → CM acelera en línea recta'
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 2 — T5 · P9: Péndulos de Marfil (Choques Elásticos 1D)
    // =========================================================================
    var t5_p9 = {
        id: 't5_p9',
        topic: 'particles',
        level: 2,
        code: 'T5 · P9',
        title: 'Bolas de Marfil en Péndulo (Choques 1D)',
        subtitle: 'Conservación de p y Ec en choques elásticos con 3 razones de masa',
        renderMode: '2d',
        learningGoal: 'Dominar la conservación simultánea de momento lineal y energía cinética en choques elásticos 1D y ver cómo la relación m₁/m₂ determina el rebote.',
        guideFromZero: [
            '<strong>1. Antes del choque:</strong> B₁ cae desde θ₀ = 60°: por conservación de la energía, <code>½ m₁ v₁² = m₁ g l (1 − cos θ₀)</code>.',
            '<strong>2. En el choque elástico (e = 1):</strong> Se conservan <code>p</code> y <code>E_c</code>: <code>v₁f = (m₁ − m₂)/(m₁ + m₂) · v₁</code> y <code>v₂f = 2m₁/(m₁ + m₂) · v₁</code>.',
            '<strong>3. Tres regímenes:</strong> Si <code>m₂ = 2m₁</code>, B₁ rebota hacia atrás; si <code>m₁ = 2m₂</code>, ambas siguen adelante; si <code>m₁ = m₂</code>, intercambian velocidades (B₁ se detiene en seco).'
        ],
        statement: 'Dos bolas de marfil B₁ y B₂ (masas m₁, m₂) cuelgan de hilos de longitud l y se tocan en equilibrio vertical. Se separa B₁ un ángulo de 60° y se suelta, chocando elásticamente contra B₂ en reposo. Calcular para: 1) m₂ = 2m₁, 2) m₁ = 2m₂, 3) m₁ = m₂: (a) v₁ al llegar, (b) velocidades tras el choque, (c) alturas finales en el caso 3.',
        paramsSchema: [
            { key: 'm1', label: 'Masa bola B₁ (m₁)', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 1 },
            { key: 'm2', label: 'Masa bola B₂ (m₂)', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 2 },
            { key: 'l', label: 'Longitud de los hilos l', unit: 'm', min: 0.5, max: 4.0, step: 0.1, def: 1.5 },
            { key: 'theta0Deg', label: 'Ángulo inicial θ₀ de B₁', unit: '°', min: 10, max: 85, step: 1, def: 60 },
            { key: 'e', label: 'Coef. restitución e (1=elástico)', unit: '', min: 0, max: 1, step: 0.05, def: 1.0 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'Caso 1: m₂ = 2m₁', apply: function (p) { p.m1 = 1; p.m2 = 2; p.e = 1; p.theta0Deg = 60; } },
            { label: 'Caso 2: m₁ = 2m₂', apply: function (p) { p.m1 = 2; p.m2 = 1; p.e = 1; p.theta0Deg = 60; } },
            { label: 'Caso 3: m₁ = m₂', apply: function (p) { p.m1 = 1.5; p.m2 = 1.5; p.e = 1; p.theta0Deg = 60; } }
        ],
        defaultDuration: 8.0,

        solveAnalytical: function (p) {
            var th0 = degToRad(p.theta0Deg);
            var h0 = p.l * (1 - Math.cos(th0));
            var v1 = Math.sqrt(2 * p.g * h0);
            var sumM = p.m1 + p.m2;
            var v1f = ((p.m1 - p.e * p.m2) / sumM) * v1;
            var v2f = ((p.m1 * (1 + p.e)) / sumM) * v1;
            var h1f = (v1f * v1f) / (2 * p.g);
            var h2f = (v2f * v2f) / (2 * p.g);

            return {
                steps: [
                    {
                        title: '(a) Velocidad de B₁ justo antes del choque',
                        formula: 'v₁ = √[2 g l (1 − cos θ₀)]',
                        detail: 'h₀ = ' + fmt(h0, 4) + ' m → v₁ = ' + fmt(v1, 4) + ' m/s  (para θ₀=60°, v₁ = √(gl) = ' + fmt(Math.sqrt(p.g * p.l), 4) + ' m/s)'
                    },
                    {
                        title: '(b) Velocidades tras el choque (e = ' + fmt(p.e, 2) + ')',
                        formula: 'v₁f = (m₁ − e·m₂)/(m₁+m₂) · v₁  ;  v₂f = m₁(1+e)/(m₁+m₂) · v₁',
                        detail: 'v₁f = ' + fmt(v1f, 4) + ' m/s  ;  v₂f = ' + fmt(v2f, 4) + ' m/s'
                    },
                    {
                        title: '(c) Alturas máximas alcanzadas tras el choque',
                        formula: 'hᵢ = vᵢf² / (2g)',
                        detail: 'h₁f = ' + fmt(h1f, 4) + ' m (' + fmt(100 * (h0 > 0 ? h1f / h0 : 0), 1) + '% de h₀)  ;  h₂f = ' + fmt(h2f, 4) + ' m (' + fmt(100 * (h0 > 0 ? h2f / h0 : 0), 1) + '% de h₀)'
                    }
                ],
                values: { v1: v1, v1f: v1f, v2f: v2f, h1f: h1f, h2f: h2f, h0: h0 }
            };
        },

        createSimulation: function (p) {
            var th0 = degToRad(p.theta0Deg);
            // y = [theta1, omega1, theta2, omega2]
            var y = [-th0, 0, 0, 0];
            var t = 0;
            var collisions = 0;
            var firstImpact = null;
            var trails = { b1: [], b2: [] };
            var ballVisR = 0.09 * Math.max(1, p.l * 0.7);

            function derivs(st) {
                var k = p.g / p.l;
                return [st[1], -k * Math.sin(st[0]), st[3], -k * Math.sin(st[2])];
            }

            function resolveImpact(st, gap0) {
                var w1 = st[1], w2 = st[3];
                if (Math.abs(w1 - w2) <= 1e-9) return st;
                // Only resolve if approaching each other in the collision direction
                if (gap0 !== undefined) {
                    if (gap0 <= 0 && w1 <= w2) return st;
                    if (gap0 > 0 && w2 <= w1) return st;
                }
                var v1 = w1 * p.l, v2 = w2 * p.l;
                var sumM = p.m1 + p.m2;
                var v1f = ((p.m1 - p.e * p.m2) * v1 + p.m2 * (1 + p.e) * v2) / sumM;
                var v2f = (p.m1 * (1 + p.e) * v1 + (p.m2 - p.e * p.m1) * v2) / sumM;
                var out = st.slice();
                // Align theta1 and theta2 mod 2*pi cleanly at collision point
                var kTurn = Math.round((out[2] - out[0]) / (2 * Math.PI));
                out[2] = out[0] + kTurn * 2 * Math.PI;
                out[1] = v1f / p.l;
                out[3] = v2f / p.l;
                collisions += 1;
                if (!firstImpact) {
                    firstImpact = { t: t, v1Pre: v1, v1Post: v1f, v2Post: v2f };
                }
                return out;
            }

            function wrapAngle(ang) {
                var a = (ang + Math.PI) % (2 * Math.PI);
                if (a < 0) a += 2 * Math.PI;
                return a - Math.PI;
            }

            function subStepWithCollisions(dt) {
                var maxSub = 0.004;
                var nSteps = Math.max(1, Math.ceil(dt / maxSub));
                var h = dt / nSteps;
                for (var s = 0; s < nSteps; s++) {
                    var yNext = rk4Integrate(y, t, h, h, derivs);
                    var gap0 = wrapAngle(y[0] - y[2]);
                    var gap1 = wrapAngle(yNext[0] - yNext[2]);
                    var hitForward = (gap0 <= 0 && gap1 >= 0 && Math.abs(gap1 - gap0) < 1.0 && (y[1] - y[3] > 1e-8 || yNext[1] - yNext[3] > 1e-8));
                    var hitReverse = (gap0 >= 0 && gap1 <= 0 && Math.abs(gap1 - gap0) < 1.0 && (y[3] - y[1] > 1e-8 || yNext[3] - yNext[1] > 1e-8));
                    if (hitForward || hitReverse) {
                        var ev = bisectEvent(y, t, h, derivs, function (st) { return wrapAngle(st[0] - st[2]); }, 24);
                        var rem = h - ev.dtEvent;
                        t = ev.tEvent;
                        y = resolveImpact(ev.yEvent, hitForward ? -1 : 1);
                        if (rem > 1e-9) {
                            y = rk4Integrate(y, t, rem, rem, derivs);
                            t += rem;
                        }
                    } else {
                        y = yNext;
                        t += h;
                    }
                }
            }

            return {
                step: function (dt) {
                    subStepWithCollisions(dt);
                    var pivotOffset = ballVisR;
                    var x1 = -pivotOffset + p.l * Math.sin(y[0]);
                    var y1 = p.l - p.l * Math.cos(y[0]);
                    var x2 = pivotOffset + p.l * Math.sin(y[2]);
                    var y2 = p.l - p.l * Math.cos(y[2]);
                    if (trails.b1.length < 800) {
                        trails.b1.push({ x: x1, y: y1 });
                        trails.b2.push({ x: x2, y: y2 });
                    }
                },
                getObservables: function () {
                    var v1 = p.l * y[1], v2 = p.l * y[3];
                    var h1 = p.l * (1 - Math.cos(y[0])), h2 = p.l * (1 - Math.cos(y[2]));
                    var Ec = 0.5 * p.m1 * v1 * v1 + 0.5 * p.m2 * v2 * v2;
                    var Ep = p.m1 * p.g * h1 + p.m2 * p.g * h2;
                    // Tangential momentum along swing
                    var P = p.m1 * v1 + p.m2 * v2;
                    var Lz = p.l * (p.m1 * v1 + p.m2 * v2);
                    return {
                        t: t,
                        Ec: Ec,
                        Ep: Ep,
                        Etot: Ec + Ep,
                        Px: p.m1 * v1 * Math.cos(y[0]) + p.m2 * v2 * Math.cos(y[2]),
                        Py: p.m1 * v1 * Math.sin(y[0]) + p.m2 * v2 * Math.sin(y[2]),
                        P: Math.abs(P),
                        Lz: Lz,
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'θ₁ / θ₂', value: fmt(radToDeg(y[0]), 1) + '° / ' + fmt(radToDeg(y[2]), 1) + '°' },
                            { label: 'v₁ / v₂', value: fmt(v1, 2) + ' / ' + fmt(v2, 2) + ' m/s' },
                            { label: 'Choques', value: String(collisions) }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t5_p9.solveAnalytical(p).values;
                    if (!firstImpact) {
                        return [
                            { label: 'v₁ (pre-choque)', unit: 'm/s', theo: ana.v1, sim: p.l * y[1] },
                            { label: 'v₁f (pendiente)', unit: 'm/s', theo: ana.v1f, sim: NaN },
                            { label: 'v₂f (pendiente)', unit: 'm/s', theo: ana.v2f, sim: NaN }
                        ];
                    }
                    return [
                        { label: 'v₁ al chocar', unit: 'm/s', theo: ana.v1, sim: firstImpact.v1Pre },
                        { label: 'v₁f tras 1er choque', unit: 'm/s', theo: ana.v1f, sim: firstImpact.v1Post },
                        { label: 'v₂f tras 1er choque', unit: 'm/s', theo: ana.v2f, sim: firstImpact.v2Post }
                    ];
                },
                getRenderState: function () {
                    var r1 = ballVisR * Math.pow(p.m1 / Math.max(p.m1, p.m2), 0.25);
                    var r2 = ballVisR * Math.pow(p.m2 / Math.max(p.m1, p.m2), 0.25);
                    var pivot1X = -r1;
                    var pivot2X = r2;
                    var py = p.l + 0.35;
                    var x1 = pivot1X + p.l * Math.sin(y[0]);
                    var y1 = py - p.l * Math.cos(y[0]);
                    var x2 = pivot2X + p.l * Math.sin(y[2]);
                    var y2 = py - p.l * Math.cos(y[2]);

                    var v1 = p.l * y[1], v2 = p.l * y[3];
                    var v1x = v1 * Math.cos(y[0]), v1y = v1 * Math.sin(y[0]);
                    var v2x = v2 * Math.cos(y[2]), v2y = v2 * Math.sin(y[2]);
                    var span = Math.max(1.8, p.l * 1.15);

                    return {
                        bounds: { minX: -span, maxX: span, minY: -0.2, maxY: py + 0.35 },
                        trails: [
                            { pts: trails.b1.map(function (pt) { return { x: pt.x - (r1 - ballVisR), y: pt.y + 0.35 }; }), color: 'rgba(56, 189, 248, 0.35)', width: 1.5 },
                            { pts: trails.b2.map(function (pt) { return { x: pt.x + (r2 - ballVisR), y: pt.y + 0.35 }; }), color: 'rgba(251, 191, 36, 0.35)', width: 1.5 }
                        ],
                        lines: [
                            { x1: -span * 0.7, y1: py, x2: span * 0.7, y2: py, color: '#475569', width: 5 },
                            { x1: pivot1X, y1: py, x2: x1, y2: y1, color: '#cbd5e1', width: 2 },
                            { x1: pivot2X, y1: py, x2: x2, y2: y2, color: '#cbd5e1', width: 2 },
                            { x1: -span * 0.9, y1: 0, x2: span * 0.9, y2: 0, color: 'rgba(148,163,184,0.25)', width: 1, dash: [6, 6] }
                        ],
                        masses: [
                            { x: x1, y: y1, r: r1, color: '#38bdf8', label: 'B₁ (' + p.m1 + ' kg)' },
                            { x: x2, y: y2, r: r2, color: '#fbbf24', label: 'B₂ (' + p.m2 + ' kg)' }
                        ],
                        vectors: [
                            { type: 'velocity', x: x1, y: y1, vx: v1x * 0.22, vy: v1y * 0.22, color: '#22d3ee', label: 'v₁' },
                            { type: 'velocity', x: x2, y: y2, vx: v2x * 0.22, vy: v2y * 0.22, color: '#fb923c', label: 'v₂' },
                            { type: 'momentum', x: x1, y: y1, vx: p.m1 * v1x * 0.08, vy: p.m1 * v1y * 0.08, color: '#a78bfa', label: 'p₁' },
                            { type: 'momentum', x: x2, y: y2, vx: p.m2 * v2x * 0.08, vy: p.m2 * v2y * 0.08, color: '#f472b6', label: 'p₂' }
                        ],
                        banner: firstImpact
                            ? 'Tras choque #' + collisions + ': v₁f = ' + fmt(firstImpact.v1Post, 2) + ' m/s, v₂f = ' + fmt(firstImpact.v2Post, 2) + ' m/s'
                            : 'B₁ cayendo desde θ₀ = ' + p.theta0Deg + '° hacia B₂'
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 3 — T5 · P6: Choque Inelástico 2D en Intersección con Rozamiento
    // =========================================================================
    var t5_p6 = {
        id: 't5_p6',
        topic: 'particles',
        level: 3,
        code: 'T5 · P6',
        title: 'Choque Inelástico 2D (Camión y Coche)',
        subtitle: 'Conservación vectorial de P en el choque y frenado por rozamiento',
        renderMode: '2d',
        learningGoal: 'Distinguir cuándo se conserva P (instante del choque, fuerzas internas impulsivas ≫ rozamiento) y cuándo NO se conserva (frenado posterior bajo fuerza externa f_r).',
        guideFromZero: [
            '<strong>1. En la colisión (t = t_choque):</strong> Las fuerzas impulsivas internas son enormes frente al rozamiento: se conserva el vector momento lineal <code>P = p_camión + p_coche</code>.',
            '<strong>2. Choque perfectamente inelástico:</strong> Quedan unidos con velocidad común <code>V = P / (M + m)</code> y dirección <code>θ = atan(Pᵧ / Pₓ)</code>. Se disipa energía cinética en deformación.',
            '<strong>3. Tras el choque:</strong> Actúa el rozamiento cinético <code>f_r = −μ(M+m)g û</code>, desacelerando el conjunto hasta detenerse en <code>d = V² / (2μg)</code>.'
        ],
        statement: 'Un camión (M = 10 Tm, v = 72 km/h) y un coche (m = 1 Tm, v = 144 km/h) ruedan por calles perpendiculares, chocan inelásticamente en la intersección y quedan unidos. Calcular: (a) Velocidad y dirección del conjunto. (b) Espacio recorrido hasta parar con μ = 0.2. (c) Pérdida de energía cinética.',
        paramsSchema: [
            { key: 'M', label: 'Masa camión M', unit: 'kg', min: 2000, max: 30000, step: 500, def: 10000 },
            { key: 'm', label: 'Masa coche m', unit: 'kg', min: 500, max: 5000, step: 100, def: 1000 },
            { key: 'vM_kmh', label: 'Velocidad camión', unit: 'km/h', min: 10, max: 140, step: 2, def: 72 },
            { key: 'vm_kmh', label: 'Velocidad coche', unit: 'km/h', min: 10, max: 200, step: 2, def: 144 },
            { key: 'mu', label: 'Coef. rozamiento μ', unit: '', min: 0.05, max: 0.9, step: 0.05, def: 0.2 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        defaultDuration: 12.0,

        solveAnalytical: function (p) {
            var vM = p.vM_kmh / 3.6;
            var vm = p.vm_kmh / 3.6;
            var Mtot = p.M + p.m;
            var Px = p.M * vM;
            var Py = p.m * vm;
            var Vx = Px / Mtot;
            var Vy = Py / Mtot;
            var V = Math.hypot(Vx, Vy);
            var thetaDeg = radToDeg(Math.atan2(Vy, Vx));
            var dist = (V * V) / (2 * p.mu * p.g);
            var EcInit = 0.5 * p.M * vM * vM + 0.5 * p.m * vm * vm;
            var EcPost = 0.5 * Mtot * V * V;
            var dEc = EcPost - EcInit;
            var lossPct = EcInit > 0 ? (-dEc / EcInit) * 100 : 0;

            return {
                steps: [
                    {
                        title: '1. Conversión a SI y momentos iniciales',
                        formula: 'v_M = ' + fmt(vM, 2) + ' m/s (+i)  ;  v_m = ' + fmt(vm, 2) + ' m/s (+j)',
                        detail: 'Pₓ = M·v_M = ' + fmt(Px / 1000, 2) + ' kN·s  ;  Pᵧ = m·v_m = ' + fmt(Py / 1000, 2) + ' kN·s'
                    },
                    {
                        title: '(a) Velocidad y ángulo tras el choque inelástico',
                        formula: 'V = P / (M + m) = (' + fmt(Vx, 3) + ' i + ' + fmt(Vy, 3) + ' j) m/s',
                        detail: '|V| = ' + fmt(V, 4) + ' m/s (' + fmt(V * 3.6, 2) + ' km/h)  ;  θ = atan(Pᵧ/Pₓ) = ' + fmt(thetaDeg, 2) + '° respecto al eje X'
                    },
                    {
                        title: '(b) Distancia recorrida hasta detenerse',
                        formula: 'a = −μg = −' + fmt(p.mu * p.g, 3) + ' m/s² → d = V² / (2μg)',
                        detail: 'd = ' + fmt(dist, 3) + ' m  (tiempo de frenado: ' + fmt(V / (p.mu * p.g), 2) + ' s)'
                    },
                    {
                        title: '(c) Pérdida de energía cinética en el choque',
                        formula: 'ΔE_c = E_c,después − E_c,antes',
                        detail: 'E_c,antes = ' + fmt(EcInit / 1e6, 4) + ' MJ → E_c,después = ' + fmt(EcPost / 1e6, 4) + ' MJ → Pérdida = ' + fmt(-dEc / 1e3, 2) + ' kJ (' + fmt(lossPct, 2) + '%)'
                    }
                ],
                values: { V: V, Vx: Vx, Vy: Vy, thetaDeg: thetaDeg, dist: dist, EcInit: EcInit, EcPost: EcPost, lossJ: -dEc }
            };
        },

        createSimulation: function (p) {
            var vM = p.vM_kmh / 3.6;
            var vm = p.vm_kmh / 3.6;
            var Mtot = p.M + p.m;
            var tApproach = 1.5;
            var t = 0;
            var collided = false;
            var stopped = false;

            // Pre-collision: individual positions; post-collision: unified [x, y, vx, vy]
            var x1 = -vM * tApproach, y1 = 0;
            var x2 = 0, y2 = -vm * tApproach;
            var combined = [0, 0, 0, 0];
            var simEcPost = null;
            var trails = { truck: [{ x: x1, y: y1 }], car: [{ x: x2, y: y2 }], wreck: [] };

            function derivsPost(st) {
                var speed = Math.hypot(st[2], st[3]);
                if (speed < 1e-9) return [0, 0, 0, 0];
                var aFric = p.mu * p.g;
                return [st[2], st[3], -aFric * st[2] / speed, -aFric * st[3] / speed];
            }

            return {
                step: function (dt) {
                    if (stopped) return;
                    var rem = dt;
                    if (!collided) {
                        if (t + rem >= tApproach) {
                            var dtPre = tApproach - t;
                            x1 += vM * dtPre;
                            y2 += vm * dtPre;
                            t = tApproach;
                            rem -= dtPre;
                            // Inelastic collision at (0,0)
                            collided = true;
                            var Vx = (p.M * vM) / Mtot;
                            var Vy = (p.m * vm) / Mtot;
                            combined = [0, 0, Vx, Vy];
                            simEcPost = 0.5 * Mtot * (Vx * Vx + Vy * Vy);
                            trails.truck.push({ x: 0, y: 0 });
                            trails.car.push({ x: 0, y: 0 });
                            trails.wreck.push({ x: 0, y: 0 });
                        } else {
                            x1 += vM * rem;
                            y2 += vm * rem;
                            t += rem;
                            trails.truck.push({ x: x1, y: y1 });
                            trails.car.push({ x: x2, y: y2 });
                            return;
                        }
                    }

                    if (collided && rem > 0 && !stopped) {
                        var speed0 = Math.hypot(combined[2], combined[3]);
                        var aFric = p.mu * p.g;
                        if (speed0 <= aFric * rem) {
                            var tStop = speed0 / aFric;
                            combined = rk4Integrate(combined, t, tStop, 0.005, derivsPost);
                            combined[2] = 0;
                            combined[3] = 0;
                            t += tStop;
                            stopped = true;
                        } else {
                            combined = rk4Integrate(combined, t, rem, 0.01, derivsPost);
                            t += rem;
                        }
                        if (trails.wreck.length < 1500) {
                            trails.wreck.push({ x: combined[0], y: combined[1] });
                        }
                    }
                },
                getObservables: function () {
                    var Ec, Px, Py, cmx, cmy;
                    if (!collided) {
                        Ec = 0.5 * p.M * vM * vM + 0.5 * p.m * vm * vm;
                        Px = p.M * vM;
                        Py = p.m * vm;
                        cmx = (p.M * x1 + p.m * x2) / Mtot;
                        cmy = (p.M * y1 + p.m * y2) / Mtot;
                    } else {
                        Ec = 0.5 * Mtot * (combined[2] * combined[2] + combined[3] * combined[3]);
                        Px = Mtot * combined[2];
                        Py = Mtot * combined[3];
                        cmx = combined[0];
                        cmy = combined[1];
                    }
                    var distSoFar = collided ? Math.hypot(combined[0], combined[1]) : 0;
                    var speed = collided ? Math.hypot(combined[2], combined[3]) : 0;
                    return {
                        t: t,
                        Ec: Ec,
                        Ep: 0,
                        Etot: Ec,
                        Px: Px,
                        Py: Py,
                        P: Math.hypot(Px, Py),
                        Lz: collided ? 0 : (x1 * 0 - y1 * (p.M * vM)) + (x2 * (p.m * vm) - y2 * 0),
                        cm: { x: cmx, y: cmy },
                        finished: stopped,
                        extra: [
                            { label: 'Estado', value: stopped ? 'Detenido' : (collided ? 'Frenando (μ=' + p.mu + ')' : 'Aproximación') },
                            { label: 'Velocidad conjunto', value: collided ? fmt(speed, 2) + ' m/s' : '—' },
                            { label: 'Distancia frenado', value: fmt(distSoFar, 2) + ' m' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t5_p6.solveAnalytical(p).values;
                    var distSim = collided ? Math.hypot(combined[0], combined[1]) : NaN;
                    var Vsim = collided ? Math.hypot(combined[2], combined[3]) : NaN;
                    return [
                        { label: 'V post-choque', unit: 'm/s', theo: ana.V, sim: simEcPost !== null ? Math.sqrt(2 * simEcPost / Mtot) : NaN },
                        { label: 'Distancia frenado', unit: 'm', theo: ana.dist, sim: stopped ? distSim : NaN },
                        { label: 'Pérdida E_c', unit: 'kJ', theo: ana.lossJ / 1000, sim: simEcPost !== null ? (ana.EcInit - simEcPost) / 1000 : NaN },
                        { label: 'Velocidad actual', unit: 'm/s', theo: stopped ? 0 : Vsim, sim: Vsim }
                    ];
                },
                getRenderState: function () {
                    var ana = t5_p6.solveAnalytical(p).values;
                    var maxDist = Math.max(40, ana.dist * 1.15);
                    var minCoord = -Math.max(40, Math.max(vM, vm) * tApproach * 1.1);
                    var roads = [
                        { x1: minCoord, y1: 0, x2: maxDist, y2: 0, color: 'rgba(100,116,139,0.28)', width: 16 },
                        { x1: 0, y1: minCoord, x2: 0, y2: maxDist * 0.65, color: 'rgba(100,116,139,0.28)', width: 16 }
                    ];
                    var masses = [];
                    var vectors = [];
                    var cm;

                    if (!collided) {
                        cm = { x: (p.M * x1 + p.m * x2) / Mtot, y: (p.M * y1 + p.m * y2) / Mtot, label: 'CM' };
                        masses.push({ x: x1, y: y1, r: 3.2, color: '#38bdf8', label: 'Camión (' + (p.M / 1000) + ' Tm)' });
                        masses.push({ x: x2, y: y2, r: 2.0, color: '#fb7185', label: 'Coche (' + (p.m / 1000) + ' Tm)' });
                        vectors.push({ type: 'velocity', x: x1, y: y1, vx: vM * 0.6, vy: 0, color: '#22d3ee', label: 'v_M' });
                        vectors.push({ type: 'velocity', x: x2, y: y2, vx: 0, vy: vm * 0.6, color: '#f472b6', label: 'v_m' });
                        vectors.push({ type: 'momentum', x: cm.x, y: cm.y, vx: (p.M * vM) * 0.00012, vy: (p.m * vm) * 0.00012, color: '#fbbf24', label: 'P_total' });
                    } else {
                        cm = { x: combined[0], y: combined[1], label: 'CM' };
                        masses.push({ x: combined[0], y: combined[1], r: 3.6, color: '#fbbf24', label: 'Conjunto (' + (Mtot / 1000) + ' Tm)' });
                        var spd = Math.hypot(combined[2], combined[3]);
                        if (spd > 1e-3) {
                            vectors.push({ type: 'velocity', x: combined[0], y: combined[1], vx: combined[2] * 0.9, vy: combined[3] * 0.9, color: '#22d3ee', label: 'V' });
                            var ux = combined[2] / spd, uy = combined[3] / spd;
                            vectors.push({ type: 'force', x: combined[0], y: combined[1], vx: -ux * 14, vy: -uy * 14, color: '#ef4444', label: 'f_r' });
                        }
                    }

                    return {
                        bounds: { minX: minCoord, maxX: maxDist, minY: minCoord, maxY: Math.max(35, ana.dist * 0.45) },
                        lines: roads,
                        trails: [
                            { pts: trails.truck, color: 'rgba(56, 189, 248, 0.5)', width: 2 },
                            { pts: trails.car, color: 'rgba(251, 113, 133, 0.5)', width: 2 },
                            { pts: trails.wreck, color: '#fbbf24', width: 3.5 }
                        ],
                        masses: masses,
                        cm: cm,
                        vectors: vectors,
                        banner: stopped
                            ? 'Detenido a d = ' + fmt(Math.hypot(combined[0], combined[1]), 2) + ' m (θ = ' + fmt(ana.thetaDeg, 1) + '°)'
                            : (collided ? 'Frenando juntos: θ = ' + fmt(ana.thetaDeg, 1) + '° | E_c disipándose por μ' : 'Camión y coche acercándose a la intersección (0, 0)')
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 4 — T5 · P12: Balón de Fútbol y Bola de Nieve en la Cima
    // =========================================================================
    var t5_p12 = {
        id: 't5_p12',
        topic: 'particles',
        level: 4,
        code: 'T5 · P12',
        title: 'Balón y Bola de Nieve en Vuelo Parabólico',
        subtitle: 'Colisión inelástica en el punto más alto bajo gravedad',
        renderMode: '2d',
        learningGoal: 'Aplicar la conservación de P en el instante instantáneo de una colisión aérea bajo gravedad (la gravedad es finita, los impulsos internos de choque dominan en Δt→0).',
        guideFromZero: [
            '<strong>1. Tramo 1 (hasta la cima):</strong> El balón sigue tiro parabólico; en la cima, <code>v_y = 0</code> y solo tiene velocidad horizontal <code>v₀ cos α</code>.',
            '<strong>2. Choque instantáneo:</strong> La bola de nieve (que viene desde la grada a la derecha, 60° bajo la horizontal) se pega al balón. Se conserva <code>P</code> justo antes y justo después.',
            '<strong>3. Tramo 2 (hacia la portería):</strong> El conjunto balón+nieve parte desde la cima con la nueva velocidad <code>V</code> y cae bajo gravedad hasta x = 28 m.'
        ],
        statement: 'Un futbolista lanza un balón de 400 g desde el suelo a 28 m de la portería, con v = 20 m/s y ángulo 45°. En el punto más alto choca y se le pega una bola de nieve de 200 g lanzada desde la grada a 10 m/s formando 60° con la horizontal (en el mismo plano). Datos: altura portería = 2.4 m. ¿Meterá gol?',
        paramsSchema: [
            { key: 'mBall', label: 'Masa balón m₁', unit: 'kg', min: 0.1, max: 2.0, step: 0.05, def: 0.4 },
            { key: 'v0', label: 'Velocidad balón v₀', unit: 'm/s', min: 8, max: 35, step: 0.5, def: 20 },
            { key: 'alphaDeg', label: 'Ángulo disparo α', unit: '°', min: 15, max: 75, step: 1, def: 45 },
            { key: 'mSnow', label: 'Masa bola nieve m₂', unit: 'kg', min: 0.05, max: 1.5, step: 0.05, def: 0.2 },
            { key: 'vSnow', label: 'Velocidad nieve v_b', unit: 'm/s', min: 0, max: 25, step: 0.5, def: 10 },
            { key: 'betaDeg', label: 'Ángulo nieve (bajo horiz.)', unit: '°', min: 0, max: 85, step: 1, def: 60 },
            { key: 'xGoal', label: 'Distancia portería', unit: 'm', min: 10, max: 45, step: 1, def: 28 },
            { key: 'hGoal', label: 'Altura larguero', unit: 'm', min: 1.0, max: 4.0, step: 0.1, def: 2.4 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        defaultDuration: 4.0,

        solveAnalytical: function (p) {
            var al = degToRad(p.alphaDeg);
            var be = degToRad(p.betaDeg);
            var v1x = p.v0 * Math.cos(al);
            var v1y = p.v0 * Math.sin(al);
            var tApex = v1y / p.g;
            var xApex = v1x * tApex;
            var yApex = 0.5 * v1y * tApex;

            // Snowball from spectator stands (right -> left, downward at angle betaDeg)
            var vbx = -p.vSnow * Math.cos(be);
            var vby = -p.vSnow * Math.sin(be);
            var Mtot = p.mBall + p.mSnow;
            var Vx = (p.mBall * v1x + p.mSnow * vbx) / Mtot;
            var Vy = (p.mBall * 0 + p.mSnow * vby) / Mtot;

            var dx = p.xGoal - xApex;
            var validGoalX = Vx > 1e-6 && dx >= 0;
            var dtGoal = validGoalX ? dx / Vx : NaN;
            var yAtGoal = validGoalX ? (yApex + Vy * dtGoal - 0.5 * p.g * dtGoal * dtGoal) : NaN;
            var isGoal = validGoalX && yAtGoal >= 0 && yAtGoal <= p.hGoal;
            var verdict = !validGoalX
                ? (Vx <= 0 ? 'NO GOL (retrocede tras el impacto)' : 'La portería está antes de la cima')
                : (yAtGoal > p.hGoal ? 'NO GOL (pasa por encima del larguero: ' + fmt(yAtGoal, 3) + ' m > ' + fmt(p.hGoal, 2) + ' m)'
                    : (yAtGoal < 0 ? 'NO GOL (bota antes de la línea de gol)' : '¡GOL! Entra por debajo del larguero'));

            return {
                steps: [
                    {
                        title: '1. Punto más alto de la trayectoria del balón',
                        formula: 't_cima = v₀ sin α / g = ' + fmt(tApex, 4) + ' s',
                        detail: 'x_cima = ' + fmt(xApex, 3) + ' m  ;  y_cima = ' + fmt(yApex, 3) + ' m  ;  v_balón(cima) = ' + fmt(v1x, 3) + ' i m/s'
                    },
                    {
                        title: '2. Conservación de P en el choque inelástico',
                        formula: 'v_nieve = −' + fmt(p.vSnow * Math.cos(be), 2) + ' i − ' + fmt(p.vSnow * Math.sin(be), 2) + ' j m/s',
                        detail: 'V_tras = (' + fmt(Vx, 4) + ' i + ' + fmt(Vy, 4) + ' j) m/s  (m_total = ' + fmt(Mtot, 2) + ' kg)'
                    },
                    {
                        title: '3. Altura al alcanzar la línea de gol (x = ' + fmt(p.xGoal, 1) + ' m)',
                        formula: 'Δt = (x_portería − x_cima) / Vₓ = ' + fmt(dtGoal, 4) + ' s',
                        detail: 'y(' + fmt(p.xGoal, 1) + ' m) = y_cima + VᵧΔt − ½gΔt² = ' + fmt(yAtGoal, 3) + ' m → ' + verdict
                    }
                ],
                values: { tApex: tApex, xApex: xApex, yApex: yApex, Vx: Vx, Vy: Vy, yAtGoal: yAtGoal, isGoal: isGoal, verdict: verdict }
            };
        },

        createSimulation: function (p) {
            var al = degToRad(p.alphaDeg);
            var be = degToRad(p.betaDeg);
            var v1x = p.v0 * Math.cos(al);
            var v1y = p.v0 * Math.sin(al);
            var tApex = v1y / p.g;
            var xApex = v1x * tApex;
            var yApex = 0.5 * v1y * tApex;
            var vbx = -p.vSnow * Math.cos(be);
            var vby = -p.vSnow * Math.sin(be);
            var Mtot = p.mBall + p.mSnow;

            // Ball state [x, y, vx, vy], Ghost unhit ball [gx, gy, gvx, gvy]
            var y = [0, 0, v1x, v1y];
            var ghost = [0, 0, v1x, v1y];
            var t = 0;
            var collided = false;
            var landed = false;
            var simYAtGoal = NaN;
            var simApexX = NaN;
            var simApexY = NaN;
            var trails = { ball: [{ x: 0, y: 0 }], ghost: [{ x: 0, y: 0 }], snow: [] };

            function derivsProj(st) {
                return [st[2], st[3], 0, -p.g];
            }

            function snowballPos(curT) {
                var tau = tApex - curT;
                // Backtrack under gravity so snowball meets exact apex at t = tApex
                return {
                    x: xApex - vbx * tau,
                    y: yApex - vby * tau - 0.5 * p.g * tau * tau,
                    vx: vbx,
                    vy: vby + p.g * tau
                };
            }
            // Note: Wait! If snowball is in free fall too, its velocity at apex is (vbx, vby).
            // So at t < tApex, vy_snow(t) = vby + g*(tApex - t), and y_snow(t) = yApex - vby*(tApex-t) - 0.5*g*(tApex-t)^2?
            // Check derivative: d/dt(yApex - vby*(tApex-t) - 0.5*g*(tApex-t)^2) = +vby + g*(tApex - t).
            // At t = tApex, vy = vby, y = yApex! Wait: check d^2y/dt^2 = -g! Exact!

            return {
                step: function (dt) {
                    if (landed) return;
                    var rem = dt;

                    // Ghost ball always moves freely
                    if (ghost[1] >= 0) {
                        ghost = rk4Integrate(ghost, t, rem, 0.008, derivsProj);
                        if (trails.ghost.length < 600 && ghost[1] >= 0) trails.ghost.push({ x: ghost[0], y: ghost[1] });
                    }

                    if (!collided) {
                        var yNext = rk4Integrate(y, t, rem, 0.005, derivsProj);
                        if (y[3] > 0 && yNext[3] <= 0) {
                            var ev = bisectEvent(y, t, rem, derivsProj, function (st) { return st[3]; }, 24);
                            var remAfter = rem - ev.dtEvent;
                            y = ev.yEvent;
                            t = ev.tEvent;
                            simApexX = y[0];
                            simApexY = y[1];
                            collided = true;
                            // Inelastic collision at apex
                            y[2] = (p.mBall * y[2] + p.mSnow * vbx) / Mtot;
                            y[3] = (p.mBall * y[3] + p.mSnow * vby) / Mtot;
                            trails.ball.push({ x: y[0], y: y[1] });
                            rem = remAfter;
                        } else {
                            y = yNext;
                            t += rem;
                            var sb = snowballPos(t);
                            if (trails.ball.length < 600) trails.ball.push({ x: y[0], y: y[1] });
                            if (trails.snow.length < 600) trails.snow.push({ x: sb.x, y: sb.y });
                            return;
                        }
                    }

                    if (collided && rem > 0) {
                        var yPost = rk4Integrate(y, t, rem, 0.005, derivsProj);
                        // Check goal line crossing x === p.xGoal
                        if (isNaN(simYAtGoal) && ((y[0] < p.xGoal && yPost[0] >= p.xGoal) || (y[0] > p.xGoal && yPost[0] <= p.xGoal))) {
                            var evG = bisectEvent(y, t, rem, derivsProj, function (st) { return st[0] - p.xGoal; }, 24);
                            simYAtGoal = evG.yEvent[1];
                        }
                        if (y[1] > 0 && yPost[1] <= 0) {
                            var evLand = bisectEvent(y, t, rem, derivsProj, function (st) { return st[1]; }, 24);
                            y = evLand.yEvent;
                            y[1] = 0;
                            t = evLand.tEvent;
                            landed = true;
                        } else {
                            y = yPost;
                            t += rem;
                        }
                        if (trails.ball.length < 800) trails.ball.push({ x: y[0], y: y[1] });
                    }
                },
                getObservables: function () {
                    var mass = collided ? Mtot : p.mBall;
                    var Ec = 0.5 * mass * (y[2] * y[2] + y[3] * y[3]);
                    var Ep = mass * p.g * Math.max(0, y[1]);
                    var Px = mass * y[2], Py = mass * y[3];
                    if (!collided) {
                        var sb = snowballPos(t);
                        Ec += 0.5 * p.mSnow * (sb.vx * sb.vx + sb.vy * sb.vy);
                        Ep += p.mSnow * p.g * Math.max(0, sb.y);
                        Px += p.mSnow * sb.vx;
                        Py += p.mSnow * sb.vy;
                    }
                    return {
                        t: t,
                        Ec: Ec,
                        Ep: Ep,
                        Etot: Ec + Ep,
                        Px: Px,
                        Py: Py,
                        P: Math.hypot(Px, Py),
                        Lz: 0,
                        cm: { x: y[0], y: y[1] },
                        finished: landed,
                        extra: [
                            { label: 'Posición balón', value: '(' + fmt(y[0], 2) + ', ' + fmt(y[1], 2) + ') m' },
                            { label: 'y en portería', value: isNaN(simYAtGoal) ? (landed ? 'No alcanza portería' : 'Pendiente') : fmt(simYAtGoal, 3) + ' m (larguero ' + p.hGoal + ' m)' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t5_p12.solveAnalytical(p).values;
                    return [
                        { label: 'x_cima', unit: 'm', theo: ana.xApex, sim: collided ? simApexX : NaN },
                        { label: 'y_cima', unit: 'm', theo: ana.yApex, sim: collided ? simApexY : NaN },
                        { label: 'Vₓ post-choque', unit: 'm/s', theo: ana.Vx, sim: collided ? y[2] : NaN },
                        { label: 'y en portería', unit: 'm', theo: ana.yAtGoal, sim: simYAtGoal }
                    ];
                },
                getRenderState: function () {
                    var ana = t5_p12.solveAnalytical(p).values;
                    var minX = Math.min(-2, y[0] - 3);
                    var maxX = Math.max(p.xGoal + 6, xApex + 14);
                    var maxY = Math.max(yApex + 4, 13);
                    var masses = [
                        { x: y[0], y: y[1], r: collided ? 0.45 : 0.38, color: collided ? '#fbbf24' : '#38bdf8', label: collided ? 'Balón+Nieve' : 'Balón' }
                    ];
                    var vectors = [
                        { type: 'velocity', x: y[0], y: y[1], vx: y[2] * 0.22, vy: y[3] * 0.22, color: '#22d3ee', label: 'v' }
                    ];
                    if (!collided) {
                        var sb = snowballPos(t);
                        masses.push({ x: sb.x, y: sb.y, r: 0.28, color: '#e2e8f0', label: 'Bola nieve' });
                        vectors.push({ type: 'velocity', x: sb.x, y: sb.y, vx: sb.vx * 0.22, vy: sb.vy * 0.22, color: '#f472b6', label: 'v_b' });
                    }
                    return {
                        bounds: { minX: minX, maxX: maxX, minY: -1, maxY: maxY },
                        lines: [
                            { x1: minX, y1: 0, x2: maxX, y2: 0, color: '#22c55e', width: 3 },
                            // Goal posts
                            { x1: p.xGoal, y1: 0, x2: p.xGoal, y2: p.hGoal, color: '#f8fafc', width: 5 },
                            { x1: p.xGoal, y1: p.hGoal, x2: p.xGoal + 1.4, y2: p.hGoal, color: '#f8fafc', width: 5 },
                            { x1: p.xGoal + 1.4, y1: 0, x2: p.xGoal + 1.4, y2: p.hGoal, color: 'rgba(248,250,252,0.4)', width: 2 }
                        ],
                        trails: [
                            { pts: trails.ghost, color: 'rgba(148, 163, 184, 0.35)', width: 1.5, dash: [5, 5] },
                            { pts: trails.snow, color: 'rgba(226, 232, 240, 0.5)', width: 1.5, dash: [3, 3] },
                            { pts: trails.ball, color: '#38bdf8', width: 3 }
                        ],
                        masses: masses,
                        cm: { x: y[0], y: y[1], label: '' },
                        vectors: vectors,
                        banner: isNaN(simYAtGoal) ? 'Trayectoria en curso hacia la portería (x = ' + p.xGoal + ' m)' : ana.verdict
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 5 — T5 · P16: Explosión de Proyectil en la Cima
    // =========================================================================
    var t5_p16 = {
        id: 't5_p16',
        topic: 'particles',
        level: 5,
        code: 'T5 · P16',
        title: 'Proyectil que Explota en la Cima',
        subtitle: 'El CM sigue la parábola original mientras los fragmentos se separan',
        renderMode: '2d',
        learningGoal: 'Visualizar que una explosión es una fuerza interna que libera energía cinética (ΔE_c > 0), pero NO altera la trayectoria parabólica del CM mientras ambos fragmentos están en el aire.',
        guideFromZero: [
            '<strong>1. Antes de explotar:</strong> En la cima, el proyectil de 3 kg solo tiene velocidad horizontal <code>v_CM,x = v₀ cos α</code>.',
            '<strong>2. En la explosión:</strong> Se conserva el momento lineal: <code>M v_CM = m₁ v₁ + m₂ v₂</code>. Como m₂ cae verticalmente (<code>v₂ₓ = 0</code>), m₁ se lleva TODO el momento horizontal (¡sale a 3·v_CM,x!).',
            '<strong>3. Energía liberada:</strong> <code>ΔE_exp = E_c,después − E_c,antes > 0</code> (energía química convertida en cinética).'
        ],
        statement: 'Se lanza un proyectil de 3 kg a 30° con v₀ = 120 m/s. En la parte superior explota en dos fragmentos de 1 kg y 2 kg. El de 2 kg cae directamente debajo del punto de explosión 3.6 s después. Calcular: (a) Velocidad del fragmento de 1 kg tras la explosión. (b) Alcance horizontal del fragmento de 1 kg. (c) Energía liberada en la explosión.',
        paramsSchema: [
            { key: 'v0', label: 'Velocidad inicial v₀', unit: 'm/s', min: 30, max: 200, step: 5, def: 120 },
            { key: 'alphaDeg', label: 'Ángulo disparo α', unit: '°', min: 15, max: 75, step: 1, def: 30 },
            { key: 'm1', label: 'Fragmento m₁', unit: 'kg', min: 0.5, max: 5, step: 0.5, def: 1 },
            { key: 'm2', label: 'Fragmento m₂ (cae recto)', unit: 'kg', min: 0.5, max: 5, step: 0.5, def: 2 },
            { key: 'tFall2', label: 'Tiempo caída m₂ tras explosión', unit: 's', min: 1.5, max: 8.0, step: 0.1, def: 3.6 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        defaultDuration: 24.0,

        solveAnalytical: function (p) {
            var M = p.m1 + p.m2;
            var al = degToRad(p.alphaDeg);
            var vx0 = p.v0 * Math.cos(al);
            var vy0 = p.v0 * Math.sin(al);
            var tExp = vy0 / p.g;
            var xExp = vx0 * tExp;
            var yExp = 0.5 * vy0 * tExp;

            // m2 falls straight down (v2x = 0) and hits y=0 in tFall2
            var v2x = 0;
            var v2y = (0.5 * p.g * p.tFall2 * p.tFall2 - yExp) / p.tFall2;
            var v1x = (M * vx0) / p.m1;
            var v1y = -(p.m2 * v2y) / p.m1;
            var v1Mag = Math.hypot(v1x, v1y);
            var v1Angle = radToDeg(Math.atan2(v1y, v1x));

            // Quadratic 0 = yExp + v1y*t1 - 0.5*g*t1^2
            var disc = v1y * v1y + 2 * p.g * yExp;
            var t1Air = (v1y + Math.sqrt(disc)) / p.g;
            var x1Impact = xExp + v1x * t1Air;

            var EcBefore = 0.5 * M * vx0 * vx0;
            var EcAfter = 0.5 * p.m1 * (v1x * v1x + v1y * v1y) + 0.5 * p.m2 * (v2y * v2y);
            var dE = EcAfter - EcBefore;

            return {
                steps: [
                    {
                        title: '1. Cima de la trayectoria (explosión)',
                        formula: 't_exp = v₀ sin α / g = ' + fmt(tExp, 3) + ' s',
                        detail: 'x_exp = ' + fmt(xExp, 2) + ' m  ;  y_exp = ' + fmt(yExp, 2) + ' m  ;  v_CM(cima) = ' + fmt(vx0, 2) + ' i m/s'
                    },
                    {
                        title: '2. Velocidad de m₂ (cae verticalmente en ' + fmt(p.tFall2, 1) + ' s)',
                        formula: '0 = y_exp + v₂ᵧ t₂ − ½g t₂²',
                        detail: 'v₂ = (0 i ' + (v2y >= 0 ? '+ ' : '− ') + fmt(Math.abs(v2y), 3) + ' j) m/s'
                    },
                    {
                        title: '(a) Velocidad del fragmento m₁ tras la explosión',
                        formula: 'v₁ = (M v_CM − m₂ v₂) / m₁',
                        detail: 'v₁ = (' + fmt(v1x, 2) + ' i + ' + fmt(v1y, 2) + ' j) m/s  →  |v₁| = ' + fmt(v1Mag, 2) + ' m/s (θ = ' + fmt(v1Angle, 2) + '°)'
                    },
                    {
                        title: '(b) Alcance horizontal del fragmento m₁',
                        formula: 't₁ = [v₁ᵧ + √(v₁ᵧ² + 2g y_exp)] / g = ' + fmt(t1Air, 3) + ' s',
                        detail: 'x₁ = x_exp + v₁ₓ t₁ = ' + fmt(x1Impact, 1) + ' m  (el CM sin chocar caería en ' + fmt(2 * xExp, 1) + ' m)'
                    },
                    {
                        title: '(c) Energía liberada en la explosión',
                        formula: 'ΔE_exp = E_c,después − E_c,antes',
                        detail: 'E_c,antes = ' + fmt(EcBefore / 1000, 3) + ' kJ → E_c,después = ' + fmt(EcAfter / 1000, 3) + ' kJ → ΔE = +' + fmt(dE / 1000, 3) + ' kJ'
                    }
                ],
                values: { tExp: tExp, xExp: xExp, yExp: yExp, v1x: v1x, v1y: v1y, v1Mag: v1Mag, v2y: v2y, x1Impact: x1Impact, dE: dE, EcBefore: EcBefore, EcAfter: EcAfter }
            };
        },

        createSimulation: function (p) {
            var M = p.m1 + p.m2;
            var al = degToRad(p.alphaDeg);
            var vx0 = p.v0 * Math.cos(al);
            var vy0 = p.v0 * Math.sin(al);
            var ana = t5_p16.solveAnalytical(p).values;

            var proj = [0, 0, vx0, vy0];
            var ghost = [0, 0, vx0, vy0];
            var frag1 = null, frag2 = null;
            var t = 0;
            var exploded = false;
            var f2Landed = false, f1Landed = false;
            var simExpX = NaN, simV1Mag = NaN, simDeltaE = NaN;
            var trails = { pre: [{ x: 0, y: 0 }], ghost: [{ x: 0, y: 0 }], f1: [], f2: [], cm: [{ x: 0, y: 0 }] };

            function derivsProj(st) { return [st[2], st[3], 0, -p.g]; }

            return {
                step: function (dt) {
                    if (f1Landed && f2Landed) return;
                    var rem = dt;

                    if (ghost[1] >= 0) {
                        var gNext = rk4Integrate(ghost, t, rem, 0.02, derivsProj);
                        if (gNext[1] <= 0) {
                            var evG = bisectEvent(ghost, t, rem, derivsProj, function (st) { return st[1]; }, 20);
                            ghost = evG.yEvent;
                            ghost[1] = 0;
                            ghost[2] = 0;
                            ghost[3] = 0;
                        } else {
                            ghost = gNext;
                        }
                        if (trails.ghost.length < 700) trails.ghost.push({ x: ghost[0], y: ghost[1] });
                    }

                    if (!exploded) {
                        var pNext = rk4Integrate(proj, t, rem, 0.015, derivsProj);
                        if (proj[3] > 0 && pNext[3] <= 0) {
                            var evE = bisectEvent(proj, t, rem, derivsProj, function (st) { return st[3]; }, 24);
                            var remAfter = rem - evE.dtEvent;
                            proj = evE.yEvent;
                            t = evE.tEvent;
                            exploded = true;
                            var simV2y = (0.5 * p.g * p.tFall2 * p.tFall2 - proj[1]) / p.tFall2;
                            var simV1x = (M * proj[2]) / p.m1;
                            var simV1y = -(p.m2 * simV2y) / p.m1;
                            var ecPre = 0.5 * M * (proj[2] * proj[2] + proj[3] * proj[3]);
                            var ecPost = 0.5 * p.m1 * (simV1x * simV1x + simV1y * simV1y) + 0.5 * p.m2 * (simV2y * simV2y);
                            simExpX = proj[0];
                            simV1Mag = Math.hypot(simV1x, simV1y);
                            simDeltaE = ecPost - ecPre;
                            frag1 = [proj[0], proj[1], simV1x, simV1y];
                            frag2 = [proj[0], proj[1], 0, simV2y];
                            trails.pre.push({ x: proj[0], y: proj[1] });
                            trails.f1.push({ x: proj[0], y: proj[1] });
                            trails.f2.push({ x: proj[0], y: proj[1] });
                            rem = remAfter;
                        } else {
                            proj = pNext;
                            t += rem;
                            if (trails.pre.length < 500) trails.pre.push({ x: proj[0], y: proj[1] });
                            if (trails.cm.length < 800) trails.cm.push({ x: proj[0], y: proj[1] });
                            return;
                        }
                    }

                    if (exploded && rem > 0) {
                        if (!f1Landed) {
                            var f1N = rk4Integrate(frag1, t, rem, 0.02, derivsProj);
                            if (frag1[1] > 0 && f1N[1] <= 0) {
                                var ev1 = bisectEvent(frag1, t, rem, derivsProj, function (st) { return st[1]; }, 22);
                                frag1 = ev1.yEvent;
                                frag1[1] = 0;
                                frag1[2] = 0;
                                frag1[3] = 0;
                                f1Landed = true;
                            } else {
                                frag1 = f1N;
                            }
                            if (trails.f1.length < 1000) trails.f1.push({ x: frag1[0], y: frag1[1] });
                        }
                        if (!f2Landed) {
                            var f2N = rk4Integrate(frag2, t, rem, 0.02, derivsProj);
                            if (frag2[1] > 0 && f2N[1] <= 0) {
                                var ev2 = bisectEvent(frag2, t, rem, derivsProj, function (st) { return st[1]; }, 22);
                                frag2 = ev2.yEvent;
                                frag2[1] = 0;
                                frag2[2] = 0;
                                frag2[3] = 0;
                                f2Landed = true;
                            } else {
                                frag2 = f2N;
                            }
                            if (trails.f2.length < 600) trails.f2.push({ x: frag2[0], y: frag2[1] });
                        }
                        t += rem;
                        var cmx = (p.m1 * frag1[0] + p.m2 * frag2[0]) / M;
                        var cmy = (p.m1 * frag1[1] + p.m2 * frag2[1]) / M;
                        if (trails.cm.length < 1000) trails.cm.push({ x: cmx, y: cmy });
                    }
                },
                getObservables: function () {
                    if (!exploded) {
                        var Ec = 0.5 * M * (proj[2] * proj[2] + proj[3] * proj[3]);
                        var Ep = M * p.g * Math.max(0, proj[1]);
                        return {
                            t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                            Px: M * proj[2], Py: M * proj[3], P: M * Math.hypot(proj[2], proj[3]), Lz: 0,
                            cm: { x: proj[0], y: proj[1] },
                            finished: false,
                            extra: [
                                { label: 'Fase', value: 'Ascenso hacia cima' },
                                { label: 'Altura y', value: fmt(proj[1], 1) + ' m' }
                            ]
                        };
                    }
                    var Ec1 = 0.5 * p.m1 * (frag1[2] * frag1[2] + frag1[3] * frag1[3]);
                    var Ec2 = 0.5 * p.m2 * (frag2[2] * frag2[2] + frag2[3] * frag2[3]);
                    var Ep1 = p.m1 * p.g * Math.max(0, frag1[1]);
                    var Ep2 = p.m2 * p.g * Math.max(0, frag2[1]);
                    var Px = p.m1 * frag1[2] + p.m2 * frag2[2];
                    var Py = p.m1 * frag1[3] + p.m2 * frag2[3];
                    var cmx = (p.m1 * frag1[0] + p.m2 * frag2[0]) / M;
                    var cmy = (p.m1 * frag1[1] + p.m2 * frag2[1]) / M;
                    return {
                        t: t, Ec: Ec1 + Ec2, Ep: Ep1 + Ep2, Etot: Ec1 + Ec2 + Ep1 + Ep2,
                        Px: Px, Py: Py, P: Math.hypot(Px, Py), Lz: 0,
                        cm: { x: cmx, y: cmy },
                        finished: f1Landed && f2Landed,
                        extra: [
                            { label: 'm₁ (x₁, y₁)', value: '(' + fmt(frag1[0], 0) + ', ' + fmt(frag1[1], 0) + ') m' },
                            { label: 'm₂ estado', value: f2Landed ? 'En suelo (x=' + fmt(frag2[0], 0) + ' m)' : 'Cayendo (y=' + fmt(frag2[1], 0) + ' m)' },
                            { label: 'CM (x, y)', value: '(' + fmt(cmx, 0) + ', ' + fmt(cmy, 0) + ') m' }
                        ]
                    };
                },
                compareWithSim: function () {
                    return [
                        { label: 'x_explosión', unit: 'm', theo: ana.xExp, sim: exploded ? simExpX : NaN },
                        { label: '|v₁| post-exp', unit: 'm/s', theo: ana.v1Mag, sim: exploded ? simV1Mag : NaN },
                        { label: 'Alcance m₁', unit: 'm', theo: ana.x1Impact, sim: f1Landed ? frag1[0] : NaN },
                        { label: 'ΔE explosión', unit: 'kJ', theo: ana.dE / 1000, sim: exploded ? simDeltaE / 1000 : NaN }
                    ];
                },
                getRenderState: function () {
                    var maxX = Math.max(1500, ana.x1Impact * 1.06);
                    var maxY = Math.max(450, ana.yExp * 2.2);
                    var masses = [];
                    var vectors = [];
                    var cm;
                    if (!exploded) {
                        cm = { x: proj[0], y: proj[1], label: 'CM' };
                        masses.push({ x: proj[0], y: proj[1], r: maxX * 0.011, color: '#38bdf8', label: 'Proyectil (' + M + ' kg)' });
                        vectors.push({ type: 'velocity', x: proj[0], y: proj[1], vx: proj[2] * 3.0, vy: proj[3] * 3.0, color: '#22d3ee', label: 'v' });
                    } else {
                        cm = { x: (p.m1 * frag1[0] + p.m2 * frag2[0]) / M, y: (p.m1 * frag1[1] + p.m2 * frag2[1]) / M, label: 'CM' };
                        masses.push({ x: frag1[0], y: frag1[1], r: maxX * 0.009, color: '#fb7185', label: 'm₁ (' + p.m1 + ' kg)' });
                        masses.push({ x: frag2[0], y: frag2[1], r: maxX * 0.012, color: '#a78bfa', label: 'm₂ (' + p.m2 + ' kg)' });
                        if (!f1Landed) vectors.push({ type: 'velocity', x: frag1[0], y: frag1[1], vx: frag1[2] * 2.5, vy: frag1[3] * 2.5, color: '#fb7185', label: 'v₁' });
                        if (!f2Landed) vectors.push({ type: 'velocity', x: frag2[0], y: frag2[1], vx: 0, vy: frag2[3] * 3.5, color: '#a78bfa', label: 'v₂' });
                    }
                    return {
                        bounds: { minX: -150, maxX: maxX, minY: -80, maxY: maxY },
                        lines: [
                            { x1: -150, y1: 0, x2: maxX, y2: 0, color: '#22c55e', width: 3 }
                        ],
                        trails: [
                            { pts: trails.ghost, color: 'rgba(148, 163, 184, 0.35)', width: 2, dash: [8, 6] },
                            { pts: trails.pre, color: '#38bdf8', width: 2.5 },
                            { pts: trails.f1, color: '#fb7185', width: 2.5 },
                            { pts: trails.f2, color: '#a78bfa', width: 2.5 },
                            { pts: trails.cm, color: '#fbbf24', width: 2, dash: [4, 4] }
                        ],
                        masses: masses,
                        cm: cm,
                        vectors: vectors,
                        banner: !exploded
                            ? 'Ascenso del proyectil intacto'
                            : (!f2Landed ? 'En el aire: ¡el CM sigue la parábola original punteada!' : (f1Landed ? 'Impacto final: m₁ en x = ' + fmt(frag1[0], 1) + ' m' : 'm₂ tocó suelo (fuerza normal externa desvía al CM); m₁ sigue volando'))
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 6 — T5 · P15: Mancuerna Libre que Captura Tercera Masa
    // =========================================================================
    var t5_p15 = {
        id: 't5_p15',
        topic: 'particles',
        level: 6,
        code: 'T5 · P15',
        title: 'Mancuerna Libre Capturando una 3ª Masa',
        subtitle: 'Conservación simultánea de P_total (= 0) y L_CM en un sistema sin pivote fijo',
        renderMode: '2d',
        learningGoal: 'Aplicar conservación de P y L_CM en un sistema libre de fuerzas externas: el CM permanece quieto en (L/6, 0) y tras el choque la barra gira alrededor del nuevo CM, NO de su centro geométrico.',
        guideFromZero: [
            '<strong>1. Antes del choque:</strong> La mancuerna gira alrededor de O con P_mancuerna = 0 y la 3ª masa está en reposo. Por tanto <code>V_CM = 0</code> y <code>R_CM = (L/6, 0)</code>.',
            '<strong>2. Momento angular respecto al CM:</strong> Evaluando respecto a R_CM = (L/6, 0), se obtiene <code>L_CM = ½ M ω L²</code>.',
            '<strong>3. Tras la captura:</strong> Se conserva V_CM = 0 (el CM NO se mueve) y L_CM. La inercia respecto al nuevo CM es <code>I_CM = ⅔ M L²</code>, luego <code>ω_f = (3/4) ω</code> y se pierde el 25% de E_c.'
        ],
        statement: 'Dos masas puntuales M unidas por una varilla sin masa de longitud L giran sin rozamiento en el plano XY con velocidad angular ω alrededor del centro O (no fijo). Se coloca otra masa M en reposo en (L/2, 0). Una de las masas choca y queda unida a ella. Calcular: (a) Posición y velocidad del CM antes del choque. (b) Velocidad del CM después. (c) L_CM antes y después. (d) Nueva ω_f. (e) Pérdida de E_c.',
        paramsSchema: [
            { key: 'M', label: 'Masa de cada partícula M', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 1.0 },
            { key: 'L', label: 'Longitud de la varilla L', unit: 'm', min: 1.0, max: 6.0, step: 0.2, def: 2.0 },
            { key: 'omega', label: 'Velocidad angular inicial ω', unit: 'rad/s', min: 0.5, max: 10, step: 0.5, def: 4.0 }
        ],
        defaultDuration: 6.0,

        solveAnalytical: function (p) {
            var xcm = p.L / 6;
            var Lcm = 0.5 * p.M * p.omega * p.L * p.L;
            var Ibefore = 0.5 * p.M * p.L * p.L;
            var Iafter = (2 / 3) * p.M * p.L * p.L;
            var omegaF = 0.75 * p.omega;
            var EcBefore = 0.25 * p.M * p.omega * p.omega * p.L * p.L;
            var EcAfter = (3 / 16) * p.M * p.omega * p.omega * p.L * p.L;
            var loss = EcBefore - EcAfter; // 1/16 M omega^2 L^2

            return {
                steps: [
                    {
                        title: '(a) CM del sistema de 3 partículas antes del choque',
                        formula: 'R_CM = [M(L/2) + M(−L/2) + M(L/2)] / (3M) = (L/6, 0)',
                        detail: 'x_CM = ' + fmt(xcm, 4) + ' m  ;  V_CM = (m₁v₁ + m₂v₂ + 0)/(3M) = (0, 0) m/s (pues v₁ = −v₂)'
                    },
                    {
                        title: '(b) Velocidad del CM después del choque',
                        formula: 'Sin fuerzas externas → P = 0 constante',
                        detail: 'V_CM(t) = (0, 0) m/s siempre: ¡el punto (L/6, 0) queda completamente quieto!'
                    },
                    {
                        title: '(c) Momento angular respecto al CM',
                        formula: 'L_CM = (L/3)M(ωL/2) + (2L/3)M(ωL/2) = ½ M ω L²',
                        detail: 'L_CM,antes = L_CM,después = ' + fmt(Lcm, 4) + ' kg·m²/s (eje +Z)'
                    },
                    {
                        title: '(d) Velocidad angular tras el choque',
                        formula: 'I_CM,f = (2M)(L/3)² + M(2L/3)² = ⅔ M L² → ω_f = L_CM / I_CM,f = ¾ ω',
                        detail: 'I_CM,antes = ' + fmt(Ibefore, 4) + ' → I_CM,f = ' + fmt(Iafter, 4) + ' kg·m²  ;  ω_f = ' + fmt(omegaF, 4) + ' rad/s'
                    },
                    {
                        title: '(e) Pérdida de energía cinética',
                        formula: 'ΔE_c = (3/16 − 1/4) M ω² L² = −(1/16) M ω² L²',
                        detail: 'E_c,i = ' + fmt(EcBefore, 3) + ' J → E_c,f = ' + fmt(EcAfter, 3) + ' J → Pérdida = ' + fmt(loss, 3) + ' J (25.0%)'
                    }
                ],
                values: { xcm: xcm, Lcm: Lcm, Ibefore: Ibefore, Iafter: Iafter, omegaF: omegaF, EcBefore: EcBefore, EcAfter: EcAfter, loss: loss }
            };
        },

        createSimulation: function (p) {
            var preAngle = Math.PI / 3; // start 60 deg before collision
            var tColl = preAngle / p.omega;
            var t = 0;
            var collided = false;
            var halfL = 0.5 * p.L;
            var rCM = p.L / 6;

            // State: before collision [theta, omega]; after collision [theta, omega] rotating around (L/6, 0)
            var y = [-preAngle, p.omega];
            var trails = { m1: [], m2: [], centerRod: [] };

            function derivs(st) {
                return [st[1], 0];
            }

            return {
                step: function (dt) {
                    var rem = dt;
                    if (!collided && t + rem >= tColl) {
                        var dtPre = tColl - t;
                        y = rk4Integrate(y, t, dtPre, 0.004, derivs);
                        t = tColl;
                        rem -= dtPre;
                        y[0] = 0;
                        y[1] = 0.75 * p.omega; // L_CM conservation
                        collided = true;
                    }
                    if (rem > 0) {
                        y = rk4Integrate(y, t, rem, 0.005, derivs);
                        t += rem;
                    }

                    var th = y[0];
                    var cx = collided ? rCM : 0;
                    var x1 = collided ? cx + (p.L / 3) * Math.cos(th) : halfL * Math.cos(th);
                    var y1 = collided ? (p.L / 3) * Math.sin(th) : halfL * Math.sin(th);
                    var x2 = collided ? cx - (2 * p.L / 3) * Math.cos(th) : -halfL * Math.cos(th);
                    var y2 = collided ? -(2 * p.L / 3) * Math.sin(th) : -halfL * Math.sin(th);
                    var ox = 0.5 * (x1 + x2), oy = 0.5 * (y1 + y2);

                    if (trails.m1.length < 900) {
                        trails.m1.push({ x: x1, y: y1 });
                        trails.m2.push({ x: x2, y: y2 });
                        trails.centerRod.push({ x: ox, y: oy });
                    }
                },
                getObservables: function () {
                    var Icur = collided ? (2 / 3) * p.M * p.L * p.L : 0.5 * p.M * p.L * p.L;
                    var Ec = 0.5 * Icur * y[1] * y[1];
                    var Lcm = Icur * y[1];
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: 0, Py: 0, P: 0, Lz: Lcm,
                        cm: { x: rCM, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'Fase', value: collided ? 'Unidas (3M gira en torno a CM)' : 'Pre-choque (mancuerna)' },
                            { label: 'ω actual', value: fmt(y[1], 3) + ' rad/s' },
                            { label: 'L_CM', value: fmt(Lcm, 3) + ' kg·m²/s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t5_p15.solveAnalytical(p).values;
                    var Icur = collided ? (2 / 3) * p.M * p.L * p.L : 0.5 * p.M * p.L * p.L;
                    return [
                        { label: 'x_CM', unit: 'm', theo: ana.xcm, sim: rCM },
                        { label: 'L_CM', unit: 'kg·m²/s', theo: ana.Lcm, sim: Icur * y[1] },
                        { label: 'ω tras choque', unit: 'rad/s', theo: ana.omegaF, sim: collided ? y[1] : NaN },
                        { label: 'E_c actual', unit: 'J', theo: collided ? ana.EcAfter : ana.EcBefore, sim: 0.5 * Icur * y[1] * y[1] }
                    ];
                },
                getRenderState: function () {
                    var th = y[0], w = y[1];
                    var cx = collided ? rCM : 0;
                    var x1 = collided ? cx + (p.L / 3) * Math.cos(th) : halfL * Math.cos(th);
                    var y1 = collided ? (p.L / 3) * Math.sin(th) : halfL * Math.sin(th);
                    var x2 = collided ? cx - (2 * p.L / 3) * Math.cos(th) : -halfL * Math.cos(th);
                    var y2 = collided ? -(2 * p.L / 3) * Math.sin(th) : -halfL * Math.sin(th);

                    var r1_cm = collided ? p.L / 3 : halfL;
                    var r2_cm = collided ? 2 * p.L / 3 : halfL;
                    var v1x = -w * r1_cm * Math.sin(th), v1y = w * r1_cm * Math.cos(th);
                    var v2x = w * r2_cm * Math.sin(th), v2y = -w * r2_cm * Math.cos(th);

                    var span = p.L * 0.95;
                    var masses = [
                        { x: x2, y: y2, r: 0.06 * p.L, color: '#38bdf8', label: 'M₂' }
                    ];
                    if (!collided) {
                        masses.push({ x: x1, y: y1, r: 0.06 * p.L, color: '#fb7185', label: 'M₁' });
                        masses.push({ x: halfL, y: 0, r: 0.06 * p.L, color: '#94a3b8', label: 'M₃ (reposo)' });
                    } else {
                        masses.push({ x: x1, y: y1, r: 0.08 * p.L, color: '#a855f7', label: 'M₁+M₃ (2M)' });
                    }

                    return {
                        bounds: { minX: -span, maxX: span, minY: -span, maxY: span },
                        rods: [{ x1: x2, y1: y2, x2: x1, y2: y1, color: '#cbd5e1', width: 4 }],
                        trails: [
                            { pts: trails.m1, color: 'rgba(251, 113, 133, 0.5)', width: 2 },
                            { pts: trails.m2, color: 'rgba(56, 189, 248, 0.5)', width: 2 },
                            { pts: trails.centerRod, color: 'rgba(148, 163, 184, 0.55)', width: 1.5, dash: [4, 4] }
                        ],
                        masses: masses,
                        cm: { x: rCM, y: 0, label: 'CM (L/6, 0)' },
                        vectors: [
                            { type: 'velocity', x: x1, y: y1, vx: v1x * 0.14, vy: v1y * 0.14, color: '#22d3ee', label: 'v₁' },
                            { type: 'velocity', x: x2, y: y2, vx: v2x * 0.14, vy: v2y * 0.14, color: '#22d3ee', label: 'v₂' }
                        ],
                        banner: collided
                            ? 'Tras captura: CM en (L/6, 0) QUIETO; el centro de la barra orbita alrededor del CM a ω_f = ¾ω'
                            : 'Pre-choque: M₁ se dirige a M₃ en (L/2, 0); el CM del sistema de 3 masas ya está quieto en (L/6, 0)'
                    };
                }
            };
        }
    };

    return [t5_p3, t5_p9, t5_p6, t5_p12, t5_p16, t5_p15];
}));
