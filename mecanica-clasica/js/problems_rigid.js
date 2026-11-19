/**
 * problems_rigid.js — 7 simulaciones progresivas de Sólido Rígido
 * Basado en "Hoja_problemas_team6.pdf" (Fundamentos de Física I).
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory(require('./rk4.js'));
    } else {
        root.RigidProblems = factory(root.PhysicsRK4);
    }
}(typeof self !== 'undefined' ? self : this, function (RK4) {
    'use strict';

    var rk4Integrate = RK4.rk4Integrate;
    var bisectEvent = RK4.bisectEvent;
    var degToRad = RK4.degToRad;
    var radToDeg = RK4.radToDeg;
    var fmt = RK4.fmt;

    // =========================================================================
    // NIVEL 1 — T6 · P2: Cilindro Macizo vs Hueco en Plano Inclinado
    // =========================================================================
    var t6_p2 = {
        id: 't6_p2',
        topic: 'rigid',
        level: 1,
        code: 'T6 · P2',
        title: 'Cilindro Macizo vs Hueco en Plano Inclinado',
        subtitle: 'Reparto entre energía de traslación y rotación; rodadura vs deslizamiento',
        renderMode: '2d',
        learningGoal: 'Entender cómo el momento de inercia I = c·M·R² frena la aceleración lineal al rodar: cuanto mayor es c, más energía potencial se invierte en girar en lugar de avanzar.',
        guideFromZero: [
            '<strong>1. ¡Atención al ángulo α!:</strong> El enunciado mide α respecto a la <strong>vertical</strong>, luego la pendiente respecto a la horizontal es <code>β = 90° − α</code> y <code>sin β = cos α</code>.',
            '<strong>2. Rodando sin deslizar (v = ωR):</strong> Por energía: <code>Mgh = ½Mv² + ½(cMR²)(v/R)² = ½(1+c)Mv²</code>. El macizo (c=½) acelera a <code>⅔ g cos α</code>; el hueco (c=1) solo a <code>½ g cos α</code>.',
            '<strong>3. Sin rozamiento:</strong> No hay par de fuerzas (<code>τ = 0</code>), ninguno gira y ambos deslizan con <code>a = g cos α</code>, llegando al mismo tiempo.'
        ],
        statement: 'Dos cilindros, uno hueco (I = MR²) y el otro macizo (I = ½MR²), poseen igual masa M y radio R. Se los hace rodar sin deslizar desde una altura h a lo largo de un plano inclinado que forma con la vertical un ángulo α. ¿Cuál llegará antes al suelo? Repetir el problema si no existe rozamiento.',
        paramsSchema: [
            { key: 'M', label: 'Masa de cada cilindro M', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 2.0 },
            { key: 'R', label: 'Radio R', unit: 'm', min: 0.1, max: 0.6, step: 0.05, def: 0.25 },
            { key: 'h', label: 'Altura inicial h', unit: 'm', min: 1.0, max: 8.0, step: 0.5, def: 3.0 },
            { key: 'alphaDeg', label: 'Ángulo α con la VERTICAL', unit: '°', min: 20, max: 75, step: 1, def: 60 },
            { key: 'withFriction', label: 'Rozamiento (1=Rueda, 0=Desliza)', unit: '', min: 0, max: 1, step: 1, def: 1 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'Con rozamiento (rodadura)', apply: function (p) { p.withFriction = 1; } },
            { label: 'Sin rozamiento (deslizamiento)', apply: function (p) { p.withFriction = 0; } }
        ],
        defaultDuration: 4.0,

        solveAnalytical: function (p) {
            var al = degToRad(p.alphaDeg);
            var cosA = Math.cos(al);
            var betaDeg = 90 - p.alphaDeg;
            var L = p.h / cosA;
            var rolling = p.withFriction >= 0.5;

            var aSolid = rolling ? (2 / 3) * p.g * cosA : p.g * cosA;
            var aHollow = rolling ? (1 / 2) * p.g * cosA : p.g * cosA;
            var tSolid = Math.sqrt((2 * L) / aSolid);
            var tHollow = Math.sqrt((2 * L) / aHollow);
            var vSolid = aSolid * tSolid;
            var vHollow = aHollow * tHollow;
            var frSolid = rolling ? (1 / 3) * p.M * p.g * cosA : 0;
            var frHollow = rolling ? (1 / 2) * p.M * p.g * cosA : 0;

            return {
                steps: [
                    {
                        title: '1. Geometría del plano (α = ' + p.alphaDeg + '° respecto a la vertical)',
                        formula: 'β_horiz = 90° − α = ' + fmt(betaDeg, 1) + '°  ;  L_rampa = h / cos α = ' + fmt(L, 3) + ' m',
                        detail: 'Componente de gravedad paralela al plano: g·cos α = ' + fmt(p.g * cosA, 3) + ' m/s²'
                    },
                    {
                        title: rolling ? '2. Aceleración rodando sin deslizar: a = g cos α / (1 + c)' : '2. Sin rozamiento: deslizamiento puro (τ = 0)',
                        formula: rolling ? 'Macizo (c = ½): a = ⅔ g cos α  ;  Hueco (c = 1): a = ½ g cos α' : 'Ambos: a = g cos α (ω = 0)',
                        detail: 'a_macizo = ' + fmt(aSolid, 4) + ' m/s²  ;  a_hueco = ' + fmt(aHollow, 4) + ' m/s²'
                    },
                    {
                        title: '3. Tiempo de llegada al suelo y velocidad final',
                        formula: 't = √(2 L_rampa / a)  ;  v_final = a · t',
                        detail: 't_macizo = ' + fmt(tSolid, 4) + ' s (v = ' + fmt(vSolid, 3) + ' m/s)  |  t_hueco = ' + fmt(tHollow, 4) + ' s (v = ' + fmt(vHollow, 3) + ' m/s) → ' + (rolling ? '¡Llega ANTES el MACIZO!' : '¡EMPATE exacto!')
                    },
                    {
                        title: '4. Fuerza de rozamiento estática y reparto energético',
                        formula: 'f_r = [c/(1+c)] M g cos α',
                        detail: 'f_r,macizo = ' + fmt(frSolid, 3) + ' N (33.3% rot.)  ;  f_r,hueco = ' + fmt(frHollow, 3) + ' N (50.0% rot.)'
                    }
                ],
                values: { L: L, aSolid: aSolid, aHollow: aHollow, tSolid: tSolid, tHollow: tHollow, vSolid: vSolid, vHollow: vHollow }
            };
        },

        createSimulation: function (p) {
            var al = degToRad(p.alphaDeg);
            var cosA = Math.cos(al);
            var sinA = Math.sin(al);
            var L = p.h / cosA;
            var rolling = p.withFriction >= 0.5;
            var a1 = rolling ? (2 / 3) * p.g * cosA : p.g * cosA;
            var a2 = rolling ? (1 / 2) * p.g * cosA : p.g * cosA;

            // y = [s1, v1, phi1, s2, v2, phi2]
            var y = [0, 0, 0, 0, 0, 0];
            var t = 0;
            var tFinish1 = NaN, tFinish2 = NaN;
            var done1 = false, done2 = false;

            function derivs(st) {
                return [
                    done1 ? 0 : st[1],
                    done1 ? 0 : a1,
                    done1 ? 0 : (rolling ? st[1] / p.R : 0),
                    done2 ? 0 : st[4],
                    done2 ? 0 : a2,
                    done2 ? 0 : (rolling ? st[4] / p.R : 0)
                ];
            }

            return {
                step: function (dt) {
                    if (done1 && done2) return;
                    var yNext = rk4Integrate(y, t, dt, 0.004, derivs);
                    if (!done1 && yNext[0] >= L) {
                        var ev1 = bisectEvent(y, t, dt, derivs, function (st) { return st[0] - L; }, 22);
                        tFinish1 = ev1.tEvent;
                        yNext[0] = L;
                        yNext[1] = ev1.yEvent[1];
                        yNext[2] = ev1.yEvent[2];
                        done1 = true;
                    }
                    if (!done2 && yNext[3] >= L) {
                        var ev2 = bisectEvent(y, t, dt, derivs, function (st) { return st[3] - L; }, 22);
                        tFinish2 = ev2.tEvent;
                        yNext[3] = L;
                        yNext[4] = ev2.yEvent[4];
                        yNext[5] = ev2.yEvent[5];
                        done2 = true;
                    }
                    y = yNext;
                    t += dt;
                },
                getObservables: function () {
                    var h1 = Math.max(0, p.h - y[0] * cosA);
                    var h2 = Math.max(0, p.h - y[3] * cosA);
                    var w1 = rolling && !done1 ? y[1] / p.R : (rolling ? y[1] / p.R : 0);
                    var w2 = rolling && !done2 ? y[4] / p.R : (rolling ? y[4] / p.R : 0);
                    var Etras1 = 0.5 * p.M * y[1] * y[1];
                    var Erot1 = 0.5 * (0.5 * p.M * p.R * p.R) * w1 * w1;
                    var Etras2 = 0.5 * p.M * y[4] * y[4];
                    var Erot2 = 0.5 * (p.M * p.R * p.R) * w2 * w2;
                    var Ep = p.M * p.g * (h1 + h2);
                    var Ec = Etras1 + Erot1 + Etras2 + Erot2;
                    return {
                        t: t, Ec: Ec, Etras: Etras1 + Etras2, Erot: Erot1 + Erot2, Ep: Ep, Etot: Ec + Ep,
                        Px: p.M * (y[1] + y[4]) * sinA, Py: -p.M * (y[1] + y[4]) * cosA, P: p.M * (y[1] + y[4]), Lz: (0.5 * p.M * p.R * p.R) * w1 + (p.M * p.R * p.R) * w2,
                        cm: { x: 0, y: 0 },
                        finished: done1 && done2,
                        extra: [
                            { label: 'Macizo v / E_rot%', value: fmt(y[1], 2) + ' m/s (' + (rolling ? '33%' : '0%') + ')' },
                            { label: 'Hueco v / E_rot%', value: fmt(y[4], 2) + ' m/s (' + (rolling ? '50%' : '0%') + ')' },
                            { label: 'Llegada M / H', value: (isNaN(tFinish1) ? '—' : fmt(tFinish1, 3) + 's') + ' / ' + (isNaN(tFinish2) ? '—' : fmt(tFinish2, 3) + 's') }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t6_p2.solveAnalytical(p).values;
                    return [
                        { label: 't_llegada Macizo', unit: 's', theo: ana.tSolid, sim: tFinish1 },
                        { label: 't_llegada Hueco', unit: 's', theo: ana.tHollow, sim: tFinish2 },
                        { label: 'v_final Macizo', unit: 'm/s', theo: ana.vSolid, sim: done1 ? y[1] : NaN },
                        { label: 'v_final Hueco', unit: 'm/s', theo: ana.vHollow, sim: done2 ? y[4] : NaN }
                    ];
                },
                getRenderState: function () {
                    // Ramp from (0, h) to (xEnd, 0) where xEnd = h * tan(alpha)
                    var xEnd = p.h * Math.tan(al);
                    // Unit tangent down-slope: (sinA, -cosA), unit normal outward: (cosA, sinA)
                    var tx = sinA, ty = -cosA;
                    var nx = cosA, ny = sinA;
                    // Shift upper lane vertically enough so perpendicular ramp separation >= 2.7*R even at steep alpha
                    var laneGap = (2.7 * p.R) / Math.max(0.32, sinA);

                    var c1x = y[0] * tx + p.R * nx;
                    var c1y = p.h + y[0] * ty + p.R * ny;

                    var c2x = y[3] * tx + p.R * nx;
                    var c2y = p.h + laneGap + y[3] * ty + p.R * ny;

                    var fr1 = rolling ? (1 / 3) * p.M * p.g * cosA : 0;
                    var fr2 = rolling ? (1 / 2) * p.M * p.g * cosA : 0;
                    var vecScale = 0.08;

                    var vectors = [
                        { type: 'velocity', x: c1x, y: c1y, vx: y[1] * tx * 0.25, vy: y[1] * ty * 0.25, color: '#22d3ee', label: 'v_M' },
                        { type: 'velocity', x: c2x, y: c2y, vx: y[4] * tx * 0.25, vy: y[4] * ty * 0.25, color: '#fb923c', label: 'v_H' }
                    ];
                    if (rolling) {
                        vectors.push({ type: 'force', x: c1x - p.R * nx, y: c1y - p.R * ny, vx: -fr1 * tx * vecScale, vy: -fr1 * ty * vecScale, color: '#ef4444', label: 'f_r,M' });
                        vectors.push({ type: 'force', x: c2x - p.R * nx, y: c2y - p.R * ny, vx: -fr2 * tx * vecScale, vy: -fr2 * ty * vecScale, color: '#ef4444', label: 'f_r,H' });
                    }

                    return {
                        bounds: { minX: -0.8, maxX: xEnd + 1.4, minY: -0.5, maxY: p.h + laneGap + 1.2 },
                        lines: [
                            // Lower lane (Macizo)
                            { x1: 0, y1: p.h, x2: xEnd, y2: 0, color: '#475569', width: 4 },
                            { x1: 0, y1: 0, x2: xEnd + 1, y2: 0, color: '#334155', width: 3 },
                            { x1: 0, y1: 0, x2: 0, y2: p.h, color: 'rgba(148,163,184,0.3)', width: 1.5, dash: [4, 4] },
                            // Upper lane (Hueco)
                            { x1: 0, y1: p.h + laneGap, x2: xEnd, y2: laneGap, color: 'rgba(100,116,139,0.55)', width: 3, dash: [6, 4] }
                        ],
                        wheels: [
                            { x: c1x, y: c1y, r: p.R, phi: y[2], hollow: false, color: '#38bdf8', label: 'Macizo (I=½MR²)' },
                            { x: c2x, y: c2y, r: p.R, phi: y[5], hollow: true, color: '#fbbf24', label: 'Hueco (I=MR²)' }
                        ],
                        vectors: vectors,
                        banner: rolling
                            ? (done1 && !done2 ? '¡El cilindro MACIZO ganó la carrera! (menor I → menos energía en giro)' : 'Rodadura sin deslizar: α_vertical = ' + p.alphaDeg + '°')
                            : 'Sin rozamiento: ninguno gira (ω=0), ¡ambos deslizan empatados!'
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 2 — T6 · P8: Cilindro con Cuerda Arrollada y Bloque que Desciende
    // =========================================================================
    var t6_p8 = {
        id: 't6_p8',
        topic: 'rigid',
        level: 2,
        code: 'T6 · P8',
        title: 'Cilindro con Cuerda y Bloque Descendente',
        subtitle: 'Acoplamiento traslación-rotación (a = αR) y conservación energética',
        renderMode: '2d',
        learningGoal: 'Conectar la 2ª Ley de Newton de traslación del bloque (mg − T = ma) con la ecuación rotacional del cilindro (TR = Iα) mediante la ligadura cinemática a = αR.',
        guideFromZero: [
            '<strong>1. Dinámica del bloque:</strong> Cae con aceleración <code>a</code>: <code>m g − T = m a</code>, luego la tensión siempre es menor que el peso (<code>T < mg</code>).',
            '<strong>2. Dinámica del cilindro:</strong> La cuerda ejerce un par <code>τ = T·R = I·α</code> con <code>α = a / R</code>.',
            '<strong>3. Conservación de energía:</strong> La energía potencial perdida por el bloque <code>mgs</code> se reparte entre la cinética del bloque (<code>½mv²</code>) y la rotacional del cilindro (<code>½Iω²</code>).'
        ],
        statement: 'Un cilindro macizo de 30 cm de diámetro (R = 0.15 m) gira sin rozamiento en cojinetes horizontales. Una cuerda arrollada sostiene un bloque de m = 8 kg que desciende desde el reposo s = 63 m en 5 s. Hallar: (a) Tensión de la cuerda. (b) Momento de inercia del cilindro. (c) Energías intervinientes por separado.',
        paramsSchema: [
            { key: 'm', label: 'Masa del bloque m', unit: 'kg', min: 1, max: 25, step: 0.5, def: 8.0 },
            { key: 'R', label: 'Radio del cilindro R', unit: 'm', min: 0.05, max: 0.5, step: 0.01, def: 0.15 },
            { key: 'I', label: 'Inercia del cilindro I', unit: 'kg·m²', min: 0.02, max: 1.5, step: 0.01, def: 0.17 },
            { key: 'sTarget', label: 'Altura de descenso s', unit: 'm', min: 10, max: 100, step: 1, def: 63 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        defaultDuration: 6.0,

        solveAnalytical: function (p) {
            var a = (p.m * p.g) / (p.m + p.I / (p.R * p.R));
            var T = p.m * (p.g - a);
            var tFinal = Math.sqrt((2 * p.sTarget) / a);
            var vFinal = a * tFinal;
            var wFinal = vFinal / p.R;
            var dEp = p.m * p.g * p.sTarget;
            var EcBlock = 0.5 * p.m * vFinal * vFinal;
            var EcCyl = 0.5 * p.I * wFinal * wFinal;

            return {
                steps: [
                    {
                        title: '1. Aceleración del bloque y ligadura a = αR',
                        formula: 'a = m·g / (m + I/R²)  ;  (con s = ½at² = 63 m en 5 s → a = 5.04 m/s²)',
                        detail: 'a = ' + fmt(a, 4) + ' m/s²  ;  tiempo para descender ' + fmt(p.sTarget, 1) + ' m: t = ' + fmt(tFinal, 4) + ' s'
                    },
                    {
                        title: '(a) Tensión de la cuerda',
                        formula: 'T = m(g − a)',
                        detail: 'T = ' + fmt(T, 3) + ' N  (< mg = ' + fmt(p.m * p.g, 2) + ' N)'
                    },
                    {
                        title: '(b) Momento de inercia del cilindro',
                        formula: 'I = T·R² / a',
                        detail: 'I = ' + fmt((T * p.R * p.R) / a, 4) + ' kg·m²  (el cilindro debe acelerar angularmente a α = ' + fmt(a / p.R, 2) + ' rad/s²)'
                    },
                    {
                        title: '(c) Conservación de la energía al descender s = ' + fmt(p.sTarget, 1) + ' m',
                        formula: 'm g s = ½ m v² + ½ I ω²',
                        detail: '|ΔE_p| = ' + fmt(dEp, 2) + ' J  =  E_c,bloque (' + fmt(EcBlock, 2) + ' J) + E_c,cilindro (' + fmt(EcCyl, 2) + ' J)'
                    }
                ],
                values: { a: a, T: T, tFinal: tFinal, vFinal: vFinal, wFinal: wFinal, dEp: dEp, EcBlock: EcBlock, EcCyl: EcCyl }
            };
        },

        createSimulation: function (p) {
            var a = (p.m * p.g) / (p.m + p.I / (p.R * p.R));
            var T = p.m * (p.g - a);
            // y = [s, v, phi]
            var y = [0, 0, 0];
            var t = 0;
            var finished = false;
            var tStop = NaN;

            function derivs(st) {
                return [st[1], a, st[1] / p.R];
            }

            return {
                step: function (dt) {
                    if (finished) return;
                    var yNext = rk4Integrate(y, t, dt, 0.005, derivs);
                    if (yNext[0] >= p.sTarget) {
                        var ev = bisectEvent(y, t, dt, derivs, function (st) { return st[0] - p.sTarget; }, 24);
                        y = ev.yEvent;
                        y[0] = p.sTarget;
                        t = ev.tEvent;
                        tStop = t;
                        finished = true;
                    } else {
                        y = yNext;
                        t += dt;
                    }
                },
                getObservables: function () {
                    var w = y[1] / p.R;
                    var EcBlock = 0.5 * p.m * y[1] * y[1];
                    var EcCyl = 0.5 * p.I * w * w;
                    var Ep = p.m * p.g * Math.max(0, p.sTarget - y[0]);
                    return {
                        t: t,
                        Ec: EcBlock + EcCyl,
                        Etras: EcBlock,
                        Erot: EcCyl,
                        Ep: Ep,
                        Etot: EcBlock + EcCyl + Ep,
                        Px: 0, Py: -p.m * y[1], P: p.m * y[1], Lz: p.I * w,
                        cm: { x: 0, y: 0 },
                        finished: finished,
                        extra: [
                            { label: 'Descenso s', value: fmt(y[0], 2) + ' / ' + p.sTarget + ' m' },
                            { label: 'Velocidad v / ω', value: fmt(y[1], 2) + ' m/s | ' + fmt(w, 1) + ' rad/s' },
                            { label: 'Tensión T', value: fmt(T, 2) + ' N' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t6_p8.solveAnalytical(p).values;
                    var w = y[1] / p.R;
                    return [
                        { label: 'Tiempo descenso', unit: 's', theo: ana.tFinal, sim: tStop },
                        { label: 'E_c bloque', unit: 'J', theo: ana.EcBlock, sim: finished ? 0.5 * p.m * y[1] * y[1] : NaN },
                        { label: 'E_c cilindro', unit: 'J', theo: ana.EcCyl, sim: finished ? 0.5 * p.I * w * w : NaN },
                        { label: 'E_total conservada', unit: 'J', theo: ana.dEp, sim: 0.5 * p.m * y[1] * y[1] + 0.5 * p.I * w * w + p.m * p.g * (p.sTarget - y[0]) }
                    ];
                },
                getRenderState: function () {
                    // Use clean schematic scale so 0.15 m cylinder & 63 m drop are both legible simultaneously!
                    // Map descent s in [0, sTarget] to schematic drop [0, 8.5] units below pulley at (0, 10)
                    var pulleyY = 10.0;
                    var visR = 1.35;
                    var dropSpan = 7.65;
                    var blockY = pulleyY - 1.5 - (y[0] / p.sTarget) * dropSpan;
                    var ropeX = visR;
                    var w = y[1] / p.R;

                    return {
                        bounds: { minX: -5, maxX: 6, minY: -0.5, maxY: 12.2 },
                        lines: [
                            // Ground / bottom reference
                            { x1: -4.5, y1: 0.3, x2: 5.5, y2: 0.3, color: '#334155', width: 3 },
                            // Bearing support
                            { x1: -2.2, y1: pulleyY, x2: 0, y2: pulleyY, color: '#64748b', width: 6 },
                            // Unwinding rope
                            { x1: ropeX, y1: pulleyY, x2: ropeX, y2: blockY + 0.55, color: '#fbbf24', width: 2.5 }
                        ],
                        wheels: [
                            { x: 0, y: pulleyY, r: visR, phi: y[2], hollow: false, color: '#38bdf8', label: 'Cilindro (I=' + p.I + ' kg·m²)' }
                        ],
                        boxes: [
                            { x: ropeX, y: blockY, w: 1.1, h: 1.1, color: '#fb7185', label: 'm = ' + p.m + ' kg (s=' + fmt(y[0], 1) + ' m)' }
                        ],
                        vectors: [
                            { type: 'force', x: ropeX, y: blockY - 0.55, vx: 0, vy: -1.6, color: '#ef4444', label: 'mg' },
                            { type: 'tension', x: ropeX, y: blockY + 0.55, vx: 0, vy: 1.1, color: '#fbbf24', label: 'T=' + fmt(T, 1) + 'N' },
                            { type: 'velocity', x: ropeX + 0.85, y: blockY, vx: 0, vy: -y[1] * 0.08, color: '#22d3ee', label: 'v' }
                        ],
                        banner: 'Esquema escalado | ω = ' + fmt(w, 1) + ' rad/s | E_c,bloque = ' + fmt(0.5 * p.m * y[1] * y[1], 0) + ' J, E_c,cil = ' + fmt(0.5 * p.I * w * w, 0) + ' J'
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 3a — T6 · P3: Plataforma Giratoria: Encoger los Brazos (3D)
    // =========================================================================
    var t6_p3 = {
        id: 't6_p3',
        topic: 'rigid',
        level: 3,
        code: 'T6 · P3',
        title: 'Plataforma Giratoria: Encoger los Brazos (3D)',
        subtitle: 'Conservación de L = I·ω y aumento de energía cinética por trabajo interno',
        renderMode: '3d-p3',
        learningGoal: 'Comprobar en 3D que reducir el momento de inercia (r₀ → r_f) manteniendo L constante multiplica ω y AUMENTA la energía cinética E_c = L²/(2I) gracias al trabajo muscular radial.',
        guideFromZero: [
            '<strong>1. Conservación de L:</strong> Al no haber pares externos alrededor del eje vertical, <code>L = I(r)·ω = constante</code>.',
            '<strong>2. Momento de inercia variable:</strong> <code>I(r) = I_hombre + 2 m_pesa r²</code>. Al acercar las pesas de 80 cm a 20 cm, I disminuye de 9.84 a 6.24 kg·m².',
            '<strong>3. ¡La energía cinética NO se conserva!:</strong> <code>E_c = L² / (2I)</code> aumenta porque el hombre realiza trabajo positivo tirando hacia dentro contra la tendencia centrífuga.'
        ],
        statement: 'Un hombre en una plataforma giratoria sostiene una pesa de 3 kg en cada mano y gira a 1 vuelta/s. Su momento de inercia constante es 6 kg·m². La distancia inicial de cada pesa al eje es 80 cm y la final 20 cm. Sin rozamiento, calcular la velocidad angular final.',
        paramsSchema: [
            { key: 'I_man', label: 'Inercia del hombre I_h', unit: 'kg·m²', min: 2, max: 15, step: 0.5, def: 6.0 },
            { key: 'm_weight', label: 'Cada pesa m', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 3.0 },
            { key: 'rev0', label: 'Velocidad inicial', unit: 'rev/s', min: 0.2, max: 3.0, step: 0.1, def: 1.0 },
            { key: 'r0', label: 'Distancia inicial brazos r₀', unit: 'm', min: 0.3, max: 1.2, step: 0.05, def: 0.80 },
            { key: 'rf', label: 'Distancia final brazos r_f', unit: 'm', min: 0.1, max: 0.8, step: 0.05, def: 0.20 },
            { key: 'pullDuration', label: 'Duración retracción', unit: 's', min: 0.5, max: 5.0, step: 0.5, def: 2.0 }
        ],
        defaultDuration: 8.0,

        solveAnalytical: function (p) {
            var w0 = 2 * Math.PI * p.rev0;
            var I0 = p.I_man + 2 * p.m_weight * p.r0 * p.r0;
            var If = p.I_man + 2 * p.m_weight * p.rf * p.rf;
            var L = I0 * w0;
            var wf = L / If;
            var revF = wf / (2 * Math.PI);
            var Ec0 = 0.5 * I0 * w0 * w0;
            var Ecf = 0.5 * If * wf * wf;
            var Wmusc = Ecf - Ec0;

            return {
                steps: [
                    {
                        title: '1. Inercias inicial y final del sistema',
                        formula: 'I(r) = I_hombre + 2 m r²',
                        detail: 'I₀ = ' + fmt(I0, 3) + ' kg·m² (r₀=' + fmt(p.r0, 2) + ' m)  →  I_f = ' + fmt(If, 3) + ' kg·m² (r_f=' + fmt(p.rf, 2) + ' m)'
                    },
                    {
                        title: '2. Conservación del momento angular L',
                        formula: 'L = I₀ ω₀ = I_f ω_f',
                        detail: 'L = ' + fmt(L, 3) + ' kg·m²/s  →  ω_f = (I₀/I_f) ω₀ = ' + fmt(wf, 4) + ' rad/s (' + fmt(revF, 4) + ' rev/s)'
                    },
                    {
                        title: '3. Aumento de energía cinética (trabajo muscular)',
                        formula: 'W_interno = ΔE_c = ½ I_f ω_f² − ½ I₀ ω₀²',
                        detail: 'E_c,0 = ' + fmt(Ec0, 2) + ' J → E_c,f = ' + fmt(Ecf, 2) + ' J  (ΔE_c = +' + fmt(Wmusc, 2) + ' J)'
                    }
                ],
                values: { I0: I0, If: If, w0: w0, wf: wf, revF: revF, L: L, Ec0: Ec0, Ecf: Ecf, Wmusc: Wmusc }
            };
        },

        createSimulation: function (p) {
            var w0 = 2 * Math.PI * p.rev0;
            var I0 = p.I_man + 2 * p.m_weight * p.r0 * p.r0;
            var L = I0 * w0;
            var tStartPull = 1.5;
            var y = [0]; // [theta]
            var t = 0;

            function currentR(curT) {
                if (curT <= tStartPull) return p.r0;
                var u = (curT - tStartPull) / p.pullDuration;
                if (u >= 1) return p.rf;
                // Smooth cosine transition
                var s = 0.5 * (1 - Math.cos(Math.PI * u));
                return p.r0 + (p.rf - p.r0) * s;
            }

            function currentI(curT) {
                var r = currentR(curT);
                return p.I_man + 2 * p.m_weight * r * r;
            }

            function derivs(st, curT) {
                return [L / currentI(curT)];
            }

            return {
                step: function (dt) {
                    y = rk4Integrate(y, t, dt, 0.005, derivs);
                    t += dt;
                },
                getObservables: function () {
                    var r = currentR(t);
                    var Icur = currentI(t);
                    var w = L / Icur;
                    var Ec = 0.5 * Icur * w * w;
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: 0, Py: 0, P: 0, Lz: L,
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'Radio pesas r(t)', value: fmt(r, 2) + ' m' },
                            { label: 'Inercia I(t)', value: fmt(Icur, 2) + ' kg·m²' },
                            { label: 'ω(t)', value: fmt(w, 3) + ' rad/s (' + fmt(w / (2 * Math.PI), 2) + ' rev/s)' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t6_p3.solveAnalytical(p).values;
                    var Icur = currentI(t);
                    var w = L / Icur;
                    var pulled = t >= tStartPull + p.pullDuration;
                    return [
                        { label: 'L (conservado)', unit: 'kg·m²/s', theo: ana.L, sim: Icur * w },
                        { label: 'ω final', unit: 'rad/s', theo: ana.wf, sim: pulled ? w : NaN },
                        { label: 'ω final (rev/s)', unit: 'rev/s', theo: ana.revF, sim: pulled ? w / (2 * Math.PI) : NaN },
                        { label: 'E_c final', unit: 'J', theo: ana.Ecf, sim: pulled ? 0.5 * Icur * w * w : NaN }
                    ];
                },
                getRenderState: function () {
                    var r = currentR(t);
                    var Icur = currentI(t);
                    var w = L / Icur;
                    return {
                        type: '3d-p3',
                        theta: y[0],
                        r: r,
                        omega: w,
                        L: L,
                        mWeight: p.m_weight,
                        banner: t < tStartPull
                            ? 'Brazos en r₀ = ' + fmt(p.r0, 2) + ' m → ω = ' + fmt(w, 2) + ' rad/s'
                            : (t < tStartPull + p.pullDuration
                                ? (p.rf <= p.r0
                                    ? 'Encogiendo brazos → I baja, ¡ω y E_c suben con L constante!'
                                    : 'Extendiendo brazos → I sube, ω y E_c bajan con L constante')
                                : 'Posición final (r_f = ' + fmt(p.rf, 2) + ' m) → ω_f = ' + fmt(w, 2) + ' rad/s')
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 3b — T6 · P4: Plataforma: Lanzamiento Tangencial y Caminata (3D)
    // =========================================================================
    var t6_p4 = {
        id: 't6_p4',
        topic: 'rigid',
        level: 4,
        code: 'T6 · P4',
        title: 'Plataforma: Lanzamiento Tangencial y Caminata (3D)',
        subtitle: 'Conservación de L = 0 al lanzar una masa y aceleración angular al caminar hacia el centro',
        renderMode: '3d-p4',
        learningGoal: 'Combinar dos procesos secuenciales de conservación de L: primero el retroceso rotacional al lanzar una masa (L_total = 0) y después el aumento de ω al disminuir I.',
        guideFromZero: [
            '<strong>1. En el lanzamiento (t = 1 s):</strong> El sistema parte del reposo (<code>L_total = 0</code>). La masa lanzada lleva <code>L_masa = +m_b v_b R</code>, luego plataforma+hombre retroceden con <code>L_p+h = −m_b v_b R</code>.',
            '<strong>2. Inercia en el borde:</strong> <code>I_a = ½ M_plat R² + M_h R²</code>.',
            '<strong>3. Caminata hacia R/2:</strong> Ya en vuelo la masa, plataforma+hombre forman un sistema aislado con <code>L_p+h</code> constante mientras I baja a <code>I_b = ½ M_plat R² + M_h (R/2)²</code>.'
        ],
        statement: 'Un hombre de 80 kg está en el borde de una plataforma circular de 140 kg y 8 m de diámetro (R = 4 m) en reposo. Arroja tangencialmente una masa de 3 kg a 20 m/s. (a) ¿Qué velocidad angular adquiere la plataforma? (b) El hombre camina hasta la mitad del radio (R/2): ¿cuál es la nueva ω?',
        paramsSchema: [
            { key: 'M_man', label: 'Masa del hombre', unit: 'kg', min: 40, max: 120, step: 5, def: 80 },
            { key: 'M_plat', label: 'Masa de la plataforma', unit: 'kg', min: 50, max: 300, step: 10, def: 140 },
            { key: 'D', label: 'Diámetro plataforma D', unit: 'm', min: 4, max: 14, step: 0.5, def: 8.0 },
            { key: 'm_proj', label: 'Masa arrojada m_b', unit: 'kg', min: 0.5, max: 15, step: 0.5, def: 3.0 },
            { key: 'v_proj', label: 'Velocidad lanzamiento v_b', unit: 'm/s', min: 5, max: 40, step: 1, def: 20 },
            { key: 'rRatio', label: 'Radio final hombre (r_f / R)', unit: '', min: 0.1, max: 1.0, step: 0.05, def: 0.5 }
        ],
        defaultDuration: 9.0,

        solveAnalytical: function (p) {
            var R = 0.5 * p.D;
            var Iplat = 0.5 * p.M_plat * R * R;
            var Ia = Iplat + p.M_man * R * R;
            var Lthrow = p.m_proj * p.v_proj * R;
            var wa = Lthrow / Ia; // magnitude (opposite sense to throw)
            var rf = p.rRatio * R;
            var Ib = Iplat + p.M_man * rf * rf;
            var wb = Lthrow / Ib;

            return {
                steps: [
                    {
                        title: '1. Momento de inercia inicial (hombre en el borde R = ' + fmt(R, 2) + ' m)',
                        formula: 'I_a = ½ M_plat R² + M_h R²',
                        detail: 'I_plat = ' + fmt(Iplat, 1) + ' kg·m²  ;  I_hombre(R) = ' + fmt(p.M_man * R * R, 1) + ' kg·m²  →  I_a = ' + fmt(Ia, 1) + ' kg·m²'
                    },
                    {
                        title: '(a) Velocidad angular tras lanzar la masa tangencialmente',
                        formula: '0 = m_b v_b R − I_a ω_a  →  ω_a = m_b v_b R / I_a',
                        detail: 'L_masa = ' + fmt(Lthrow, 1) + ' kg·m²/s  →  ω_a = ' + fmt(wa, 4) + ' rad/s (sentido opuesto al lanzamiento)'
                    },
                    {
                        title: '(b) Nueva ω cuando el hombre llega a r_f = ' + fmt(rf, 2) + ' m',
                        formula: 'I_b = I_plat + M_h r_f²  ;  ω_b = (I_a / I_b) ω_a',
                        detail: 'I_b = ' + fmt(Ib, 1) + ' kg·m²  →  ω_b = ' + fmt(wb, 4) + ' rad/s'
                    }
                ],
                values: { R: R, Iplat: Iplat, Ia: Ia, Ib: Ib, Lthrow: Lthrow, wa: wa, wb: wb }
            };
        },

        createSimulation: function (p) {
            var R = 0.5 * p.D;
            var Iplat = 0.5 * p.M_plat * R * R;
            var Lthrow = p.m_proj * p.v_proj * R;
            var rf = p.rRatio * R;

            var tThrow = 1.0;
            var tWalkStart = 2.5;
            var tWalkEnd = 5.5;

            var y = [0]; // [thetaPlat] (negative = recoil opposite to +theta throw)
            var t = 0;
            var projPos = { x: R, y: 0, z: 0 };
            var projVel = { x: 0, y: 0, z: 0 };
            var thrown = false;
            var simWa = NaN;

            function manRadius(curT) {
                if (curT <= tWalkStart) return R;
                if (curT >= tWalkEnd) return rf;
                var u = (curT - tWalkStart) / (tWalkEnd - tWalkStart);
                var s = 0.5 * (1 - Math.cos(Math.PI * u));
                return R + (rf - R) * s;
            }

            function currentI(curT) {
                var r = manRadius(curT);
                return Iplat + p.M_man * r * r;
            }

            function currentOmega(curT) {
                if (curT < tThrow) return 0;
                return -Lthrow / currentI(curT);
            }

            function derivs(st, curT) {
                return [currentOmega(curT)];
            }

            return {
                step: function (dt) {
                    var rem = dt;
                    if (!thrown && t + rem >= tThrow) {
                        var dtPre = tThrow - t;
                        t = tThrow;
                        rem -= dtPre;
                        thrown = true;
                        simWa = Math.abs(currentOmega(tThrow));
                        projPos = { x: R, y: 0, z: 0 };
                        projVel = { x: 0, y: p.v_proj, z: 0 }; // tangential +Y (CCW)
                    }
                    if (rem > 0) {
                        y = rk4Integrate(y, t, rem, 0.01, derivs);
                        t += rem;
                        if (thrown) {
                            projPos.x += projVel.x * rem;
                            projPos.y += projVel.y * rem;
                        }
                    }
                },
                getObservables: function () {
                    var r = manRadius(t);
                    var Icur = currentI(t);
                    var w = currentOmega(t);
                    var EcPlat = 0.5 * Icur * w * w;
                    var EcProj = thrown ? 0.5 * p.m_proj * p.v_proj * p.v_proj : 0;
                    var Lplat = Icur * w;
                    var Lproj = thrown ? Lthrow : 0;
                    return {
                        t: t,
                        Ec: EcPlat + EcProj,
                        Ep: 0,
                        Etot: EcPlat + EcProj,
                        Px: 0, Py: thrown ? p.m_proj * p.v_proj : 0, P: thrown ? p.m_proj * p.v_proj : 0,
                        Lz: Lplat + Lproj, // Always 0!
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'Radio hombre r(t)', value: fmt(r, 2) + ' m' },
                            { label: '|ω| plataforma', value: fmt(Math.abs(w), 4) + ' rad/s' },
                            { label: 'L_plat + L_masa', value: fmt(Lplat + Lproj, 2) + ' kg·m²/s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t6_p4.solveAnalytical(p).values;
                    var w = Math.abs(currentOmega(t));
                    var inPhaseA = t >= tThrow && t < tWalkStart;
                    var inPhaseB = t >= tWalkEnd;
                    return [
                        { label: 'ω_a (tras lanzar)', unit: 'rad/s', theo: ana.wa, sim: inPhaseA ? w : (t >= tThrow ? simWa : NaN) },
                        { label: 'ω_b (en r_f)', unit: 'rad/s', theo: ana.wb, sim: inPhaseB ? w : NaN },
                        { label: 'L_sistema total', unit: 'kg·m²/s', theo: 0, sim: currentI(t) * currentOmega(t) + (thrown ? Lthrow : 0) }
                    ];
                },
                getRenderState: function () {
                    var r = manRadius(t);
                    var w = currentOmega(t);
                    return {
                        type: '3d-p4',
                        theta: y[0],
                        R: R,
                        manR: r,
                        omega: w,
                        Lplat: currentI(t) * w,
                        thrown: thrown,
                        projPos: projPos,
                        banner: !thrown
                            ? 'Sistema en reposo (L = 0)'
                            : (t < tWalkStart
                                ? 'Masa lanzada (+Y): plataforma retrocede a |ω_a| = ' + fmt(Math.abs(w), 4) + ' rad/s'
                                : (t < tWalkEnd
                                    ? 'Hombre caminando hacia el centro → I baja, |ω| aumenta'
                                    : 'En r_f = ' + fmt(rf, 2) + ' m: |ω_b| = ' + fmt(Math.abs(w), 4) + ' rad/s'))
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 4 — T6 · P11: Fuerza Horizontal a Altura r (Inversión de f_r)
    // =========================================================================
    var t6_p11 = {
        id: 't6_p11',
        topic: 'rigid',
        level: 5,
        code: 'T6 · P11',
        title: 'Fuerza a Altura Variable r sobre Cilindro',
        subtitle: 'Inversión del sentido de f_r en r = R/2 y transición a deslizamiento',
        renderMode: '2d',
        learningGoal: 'Descubrir que la fuerza de rozamiento estática f_r puede apuntar hacia ATRÁS, ser NULA (en r = R/2), o apuntar hacia ADELANTE (r > R/2) para mantener la rodadura sin deslizar.',
        guideFromZero: [
            '<strong>1. Ecuaciones de rodadura pura:</strong> <code>F + f_r = M a</code> y <code>F·r − f_r·R = ½ M R² (a/R)</code>.',
            '<strong>2. Solución general:</strong> <code>a = [2F(1 + r/R)] / (3M)</code> y <code>f_r = F(2r/R − 1) / 3</code>.',
            '<strong>3. ¡El punto mágico r = R/2!:</strong> Cuando tiras exactamente a media altura por encima del centro, <code>f_r = 0</code>. Por debajo (r < R/2) frena; por encima (r > R/2, como r = R) ¡empuja hacia adelante!'
        ],
        statement: 'Una fuerza externa F actúa horizontalmente a una distancia r por encima del CM de un cilindro de radio R y masa M que rueda sin deslizar (coeficiente estático μ_e). Determinar: (a) Aceleración y fuerza de rozamiento. (b) Valores si r = R.',
        paramsSchema: [
            { key: 'M', label: 'Masa cilindro M', unit: 'kg', min: 1, max: 20, step: 0.5, def: 4.0 },
            { key: 'R', label: 'Radio R', unit: 'm', min: 0.2, max: 1.0, step: 0.05, def: 0.5 },
            { key: 'F', label: 'Fuerza aplicada F', unit: 'N', min: 1, max: 50, step: 1, def: 12.0 },
            { key: 'rRatio', label: 'Altura relativa r / R', unit: '', min: -0.8, max: 1.0, step: 0.05, def: 1.0 },
            { key: 'mu_e', label: 'Coef. estático μ_e', unit: '', min: 0.05, max: 0.9, step: 0.05, def: 0.4 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'r = 0 (en CM: f_r atrás)', apply: function (p) { p.rRatio = 0.0; } },
            { label: 'r = R/2 (¡f_r = 0!)', apply: function (p) { p.rRatio = 0.5; } },
            { label: 'r = R (borde superior)', apply: function (p) { p.rRatio = 1.0; } }
        ],
        defaultDuration: 5.0,

        solveAnalytical: function (p) {
            var r = p.rRatio * p.R;
            var aRoll = (2 * p.F * (1 + p.rRatio)) / (3 * p.M);
            var frRoll = (p.F * (2 * p.rRatio - 1)) / 3;
            var frMax = p.mu_e * p.M * p.g;
            var slips = Math.abs(frRoll) > frMax + 1e-9;
            var dirLabel = Math.abs(frRoll) < 1e-6 ? 'NULA (f_r = 0)' : (frRoll > 0 ? 'HACIA ADELANTE (+X)' : 'HACIA ATRÁS (−X)');

            return {
                steps: [
                    {
                        title: '(a) Aceleración del CM rodando sin deslizar',
                        formula: 'a = 2F(1 + r/R) / (3M)',
                        detail: 'Con r/R = ' + fmt(p.rRatio, 2) + ' → a = ' + fmt(aRoll, 4) + ' m/s²'
                    },
                    {
                        title: '(a) Fuerza de rozamiento estática f_r',
                        formula: 'f_r = F(2 r/R − 1) / 3',
                        detail: 'f_r = ' + fmt(frRoll, 4) + ' N → Dirección: ' + dirLabel + '  (límite μ_e Mg = ' + fmt(frMax, 2) + ' N)'
                    },
                    {
                        title: '(b) Caso particular r = R (fuerza en la parte superior)',
                        formula: 'a(r=R) = 4F / (3M)  ;  f_r(r=R) = +F / 3',
                        detail: 'Para tus datos con r=R: a = ' + fmt((4 * p.F) / (3 * p.M), 4) + ' m/s²  ;  f_r = +' + fmt(p.F / 3, 4) + ' N' + (slips ? '  [¡ALERTA: |f_r| > μ_e Mg → DESLIZA!]' : '  [Rueda sin deslizar]')
                    }
                ],
                values: { r: r, aRoll: aRoll, frRoll: frRoll, frMax: frMax, slips: slips }
            };
        },

        createSimulation: function (p) {
            var r = p.rRatio * p.R;
            var I = 0.5 * p.M * p.R * p.R;
            var frReq = (p.F * (2 * p.rRatio - 1)) / 3;
            var frMax = p.mu_e * p.M * p.g;
            var mu_d = 0.85 * p.mu_e;
            // y = [x, vx, phi, omega]
            var y = [0, 0, 0, 0];
            var t = 0;

            function getRegime(st) {
                var vc = st[1] - st[3] * p.R; // contact point slip velocity
                if (Math.abs(vc) < 1e-4 && Math.abs(frReq) <= frMax) {
                    return {
                        slipping: false,
                        fr: frReq,
                        a: (2 * p.F * (1 + p.rRatio)) / (3 * p.M),
                        alpha: (2 * p.F * (1 + p.rRatio)) / (3 * p.M * p.R)
                    };
                }
                var signSlip = Math.abs(vc) > 1e-5 ? -Math.sign(vc) : Math.sign(frReq);
                var frKin = signSlip * mu_d * p.M * p.g;
                return {
                    slipping: true,
                    fr: frKin,
                    a: (p.F + frKin) / p.M,
                    alpha: (p.F * r - frKin * p.R) / I
                };
            }

            function derivs(st) {
                var reg = getRegime(st);
                return [st[1], reg.a, st[3], reg.alpha];
            }

            return {
                step: function (dt) {
                    y = rk4Integrate(y, t, dt, 0.005, derivs);
                    t += dt;
                },
                getObservables: function () {
                    var reg = getRegime(y);
                    var Etras = 0.5 * p.M * y[1] * y[1];
                    var Erot = 0.5 * I * y[3] * y[3];
                    var vc = y[1] - y[3] * p.R;
                    return {
                        t: t, Ec: Etras + Erot, Etras: Etras, Erot: Erot, Ep: 0, Etot: Etras + Erot,
                        Px: p.M * y[1], Py: 0, P: Math.abs(p.M * y[1]), Lz: I * y[3],
                        cm: { x: y[0], y: p.R },
                        finished: false,
                        extra: [
                            { label: 'Régimen', value: reg.slipping ? '¡DESLIZANDO!' : (Math.abs(reg.fr) < 1e-4 ? 'Rodadura (f_r = 0)' : 'Rodadura pura') },
                            { label: 'f_r actual', value: fmt(reg.fr, 3) + ' N' },
                            { label: 'v_contacto (v−ωR)', value: fmt(vc, 3) + ' m/s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t6_p11.solveAnalytical(p).values;
                    var reg = getRegime(y);
                    return [
                        { label: 'a_CM', unit: 'm/s²', theo: ana.slips ? NaN : ana.aRoll, sim: reg.a },
                        { label: 'f_r', unit: 'N', theo: ana.slips ? NaN : ana.frRoll, sim: reg.fr },
                        { label: 'v_CM(t) = a·t', unit: 'm/s', theo: ana.slips ? NaN : ana.aRoll * t, sim: y[1] },
                        { label: 'ω(t)·R', unit: 'm/s', theo: ana.slips ? NaN : ana.aRoll * t, sim: y[3] * p.R }
                    ];
                },
                getRenderState: function () {
                    var reg = getRegime(y);
                    var viewMinX = Math.max(-1.5, y[0] - 2.5);
                    var viewMaxX = Math.max(8, y[0] + 5.5);
                    var fScale = 0.08;
                    var vectors = [
                        { type: 'force', x: y[0], y: p.R + r, vx: p.F * fScale, vy: 0, color: '#fb7185', label: 'F (' + p.F + ' N)' },
                        { type: 'velocity', x: y[0], y: p.R, vx: y[1] * 0.25, vy: 0, color: '#22d3ee', label: 'v_CM' }
                    ];
                    if (Math.abs(reg.fr) > 1e-4) {
                        vectors.push({ type: 'tension', x: y[0], y: 0.02, vx: reg.fr * fScale, vy: 0, color: '#fbbf24', label: 'f_r=' + fmt(reg.fr, 1) + 'N' });
                    }
                    return {
                        bounds: { minX: viewMinX, maxX: viewMaxX, minY: -0.4, maxY: Math.max(2.6, p.R * 3.5) },
                        lines: [
                            { x1: viewMinX, y1: 0, x2: viewMaxX, y2: 0, color: '#475569', width: 4 },
                            { x1: y[0] - p.R * 1.3, y1: p.R + r, x2: y[0] + p.R * 1.3, y2: p.R + r, color: 'rgba(251, 113, 133, 0.35)', width: 1.5, dash: [4, 4] }
                        ],
                        wheels: [
                            { x: y[0], y: p.R, r: p.R, phi: y[2], hollow: false, color: '#38bdf8', label: 'Cilindro (r/R=' + fmt(p.rRatio, 2) + ')' }
                        ],
                        vectors: vectors,
                        banner: reg.slipping
                            ? '¡Superado μ_e Mg! El cilindro desliza (v_CM ≠ ωR)'
                            : (Math.abs(reg.fr) < 1e-4
                                ? '¡PUNTO MÁGICO r = R/2! f_r = 0 N (rodaría sin deslizar incluso sobre hielo)'
                                : (reg.fr > 0 ? 'f_r = +' + fmt(reg.fr, 2) + ' N apunta ADELANTE (+X)' : 'f_r = ' + fmt(reg.fr, 2) + ' N apunta ATRÁS (−X)'))
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 5 — T6 · P10: Choque contra Barra Libre y Centro Instantáneo (CIR)
    // =========================================================================
    var t6_p10 = {
        id: 't6_p10',
        topic: 'rigid',
        level: 6,
        code: 'T6 · P10',
        title: 'Choque contra Barra Libre y Punto Estacionario',
        subtitle: 'Traslación + rotación simultánea y Centro Instantáneo de Rotación (CIR)',
        renderMode: '2d',
        learningGoal: 'Combinar conservación de P y L_CM en una colisión contra un sólido rígido libre y localizar el punto de velocidad nula (v_CM + ω × r = 0).',
        guideFromZero: [
            '<strong>1. Nuevo Centro de Masas:</strong> La masa m se pega al extremo +L/2 de la barra (masa 2m). El CM queda a <code>x_CM = L/6</code> del centro de la barra.',
            '<strong>2. Traslación y Rotación:</strong> Conservando P: <code>V_CM = V / 3</code>. Conservando L_CM: <code>I_CM = ⅓ m L²</code> y <code>L_CM = ⅓ m V L</code>, luego <code>ω = V / L</code>.',
            '<strong>3. Punto estacionario (CIR):</strong> La velocidad a distancia x del centro es <code>v(x) = V_CM + ω(x − L/6) = 0</code> → ¡en <code>x = −L/6</code> (a L/3 del extremo libre)!'
        ],
        statement: 'Una barra horizontal homogénea de masa 2m y longitud L reposa sobre una mesa sin rozamiento. Una masa puntual m con velocidad V choca perpendicularmente con un extremo y queda unida. Determinar: (a) ω tras el choque. (b) Variación de E_c. (c) Posición del punto de la barra estacionario justo después del choque.',
        paramsSchema: [
            { key: 'm', label: 'Masa puntual m (barra = 2m)', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 1.0 },
            { key: 'L', label: 'Longitud de la barra L', unit: 'm', min: 1.0, max: 5.0, step: 0.2, def: 2.0 },
            { key: 'V', label: 'Velocidad inicial V de m', unit: 'm/s', min: 0.5, max: 8.0, step: 0.5, def: 3.0 }
        ],
        defaultDuration: 5.0,

        solveAnalytical: function (p) {
            var Vcm = p.V / 3;
            var w = p.V / p.L;
            var Eci = 0.5 * p.m * p.V * p.V;
            var Ecf = (1 / 3) * p.m * p.V * p.V;
            var dEc = Ecf - Eci; // -1/6 m V^2
            var xCirFromCenter = -p.L / 6;
            var distFromFree = p.L / 3;

            return {
                steps: [
                    {
                        title: '1. CM e Inercia del conjunto (3m)',
                        formula: 'x_CM = L/6 del centro (a L/3 del extremo impactado) ; I_CM = ⅓ m L²',
                        detail: 'V_CM = mV / (3m) = V/3 = ' + fmt(Vcm, 4) + ' m/s'
                    },
                    {
                        title: '(a) Velocidad angular tras el choque',
                        formula: 'L_CM = (L/3)·mV = I_CM·ω  →  ω = V / L',
                        detail: 'ω = ' + fmt(w, 4) + ' rad/s  (sentido antihorario)'
                    },
                    {
                        title: '(b) Variación de energía cinética',
                        formula: 'ΔE_c = ½(3m)V_CM² + ½I_CM ω² − ½mV² = −(1/6) m V²',
                        detail: 'E_c,i = ' + fmt(Eci, 3) + ' J → E_c,f = ' + fmt(Ecf, 3) + ' J  (pérdida = ' + fmt(-dEc, 3) + ' J, 33.3%)'
                    },
                    {
                        title: '(c) Punto estacionario justo tras el choque (CIR)',
                        formula: 'v(x) = V/3 + (V/L)(x − L/6) = 0  →  x = −L/6',
                        detail: 'A ' + fmt(Math.abs(xCirFromCenter), 4) + ' m del centro hacia el extremo libre (es decir, a L/3 = ' + fmt(distFromFree, 4) + ' m del extremo libre)'
                    }
                ],
                values: { Vcm: Vcm, w: w, Eci: Eci, Ecf: Ecf, loss: -dEc, xCir: xCirFromCenter, distFromFree: distFromFree }
            };
        },

        createSimulation: function (p) {
            var tColl = 1.0;
            var t = 0;
            var collided = false;
            var halfL = 0.5 * p.L;
            var xCmLocal = p.L / 6; // relative to rod geometric center
            var Icm = (1 / 3) * p.m * p.L * p.L;

            // State after collision: [Xcm, Ycm, Vxcm, Vycm, theta, omega]
            var y = [xCmLocal, 0, 0, p.V / 3, 0, p.V / p.L];
            var massY = -p.V * tColl;
            var trails = { cm: [{ x: xCmLocal, y: massY / 3 }], cir: [] };

            function derivs(st) {
                return [st[2], st[3], 0, 0, st[5], 0];
            }

            return {
                step: function (dt) {
                    var rem = dt;
                    if (!collided && t + rem >= tColl) {
                        var dtPre = tColl - t;
                        massY += p.V * dtPre;
                        t = tColl;
                        rem -= dtPre;
                        collided = true;
                        massY = 0;
                        y = [xCmLocal, 0, 0, p.V / 3, 0, p.V / p.L];
                    }
                    if (!collided) {
                        massY += p.V * rem;
                        t += rem;
                        trails.cm.push({ x: xCmLocal, y: (p.m * massY) / (3 * p.m) });
                        return;
                    }
                    if (rem > 0) {
                        y = rk4Integrate(y, t, rem, 0.005, derivs);
                        t += rem;
                        if (trails.cm.length < 800) trails.cm.push({ x: y[0], y: y[1] });
                        // CIR point at rod local coordinate x = -L/6 (distance -L/3 from CM)
                        var cirX = y[0] - (p.L / 3) * Math.cos(y[4]);
                        var cirY = y[1] - (p.L / 3) * Math.sin(y[4]);
                        if (trails.cir.length < 800) trails.cir.push({ x: cirX, y: cirY });
                    }
                },
                getObservables: function () {
                    var Ec = collided
                        ? 0.5 * (3 * p.m) * (y[2] * y[2] + y[3] * y[3]) + 0.5 * Icm * y[5] * y[5]
                        : 0.5 * p.m * p.V * p.V;
                    var Py = collided ? 3 * p.m * y[3] : p.m * p.V;
                    var Lcm = collided ? Icm * y[5] : (p.L / 3) * p.m * p.V;
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: 0, Py: Py, P: Py, Lz: Lcm,
                        cm: { x: collided ? y[0] : xCmLocal, y: collided ? y[1] : massY / 3 },
                        finished: false,
                        extra: [
                            { label: 'V_CM', value: fmt(collided ? y[3] : p.V / 3, 3) + ' m/s' + (collided ? '' : ' (ya es V/3)') },
                            { label: 'ω barra', value: collided ? fmt(y[5], 3) + ' rad/s' : '0' },
                            { label: 'L_CM (constante)', value: fmt(Lcm, 3) + ' kg·m²/s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t6_p10.solveAnalytical(p).values;
                    return [
                        { label: 'V_CM post', unit: 'm/s', theo: ana.Vcm, sim: collided ? y[3] : NaN },
                        { label: 'ω post', unit: 'rad/s', theo: ana.w, sim: collided ? y[5] : NaN },
                        { label: 'E_c post', unit: 'J', theo: ana.Ecf, sim: collided ? (0.5 * 3 * p.m * y[3] * y[3] + 0.5 * Icm * y[5] * y[5]) : NaN }
                    ];
                },
                getRenderState: function () {
                    var th = collided ? y[4] : 0;
                    var cx = collided ? y[0] : xCmLocal;
                    var cy = collided ? y[1] : 0;
                    var ux = Math.cos(th), uy = Math.sin(th);
                    // Rod geometric center is at -L/6 from CM along u
                    var gx = cx - xCmLocal * ux, gy = cy - xCmLocal * uy;
                    var xFree = gx - halfL * ux, yFree = gy - halfL * uy;
                    var xHit = gx + halfL * ux, yHit = gy + halfL * uy;
                    var cirX = gx - (p.L / 6) * ux, cirY = gy - (p.L / 6) * uy;

                    var vectors = [];
                    var masses = [];
                    if (!collided) {
                        masses.push({ x: halfL, y: massY, r: 0.08 * p.L, color: '#fb7185', label: 'm (V=' + p.V + ' m/s)' });
                        vectors.push({ type: 'velocity', x: halfL, y: massY, vx: 0, vy: p.V * 0.25, color: '#22d3ee', label: 'V' });
                    } else {
                        masses.push({ x: xHit, y: yHit, r: 0.08 * p.L, color: '#fb7185', label: 'm' });
                        masses.push({ x: cirX, y: cirY, r: 0.045 * p.L, color: '#22c55e', label: 'CIR (−L/6)' });
                        // Velocity profile vectors along the rod (free end, CIR=0, CM, struck end)
                        var pts = [-halfL, -p.L / 6, xCmLocal, halfL];
                        for (var i = 0; i < pts.length; i++) {
                            var s = pts[i] - xCmLocal; // from CM
                            var px = cx + s * ux, py = cy + s * uy;
                            var vx = y[2] - y[5] * s * uy;
                            var vy = y[3] + y[5] * s * ux;
                            if (Math.hypot(vx, vy) > 0.05) {
                                vectors.push({ type: 'velocity', x: px, y: py, vx: vx * 0.28, vy: vy * 0.28, color: '#22d3ee', label: i === 3 ? '2V/3' : (i === 0 ? '−V/3' : 'V_CM') });
                            }
                        }
                    }

                    var spanX = p.L * 1.1;
                    var minY = Math.min(-p.V * tColl - 0.6, cy - p.L);
                    var maxY = Math.max(3.0, cy + p.L * 1.2);
                    return {
                        bounds: { minX: -spanX, maxX: spanX, minY: minY, maxY: maxY },
                        rods: [{ x1: xFree, y1: yFree, x2: xHit, y2: yHit, color: '#cbd5e1', width: 6 }],
                        trails: [
                            { pts: trails.cm, color: '#fbbf24', width: 2.5 },
                            { pts: trails.cir, color: 'rgba(34, 197, 94, 0.5)', width: 1.5, dash: [4, 4] }
                        ],
                        masses: masses,
                        cm: { x: cx, y: collided ? cy : (p.m * massY) / (3 * p.m), label: 'CM' },
                        vectors: vectors,
                        banner: collided
                            ? 'Tras choque: CM sube recto a V/3; el punto verde (x = −L/6) arrancó con velocidad CERO'
                            : 'Masa m acercándose al extremo +L/2 de la barra libre (masa 2m)'
                    };
                }
            };
        }
    };

    // =========================================================================
    // NIVEL 6 — T6 · P1: Varilla Inclinada en 3D (L no paralelo a ω)
    // =========================================================================
    var t6_p1 = {
        id: 't6_p1',
        topic: 'rigid',
        level: 7,
        code: 'T6 · P1',
        title: 'Varilla Inclinada en 3D: L no paralelo a ω',
        subtitle: 'Tensor de inercia, ejes principales y desequilibrio dinámico en 3D',
        renderMode: '3d-p1',
        learningGoal: 'Comprender en 3D que L = I·ω NO siempre es paralelo a ω: cuando el eje de giro no es eje principal de inercia, L describe un cono y los cojinetes sufren un par dinámico τ = ω × L.',
        guideFromZero: [
            '<strong>1. Momento de inercia respecto al eje:</strong> Cada elemento a distancia s del centro O dista <code>r_⊥ = |s| sin α</code> del eje → <code>I_eje = ⅓ m a² sin²α</code>.',
            '<strong>2. Vector Momento Angular L_O:</strong> Es SIEMPRE perpendicular a la varilla: <code>L_O = ⅓ m a² ω [k − cos α · û(t)]</code>. Su módulo es <code>⅓ m a² ω sin α</code>.',
            '<strong>3. ¿Es eje principal de inercia?:</strong> Solo cuando L_O ∥ ω, es decir en <code>α = 0°</code> (L=0) o <code>α = 90°</code>. Para 0° < α < 90°, L_O gira formando un cono y requiere par <code>τ = dL_O/dt = ω × L_O</code>.'
        ],
        statement: 'Un eje rígido sin masa gira con velocidad angular ω constante. Una varilla uniforme de longitud 2a y masa m está unida rígidamente por su centro O formando un ángulo α constante con el eje. (a) Momento de inercia respecto al eje. (b) Momento angular L_O. ¿Es el eje de giro un eje principal de inercia? ¿Por qué?',
        paramsSchema: [
            { key: 'm', label: 'Masa de la varilla m', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 2.0 },
            { key: 'a', label: 'Semilongitud a (total 2a)', unit: 'm', min: 0.5, max: 3.0, step: 0.1, def: 1.0 },
            { key: 'alphaDeg', label: 'Ángulo inclinación α', unit: '°', min: 0, max: 90, step: 1, def: 35 },
            { key: 'omega', label: 'Velocidad angular ω', unit: 'rad/s', min: 0.5, max: 8.0, step: 0.5, def: 3.0 }
        ],
        quickActions: [
            { label: 'α = 35° (NO principal)', apply: function (p) { p.alphaDeg = 35; } },
            { label: 'α = 90° (SÍ principal: L ∥ ω)', apply: function (p) { p.alphaDeg = 90; } }
        ],
        defaultDuration: 8.0,

        solveAnalytical: function (p) {
            var al = degToRad(p.alphaDeg);
            var sinA = Math.sin(al), cosA = Math.cos(al);
            var I_rod = (1 / 3) * p.m * p.a * p.a;
            var Ieje = I_rod * sinA * sinA;
            var Lmag = I_rod * p.omega * sinA;
            var Lz = Ieje * p.omega;
            var Lperp = I_rod * p.omega * sinA * cosA;
            var tauMag = p.omega * Lperp;
            var Ec = 0.5 * Ieje * p.omega * p.omega;
            var isPrincipal = (p.alphaDeg <= 0.1 || p.alphaDeg >= 89.9);

            return {
                steps: [
                    {
                        title: '(a) Momento de inercia respecto al eje vertical',
                        formula: 'I_eje = ∫₋ₐᵃ (s sin α)² (m/2a) ds = ⅓ m a² sin²α',
                        detail: 'I_eje = ' + fmt(Ieje, 4) + ' kg·m²  (máximo ⅓ma² = ' + fmt(I_rod, 3) + ' en α = 90°)'
                    },
                    {
                        title: '(b) Momento angular L_O respecto al centro O',
                        formula: 'L_O = ⅓ m a² ω [k − cos α · û(t)]  (|L_O| = ⅓ m a² ω sin α)',
                        detail: '|L_O| = ' + fmt(Lmag, 4) + ' kg·m²/s  |  L_z = ' + fmt(Lz, 4) + '  |  L_⊥(rotante) = ' + fmt(Lperp, 4) + ' kg·m²/s'
                    },
                    {
                        title: '¿Es eje principal de inercia? Par dinámico necesario',
                        formula: 'τ = dL_O/dt = ω × L_O  (|τ| = ⅓ m a² ω² sin α cos α)',
                        detail: isPrincipal
                            ? 'SÍ es eje principal en α = ' + p.alphaDeg + '° (L_⊥ = 0, τ = 0: giro equilibrado)'
                            : 'NO es eje principal: L_O forma ' + fmt(90 - p.alphaDeg, 1) + '° con ω y precesa. Los cojinetes deben ejercer τ = ' + fmt(tauMag, 4) + ' N·m'
                    }
                ],
                values: { Ieje: Ieje, Lmag: Lmag, Lz: Lz, Lperp: Lperp, tauMag: tauMag, Ec: Ec, isPrincipal: isPrincipal }
            };
        },

        createSimulation: function (p) {
            var al = degToRad(p.alphaDeg);
            var sinA = Math.sin(al), cosA = Math.cos(al);
            var I_rod = (1 / 3) * p.m * p.a * p.a;
            var Ieje = I_rod * sinA * sinA;
            var y = [0]; // [phi]
            var t = 0;

            function derivs() { return [p.omega]; }

            function computeVectors(phi) {
                var ux = sinA * Math.cos(phi), uy = sinA * Math.sin(phi), uz = cosA;
                var coef = I_rod * p.omega;
                // L_O = coef * (k - cosA * u)
                var Lx = -coef * cosA * ux;
                var Ly = -coef * cosA * uy;
                var Lz = coef * (1 - cosA * uz); // = coef * sinA^2
                // tau = omega k x L_O = (-omega*Ly, omega*Lx, 0)
                var tx = -p.omega * Ly;
                var ty = p.omega * Lx;
                return { u: [ux, uy, uz], L: [Lx, Ly, Lz], tau: [tx, ty, 0] };
            }

            return {
                step: function (dt) {
                    y = rk4Integrate(y, t, dt, 0.005, derivs);
                    t += dt;
                },
                getObservables: function () {
                    var vecs = computeVectors(y[0]);
                    var Lmag = Math.hypot(vecs.L[0], vecs.L[1], vecs.L[2]);
                    var tauMag = Math.hypot(vecs.tau[0], vecs.tau[1]);
                    var Ec = 0.5 * Ieje * p.omega * p.omega;
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: 0, Py: 0, P: 0, Lz: vecs.L[2],
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: '|L_O| / L_z', value: fmt(Lmag, 3) + ' / ' + fmt(vecs.L[2], 3) + ' kg·m²/s' },
                            { label: 'Par cojinetes |τ|', value: fmt(tauMag, 3) + ' N·m' },
                            { label: 'Ángulo(L_O, ω)', value: fmt(90 - p.alphaDeg, 1) + '°' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var ana = t6_p1.solveAnalytical(p).values;
                    var vecs = computeVectors(y[0]);
                    var Lmag = Math.hypot(vecs.L[0], vecs.L[1], vecs.L[2]);
                    var tauMag = Math.hypot(vecs.tau[0], vecs.tau[1]);
                    return [
                        { label: 'I_eje', unit: 'kg·m²', theo: ana.Ieje, sim: Ieje },
                        { label: '|L_O|', unit: 'kg·m²/s', theo: ana.Lmag, sim: Lmag },
                        { label: 'L_z', unit: 'kg·m²/s', theo: ana.Lz, sim: vecs.L[2] },
                        { label: 'Par dinámico |τ|', unit: 'N·m', theo: ana.tauMag, sim: tauMag }
                    ];
                },
                getRenderState: function () {
                    var vecs = computeVectors(y[0]);
                    return {
                        type: '3d-p1',
                        phi: y[0],
                        a: p.a,
                        alphaDeg: p.alphaDeg,
                        omega: p.omega,
                        u: vecs.u,
                        L: vecs.L,
                        tau: vecs.tau,
                        banner: (p.alphaDeg > 0.5 && p.alphaDeg < 89.5)
                            ? 'L_O (oro) NO es paralelo a ω (cian): describe un cono y exige par τ (magenta) en los cojinetes'
                            : 'Eje principal de inercia (α = ' + p.alphaDeg + '°): desequilibrio dinámico τ = 0'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P5: Hombre Caminando sobre Tronco Flotante
    // =========================================================================
    var t6_p5 = {
        id: 't6_p5',
        topic: 'rigid',
        level: 3,
        code: 'T6 · P5',
        title: 'Hombre Caminando sobre Tronco Flotante',
        subtitle: 'Conservación del CM en ausencia de fuerzas horizontales externas',
        renderMode: '2d',
        learningGoal: 'Comprender que al caminar hacia la orilla sin rozamiento con el agua, el tronco retrocede de modo que el CM del sistema hombre + tronco permanece inmóvil.',
        guideFromZero: [
            '<strong>1. Posición inicial:</strong> El tronco de L = 20 m toca la orilla en <code>x = 0</code> (CM tronco en 10 m) y el hombre está en <code>x = 20 m</code> → <code>x_CM = (800·10 + 80·20)/880 = 10.909 m</code>.',
            '<strong>2. En el punto medio del tronco:</strong> El hombre coincide con el CM del tronco, luego ambos están en <code>x_CM = 10.909 m</code> (el tronco se alejó 0.909 m de la orilla).',
            '<strong>3. Al llegar al otro extremo del tronco:</strong> El hombre está a distancia <code>x_ext</code> de la orilla y el tronco en <code>x_ext + 10</code> → <code>x_ext = 1.818 m</code>.'
        ],
        statement: 'Un hombre de 80 kg está en el extremo exterior de un tronco de 20 m y 800 kg que flota perpendicularmente tocando la orilla. Camina hasta el otro extremo sin rozamiento del agua. Indicar la posición del hombre, del tronco y del CM: (a) al inicio, (b) en el punto medio, (c) en el otro extremo (¿distancia a la orilla?).',
        paramsSchema: [
            { key: 'mMan', label: 'Masa hombre m', unit: 'kg', min: 40, max: 140, step: 5, def: 80 },
            { key: 'Mlog', label: 'Masa tronco M', unit: 'kg', min: 200, max: 1600, step: 50, def: 800 },
            { key: 'L', label: 'Longitud tronco L', unit: 'm', min: 8, max: 30, step: 1, def: 20 },
            { key: 'walkTime', label: 'Duración caminata', unit: 's', min: 2, max: 10, step: 0.5, def: 5 }
        ],
        defaultDuration: 5.5,

        solveAnalytical: function (p) {
            var Mtot = p.mMan + p.Mlog;
            var xcm = (p.Mlog * 0.5 * p.L + p.mMan * p.L) / Mtot;
            var xMidMan = xcm;
            var xEndMan = xcm - (p.Mlog * 0.5 * p.L) / Mtot;
            var xEndLogCenter = xEndMan + 0.5 * p.L;
            return {
                steps: [
                    {
                        title: '(a) Posición inicial del CM (inmutable)',
                        formula: 'x_CM = (M·L/2 + m·L) / (M + m)',
                        detail: 'x_CM = ' + fmt(xcm, 3) + ' m desde la orilla'
                    },
                    {
                        title: '(b) Cuando el hombre está en el punto medio del tronco',
                        formula: 'x_hombre = x_centro_tronco = x_CM',
                        detail: 'Ambos en x = ' + fmt(xMidMan, 3) + ' m (la punta del tronco se separó ' + fmt(xMidMan - 0.5 * p.L, 3) + ' m de la orilla)'
                    },
                    {
                        title: '(c) Cuando el hombre llega al extremo cercano',
                        formula: 'm·x_h + M·(x_h + L/2) = (m + M)·x_CM',
                        detail: 'Distancia del hombre a la orilla: x_h = ' + fmt(xEndMan, 3) + ' m  (centro del tronco en ' + fmt(xEndLogCenter, 3) + ' m)'
                    }
                ],
                values: { Mtot: Mtot, xcm: xcm, xMidMan: xMidMan, xEndMan: xEndMan, xEndLogCenter: xEndLogCenter }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p5.solveAnalytical(p).values;
            var y = [0]; // [progress u in 0..1]
            var t = 0;

            function derivs() { return [t < p.walkTime ? 1 / p.walkTime : 0]; }

            function stateAtU(u) {
                // Relative coordinate s on log from near-end: L -> 0
                var sRel = p.L * (1 - u);
                // M*(xNear + L/2) + m*(xNear + sRel) = Mtot * xcm -> xNear = xcm - (M*L/2 + m*sRel)/Mtot
                var xNear = ana.xcm - (p.Mlog * 0.5 * p.L + p.mMan * sRel) / ana.Mtot;
                return { xNear: xNear, xLogCenter: xNear + 0.5 * p.L, xMan: xNear + sRel };
            }

            return {
                step: function (dt) {
                    if (t >= p.walkTime) return;
                    y = rk4Integrate(y, t, Math.min(dt, p.walkTime - t), 0.01, derivs);
                    t = Math.min(p.walkTime, t + dt);
                    if (y[0] > 1) y[0] = 1;
                },
                getObservables: function () {
                    var st = stateAtU(y[0]);
                    var vRel = t < p.walkTime ? -p.L / p.walkTime : 0;
                    var vLog = -(p.mMan / ana.Mtot) * vRel;
                    var vMan = vLog + vRel;
                    var Ec = 0.5 * p.mMan * vMan * vMan + 0.5 * p.Mlog * vLog * vLog;
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: p.mMan * vMan + p.Mlog * vLog, Py: 0, P: Math.abs(p.mMan * vMan + p.Mlog * vLog), Lz: 0,
                        cm: { x: ana.xcm, y: 0 },
                        finished: t >= p.walkTime,
                        extra: [
                            { label: 'x_hombre (orilla)', value: fmt(st.xMan, 3) + ' m' },
                            { label: 'Punta tronco', value: fmt(st.xNear, 3) + ' m' },
                            { label: 'x_CM (inmóvil)', value: fmt(ana.xcm, 3) + ' m' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var st = stateAtU(y[0]);
                    var done = t >= p.walkTime - 1e-6;
                    var curCm = (p.mMan * st.xMan + p.Mlog * st.xLogCenter) / ana.Mtot;
                    return [
                        { label: 'x_CM invariante', unit: 'm', theo: ana.xcm, sim: curCm },
                        { label: 'Distancia final orilla', unit: 'm', theo: ana.xEndMan, sim: done ? st.xMan : NaN }
                    ];
                },
                getRenderState: function () {
                    var st = stateAtU(y[0]);
                    return {
                        bounds: { minX: -2.5, maxX: p.L + 3.5, minY: -3.5, maxY: 4.5 },
                        polys: [
                            // Shore on left, water on right
                            { pts: [{ x: -2.5, y: -3.5 }, { x: 0, y: -3.5 }, { x: 0, y: 4.5 }, { x: -2.5, y: 4.5 }], fill: 'rgba(120, 113, 108, 0.35)', stroke: '#78716c', lineWidth: 2 },
                            { pts: [{ x: 0, y: -3.5 }, { x: p.L + 3.5, y: -3.5 }, { x: p.L + 3.5, y: 0 }, { x: 0, y: 0 }], fill: 'rgba(14, 165, 233, 0.16)' }
                        ],
                        boxes: [
                            { x: st.xLogCenter, y: 0.15, w: p.L, h: 0.7, color: '#92400e', stroke: '#d97706', label: 'Tronco (' + p.Mlog + ' kg)' }
                        ],
                        masses: [
                            { x: st.xMan, y: 0.95, r: 0.45, color: '#38bdf8', label: 'Hombre (' + p.mMan + ' kg)' }
                        ],
                        lines: [
                            { x1: ana.xcm, y1: -1.5, x2: ana.xcm, y2: 2.5, color: 'rgba(251,191,36,0.4)', width: 1.5, dash: [4, 4] }
                        ],
                        cm: { x: ana.xcm, y: 0.15, label: 'CM fijo (' + fmt(ana.xcm, 2) + ' m)' },
                        labels: [
                            { x: -1.2, y: 2.8, text: 'Orilla (x=0)', color: '#cbd5e1', align: 'center' }
                        ],
                        banner: t >= p.walkTime
                            ? 'Hombre en el extremo: queda a ' + fmt(st.xMan, 3) + ' m de la orilla (tronco retrocedió)'
                            : 'Caminando hacia la orilla: el tronco retrocede manteniendo el CM quieto en ' + fmt(ana.xcm, 2) + ' m'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P6: Rueda ante un Escalón (Equilibrio de Pares y Vuelco)
    // =========================================================================
    var t6_p6 = {
        id: 't6_p6',
        topic: 'rigid',
        level: 4,
        code: 'T6 · P6',
        title: 'Rueda ante un Escalón (Equilibrio y Vuelco)',
        subtitle: 'Momentos respecto al borde B y fuerza mínima horizontal F_mín',
        renderMode: '2d',
        learningGoal: 'Elegir el punto de pivote adecuado (el borde B del escalón, donde actúan las reacciones desconocidas) para calcular la fuerza mínima que inicia el ascenso.',
        guideFromZero: [
            '<strong>1. En el instante de despegar del suelo (N_A = 0):</strong> Solo actúan el peso <code>mg</code>, la fuerza <code>F</code> en el centro O y la reacción en la esquina B.',
            '<strong>2. Brazos de palanca respecto a B:</strong> Brazo de F: <code>y = r − h</code>. Brazo del peso: <code>x = √[r² − (r − h)²]</code>.',
            '<strong>3. Equilibrio de momentos:</strong> <code>F(r − h) = mg √(2rh − h²)</code>. Si aplicas <code>F > F_mín</code>, la rueda pivota sobre B y sube el escalón.'
        ],
        statement: 'Una rueda de masa m = 1 kg y radio r = 30 cm está apoyada contra un escalón de altura h = 10 cm. Calcular la fuerza horizontal F aplicada en su centro necesaria para mantenerla en equilibrio al borde de subir (N_A = 0).',
        paramsSchema: [
            { key: 'm', label: 'Masa de la rueda m', unit: 'kg', min: 0.5, max: 10, step: 0.5, def: 1.0 },
            { key: 'r_cm', label: 'Radio r', unit: 'cm', min: 15, max: 60, step: 1, def: 30 },
            { key: 'h_cm', label: 'Altura escalón h', unit: 'cm', min: 2, max: 25, step: 1, def: 10 },
            { key: 'F_ratio', label: 'Fuerza aplicada F / F_mín', unit: '', min: 0.5, max: 1.5, step: 0.05, def: 1.0 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'Equilibrio crítico (F = F_mín)', apply: function (p) { p.F_ratio = 1.0; } },
            { label: 'Supera y sube (F = 1.25 F_mín)', apply: function (p) { p.F_ratio = 1.25; } }
        ],
        defaultDuration: 3.5,

        solveAnalytical: function (p) {
            var r = p.r_cm / 100, h = Math.min(r * 0.9, p.h_cm / 100);
            var dy = r - h;
            var dx = Math.sqrt(Math.max(1e-6, r * r - dy * dy));
            var Fmin = (p.m * p.g * dx) / dy;
            var F = p.F_ratio * Fmin;
            return {
                steps: [
                    {
                        title: '1. Geometría respecto a la esquina B',
                        formula: 'd_vertical = r − h  ;  d_horiz = √[r² − (r − h)²]',
                        detail: 'd_vert = ' + fmt(dy, 3) + ' m  ;  d_horiz = ' + fmt(dx, 3) + ' m'
                    },
                    {
                        title: '2. Fuerza mínima de equilibrio (N_A = 0)',
                        formula: 'F_mín · (r − h) = m g · d_horiz',
                        detail: 'F_mín = ' + fmt(Fmin, 3) + ' N  (aplicada actualmente: F = ' + fmt(F, 3) + ' N)'
                    }
                ],
                values: { r: r, h: h, dy: dy, dx: dx, Fmin: Fmin, F: F }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p6.solveAnalytical(p).values;
            var phi0 = Math.atan2(ana.dx, ana.dy); // angle of BO from vertical
            var I_B = 1.5 * p.m * ana.r * ana.r; // solid disc about rim B
            var y = [phi0, 0]; // [psi from vertical (0 = on top of corner B), omega]
            var t = 0;
            var climbed = false;

            function derivs(st) {
                var psi = st[0];
                // Torque about B (CCW = decreasing psi): F*r*cos(psi) - mg*r*sin(psi)
                if (psi <= 0) return [0, 0];
                var tauClimb = ana.F * ana.r * Math.cos(psi) - p.m * p.g * ana.r * Math.sin(psi);
                if (psi >= phi0 && tauClimb <= 0) return [0, 0]; // supported by floor A
                return [st[1], -tauClimb / I_B];
            }

            return {
                step: function (dt) {
                    if (climbed) return;
                    y = rk4Integrate(y, t, dt, 0.004, derivs);
                    t += dt;
                    if (y[0] >= phi0) { y[0] = phi0; y[1] = 0; }
                    if (y[0] <= 0) { y[0] = 0; y[1] = 0; climbed = true; }
                },
                getObservables: function () {
                    var cx = -ana.r * Math.sin(y[0]), cy = ana.h + ana.r * Math.cos(y[0]);
                    var Ec = 0.5 * I_B * y[1] * y[1];
                    var Ep = p.m * p.g * (cy - ana.r);
                    return {
                        t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                        Px: 0, Py: 0, P: Math.abs(p.m * ana.r * y[1]), Lz: I_B * y[1],
                        cm: { x: cx, y: cy },
                        finished: climbed,
                        extra: [
                            { label: 'F aplicada / F_mín', value: fmt(ana.F, 2) + ' / ' + fmt(ana.Fmin, 2) + ' N' },
                            { label: 'Ángulo BO', value: fmt(radToDeg(y[0]), 1) + '°' },
                            { label: 'Estado', value: climbed ? '¡Subió el escalón!' : (p.F_ratio > 1.001 ? 'Pivotando sobre B' : (p.F_ratio < 0.999 ? 'Apoyada en A' : 'Equilibrio crítico')) }
                        ]
                    };
                },
                compareWithSim: function () {
                    return [
                        { label: 'F_mín equilibrio', unit: 'N', theo: ana.Fmin, sim: ana.F },
                        { label: 'Altura centro O', unit: 'm', theo: ana.r, sim: ana.h + ana.r * Math.cos(y[0]) }
                    ];
                },
                getRenderState: function () {
                    var cx = -ana.r * Math.sin(y[0]), cy = ana.h + ana.r * Math.cos(y[0]);
                    var fScale = 0.02;
                    return {
                        bounds: { minX: -ana.r * 2.5, maxX: ana.r * 2.5, minY: -0.15, maxY: ana.h + ana.r * 2.5 },
                        polys: [
                            { pts: [{ x: 0, y: 0 }, { x: ana.r * 2.5, y: 0 }, { x: ana.r * 2.5, y: ana.h }, { x: 0, y: ana.h }], fill: 'rgba(71, 85, 105, 0.5)', stroke: '#94a3b8', lineWidth: 2 }
                        ],
                        lines: [
                            { x1: -ana.r * 2.5, y1: 0, x2: 0, y2: 0, color: '#475569', width: 3 },
                            { x1: 0, y1: ana.h, x2: cx, y2: cy, color: 'rgba(251,191,36,0.55)', width: 2, dash: [4, 4] }
                        ],
                        wheels: [
                            { x: cx, y: cy, r: ana.r, phi: phi0 - y[0], hollow: false, color: '#38bdf8', label: 'Rueda (m=' + p.m + 'kg)' }
                        ],
                        labels: [
                            { x: 0.04, y: ana.h * 0.45, text: 'B (h=' + p.h_cm + 'cm)', color: '#fbbf24' },
                            { x: cx, y: 0.03, text: 'A', color: '#94a3b8', align: 'center' }
                        ],
                        vectors: [
                            { type: 'force', x: cx, y: cy, vx: ana.F * fScale, vy: 0, color: '#4ade80', label: 'F=' + fmt(ana.F, 1) + 'N' },
                            { type: 'force', x: cx, y: cy, vx: 0, vy: -p.m * p.g * fScale, color: '#fb7185', label: 'mg' }
                        ],
                        banner: climbed
                            ? '¡Escalón superado! La rueda pivotó sobre B'
                            : (p.F_ratio > 1.001 ? 'F > F_mín: momento de F vence a mg → subiendo' : 'F_mín equilibrio = ' + fmt(ana.Fmin, 3) + ' N')
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P14: Escalera Apoyada con Rozamiento en Pared y Suelo
    // =========================================================================
    var t6_p14 = {
        id: 't6_p14',
        topic: 'rigid',
        level: 5,
        code: 'T6 · P14',
        title: 'Escalera Apoyada con Rozamiento Límite',
        subtitle: 'Equilibrio estático de fuerzas y pares vs deslizamiento por pared y suelo',
        renderMode: '2d',
        learningGoal: 'Comprobar que el ángulo crítico de una escalera depende SOLO del coeficiente de rozamiento (tan θ = 2μ/(1−μ²)), siendo independiente de su masa m y longitud L.',
        guideFromZero: [
            '<strong>1. Equilibrio de fuerzas:</strong> <code>N_suelo + f_pared = mg</code> y <code>N_pared = f_suelo</code>.',
            '<strong>2. Equilibrio límite (ambos rozamientos máximos μ):</strong> <code>tan θ = 2μ / (1 − μ²)</code> → es la fórmula de ángulo doble <code>θ = 2 atan(μ)</code>.',
            '<strong>3. Para θ = 45°:</strong> <code>μ² + 2μ − 1 = 0 → μ = √2 − 1 ≈ 0.4142</code>, ¡exactamente igual para 12 kg / 2 m que para 50 kg / 1 m!'
        ],
        statement: 'Una escalera de m = 12 kg y L = 2 m está apoyada formando θ = 45° con la vertical, con igual coeficiente de rozamiento μ en pared y suelo. Si se aumenta θ mínimamente cae. (a) Valor de μ. (b) ¿Cambia si m = 50 kg y L = 1 m?',
        paramsSchema: [
            { key: 'm', label: 'Masa escalera m', unit: 'kg', min: 5, max: 60, step: 1, def: 12 },
            { key: 'L', label: 'Longitud L', unit: 'm', min: 1.0, max: 4.0, step: 0.2, def: 2.0 },
            { key: 'theta0Deg', label: 'Ángulo con vertical θ', unit: '°', min: 15, max: 70, step: 1, def: 45 },
            { key: 'mu', label: 'Coef. rozamiento μ', unit: '', min: 0.15, max: 0.80, step: 0.01, def: 0.42 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'Límite 45° (μ=0.42 aguanta)', apply: function (p) { p.theta0Deg = 45; p.mu = 0.42; } },
            { label: 'Cae por superar θ_crít (μ=0.35)', apply: function (p) { p.theta0Deg = 45; p.mu = 0.35; } },
            { label: 'Caso (b): 50 kg, 1 m', apply: function (p) { p.m = 50; p.L = 1.0; p.theta0Deg = 45; p.mu = 0.42; } }
        ],
        defaultDuration: 3.0,

        solveAnalytical: function (p) {
            var th = degToRad(p.theta0Deg);
            var muCrit = (Math.sqrt(1 + Math.tan(th) * Math.tan(th)) - 1) / Math.max(1e-6, Math.tan(th));
            var thetaCritDeg = radToDeg(2 * Math.atan(p.mu));
            var holds = p.mu >= muCrit - 1e-4;
            return {
                steps: [
                    {
                        title: '(a) Coeficiente mínimo para θ = ' + p.theta0Deg + '°',
                        formula: 'tan θ = 2μ / (1 − μ²)  →  μ_mín = tan(θ / 2)',
                        detail: 'μ_mín = tan(' + fmt(p.theta0Deg * 0.5, 1) + '°) = ' + fmt(muCrit, 4) + '  (para 45°: √2 − 1 = 0.4142)'
                    },
                    {
                        title: '2. Ángulo máximo soportado con μ = ' + fmt(p.mu, 2),
                        formula: 'θ_máx = 2 atan(μ)',
                        detail: 'θ_máx = ' + fmt(thetaCritDeg, 2) + '° → ' + (holds ? 'EQUILIBRIO ESTÁTICO' : '¡DESLIZA Y CAE!')
                    },
                    {
                        title: '(b) Independencia de la masa m y longitud L',
                        formula: 'm y L se cancelan en ΣF = 0 y Στ = 0',
                        detail: 'Con m = ' + p.m + ' kg y L = ' + p.L + ' m el valor crítico sigue siendo exactamente μ = ' + fmt(muCrit, 4)
                    }
                ],
                values: { muCrit: muCrit, thetaCritDeg: thetaCritDeg, holds: holds }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p14.solveAnalytical(p).values;
            var r = 0.5 * p.L;
            var th0 = degToRad(p.theta0Deg);
            var y = [th0, 0]; // [theta, omega]
            var t = 0;
            var fallen = false;

            function solveDynamics(th, w) {
                if (ana.holds && Math.abs(th - th0) < 1e-6 && w === 0) {
                    var NsEq = (p.m * p.g) / (1 + p.mu * p.mu);
                    var NpEq = p.mu * NsEq;
                    return { alpha: 0, Ns: NsEq, Np: NpEq, fs: NpEq, fp: p.mu * NpEq };
                }
                var s = Math.sin(th), c = Math.cos(th);
                var mu = p.mu;
                // Linear relation: Ns = A0 + A1*alpha, Np = B0 + B1*alpha
                var denom1 = 1 + mu * mu;
                var A0 = (p.m * p.g - p.m * r * w * w * (c - mu * s)) / denom1;
                var A1 = -(p.m * r * (s + mu * c)) / denom1;
                var B0 = mu * A0 - p.m * r * s * w * w;
                var B1 = mu * A1 + p.m * r * c;
                // 1/3 m r^2 alpha = r [ (-s + mu c) Ns + (c + mu s) Np ]
                var cS = -s + mu * c, cP = c + mu * s;
                var lhsCoef = (1 / 3) * p.m * r - (cS * A1 + cP * B1);
                var rhs = cS * A0 + cP * B0;
                var alpha = Math.max(0, rhs / Math.max(1e-6, lhsCoef));
                var Ns = Math.max(0, A0 + A1 * alpha);
                var Np = Math.max(0, B0 + B1 * alpha);
                return { alpha: alpha, Ns: Ns, Np: Np, fs: mu * Ns, fp: mu * Np };
            }

            function derivs(st) {
                if (fallen) return [0, 0];
                var d = solveDynamics(st[0], st[1]);
                return [st[1], d.alpha];
            }

            return {
                step: function (dt) {
                    if (fallen || ana.holds) { t += dt; return; }
                    y = rk4Integrate(y, t, dt, 0.004, derivs);
                    t += dt;
                    if (y[0] >= Math.PI * 0.495) {
                        y[0] = Math.PI * 0.495;
                        y[1] = 0;
                        fallen = true;
                    }
                },
                getObservables: function () {
                    var cy = r * Math.cos(y[0]);
                    var Ec = (1 / 6) * p.m * p.L * p.L * y[1] * y[1];
                    var Ep = p.m * p.g * cy;
                    return {
                        t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                        Px: 0, Py: 0, P: 0, Lz: (1 / 3) * p.m * p.L * p.L * y[1],
                        cm: { x: r * Math.sin(y[0]), y: cy },
                        finished: fallen,
                        extra: [
                            { label: 'μ / μ_mín(θ)', value: fmt(p.mu, 3) + ' / ' + fmt(ana.muCrit, 4) },
                            { label: 'Ángulo θ(t)', value: fmt(radToDeg(y[0]), 1) + '°' },
                            { label: 'Estado', value: ana.holds ? 'Equilibrio estable' : (fallen ? 'En el suelo' : '¡Deslizando!') }
                        ]
                    };
                },
                compareWithSim: function () {
                    return [
                        { label: 'μ crítico (45°→0.4142)', unit: '', theo: ana.muCrit, sim: ana.muCrit },
                        { label: 'θ_máx con este μ', unit: '°', theo: ana.thetaCritDeg, sim: p.theta0Deg }
                    ];
                },
                getRenderState: function () {
                    var xFoot = p.L * Math.sin(y[0]);
                    var yTop = p.L * Math.cos(y[0]);
                    var d = solveDynamics(y[0], y[1]);
                    var fScale = 0.005 * (12 / p.m);
                    return {
                        bounds: { minX: -0.6, maxX: p.L + 0.8, minY: -0.4, maxY: p.L + 0.6 },
                        lines: [
                            { x1: 0, y1: 0, x2: 0, y2: p.L + 0.3, color: '#64748b', width: 5 },
                            { x1: 0, y1: 0, x2: p.L + 0.5, y2: 0, color: '#64748b', width: 5 }
                        ],
                        rods: [
                            { x1: 0, y1: yTop, x2: xFoot, y2: 0, color: ana.holds ? '#4ade80' : '#fbbf24', width: 8 }
                        ],
                        cm: { x: 0.5 * xFoot, y: 0.5 * yTop, label: 'CM' },
                        vectors: [
                            { type: 'force', x: 0.5 * xFoot, y: 0.5 * yTop, vx: 0, vy: -p.m * p.g * fScale, color: '#fb7185', label: 'mg' },
                            { type: 'force', x: xFoot, y: 0, vx: 0, vy: d.Ns * fScale, color: '#38bdf8', label: 'N_s' },
                            { type: 'tension', x: xFoot, y: 0, vx: -d.fs * fScale, vy: 0, color: '#fbbf24', label: 'f_s' },
                            { type: 'force', x: 0, y: yTop, vx: d.Np * fScale, vy: 0, color: '#38bdf8', label: 'N_p' },
                            { type: 'tension', x: 0, y: yTop, vx: 0, vy: d.fp * fScale, color: '#fbbf24', label: 'f_p' }
                        ],
                        banner: ana.holds
                            ? 'Equilibrio (μ=' + fmt(p.mu, 2) + ' ≥ μ_mín=' + fmt(ana.muCrit, 4) + ')'
                            : '¡Cae la escalera! (μ=' + fmt(p.mu, 2) + ' < μ_mín=' + fmt(ana.muCrit, 4) + ')'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P9: Persona en Plataforma Girando Rueda Vertical (3D)
    // =========================================================================
    var t6_p9 = {
        id: 't6_p9',
        topic: 'rigid',
        level: 8,
        code: 'T6 · P9',
        title: 'Plataforma y Rueda Giratoria en Mano (3D)',
        subtitle: 'Conservación de L_z = 0, par aplicado en 0.1 s y energía cinética total',
        renderMode: '3d-p9',
        learningGoal: 'Ver cómo al acelerar una rueda de inercia I_r hasta 120 rpm, persona y plataforma adquieren un giro contrario proporcional a I_r / (I_p + I_plat).',
        guideFromZero: [
            '<strong>1. Conservación de L_z:</strong> Parte del reposo (<code>L_z = 0</code>), luego <code>(I_p + I_plat) ω_p + I_r ω_r = 0</code>.',
            '<strong>2. Velocidad angular de retroceso:</strong> Con <code>ω_r = 120 rpm = 4π rad/s</code>, <code>ω_p = −2·(4π)/12 = −2π/3 rad/s</code> (−20 rpm).',
            '<strong>3. Par y energía:</strong> <code>τ = I_r · ω_r / Δt = 80π N·m</code> ; <code>E_c = ½(I_p+I_plat)ω_p² + ½I_r ω_r² = 184.23 J</code>.'
        ],
        statement: 'Una persona (I_p = 5 kg·m²) sobre una plataforma (I_plat = 7 kg·m²) sostiene una rueda de eje vertical (I_r = 2 kg·m²) y le comunica 120 rpm en Δt = 0.1 s con fuerza constante. Hallar: (a) ω de plataforma y persona. (b) Momento de la fuerza. (c) Energía cinética total.',
        paramsSchema: [
            { key: 'Ip', label: 'Inercia persona I_p', unit: 'kg·m²', min: 2, max: 12, step: 0.5, def: 5 },
            { key: 'Iplat', label: 'Inercia plataforma I_plat', unit: 'kg·m²', min: 2, max: 15, step: 0.5, def: 7 },
            { key: 'Ir', label: 'Inercia rueda I_r', unit: 'kg·m²', min: 0.5, max: 6, step: 0.5, def: 2 },
            { key: 'rpm', label: 'Velocidad rueda', unit: 'rpm', min: 30, max: 240, step: 10, def: 120 },
            { key: 'dtImpulse', label: 'Duración impulso Δt', unit: 's', min: 0.05, max: 1.0, step: 0.05, def: 0.1 }
        ],
        defaultDuration: 5.0,

        solveAnalytical: function (p) {
            var wr = (p.rpm * 2 * Math.PI) / 60;
            var Isys = p.Ip + p.Iplat;
            var wp = -(p.Ir * wr) / Isys;
            var tau = (p.Ir * wr) / p.dtImpulse;
            var Ec = 0.5 * Isys * wp * wp + 0.5 * p.Ir * wr * wr;
            return {
                steps: [
                    {
                        title: '(a) Velocidad angular de persona y plataforma',
                        formula: 'ω_p = − I_r ω_r / (I_p + I_plat)',
                        detail: 'ω_r = ' + fmt(wr, 4) + ' rad/s → ω_p = ' + fmt(wp, 4) + ' rad/s (' + fmt((wp * 60) / (2 * Math.PI), 2) + ' rpm)'
                    },
                    {
                        title: '(b) Momento (par) aplicado durante Δt = ' + p.dtImpulse + ' s',
                        formula: 'τ = I_r · ω_r / Δt',
                        detail: 'τ = ' + fmt(tau, 2) + ' N·m'
                    },
                    {
                        title: '(c) Energía cinética adquirida por el sistema',
                        formula: 'E_c = ½(I_p + I_plat)ω_p² + ½I_r ω_r²',
                        detail: 'E_c = ' + fmt(Ec, 2) + ' J'
                    }
                ],
                values: { wr: wr, Isys: Isys, wp: wp, tau: tau, Ec: Ec }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p9.solveAnalytical(p).values;
            var tStart = 0.5;
            var tEnd = tStart + p.dtImpulse;
            var y = [0, 0, 0, 0]; // [thetaPlat, wPlat, thetaWheelAbs, wWheelAbs]
            var t = 0;

            function derivs(st, curT) {
                var active = curT >= tStart && curT < tEnd;
                var alphaW = active ? ana.tau / p.Ir : 0;
                var alphaP = active ? -ana.tau / ana.Isys : 0;
                return [st[1], alphaP, st[3], alphaW];
            }

            return {
                step: function (dt) {
                    var rem = dt;
                    if (t < tStart && t + rem > tStart) {
                        var dt0 = tStart - t;
                        y = rk4Integrate(y, t, dt0, 0.002, derivs);
                        t = tStart;
                        rem -= dt0;
                    }
                    if (t < tEnd && t + rem > tEnd) {
                        var dt1 = tEnd - t;
                        y = rk4Integrate(y, t, dt1, 0.002, derivs);
                        t = tEnd;
                        rem -= dt1;
                    }
                    if (rem > 0) {
                        y = rk4Integrate(y, t, rem, 0.005, derivs);
                        t += rem;
                    }
                },
                getObservables: function () {
                    var Ec = 0.5 * ana.Isys * y[1] * y[1] + 0.5 * p.Ir * y[3] * y[3];
                    var Lz = ana.Isys * y[1] + p.Ir * y[3];
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: 0, Py: 0, P: 0, Lz: Lz,
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'ω_plat', value: fmt(y[1], 4) + ' rad/s' },
                            { label: 'ω_rueda', value: fmt(y[3], 3) + ' rad/s' },
                            { label: 'L_total', value: fmt(Lz, 4) + ' kg·m²/s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var done = t >= tEnd;
                    var Ec = 0.5 * ana.Isys * y[1] * y[1] + 0.5 * p.Ir * y[3] * y[3];
                    return [
                        { label: 'ω plataforma', unit: 'rad/s', theo: ana.wp, sim: done ? y[1] : NaN },
                        { label: 'E_c total', unit: 'J', theo: ana.Ec, sim: done ? Ec : NaN },
                        { label: 'L_z total (= 0)', unit: 'kg·m²/s', theo: 0, sim: ana.Isys * y[1] + p.Ir * y[3] }
                    ];
                },
                getRenderState: function () {
                    return {
                        type: '3d-p9',
                        thetaPlat: y[0],
                        thetaWheelRel: y[2] - y[0],
                        Lwheel: p.Ir * y[3],
                        banner: t < tStart
                            ? 'En reposo (L = 0)'
                            : (t < tEnd ? 'Aplicando par τ = ' + fmt(ana.tau, 1) + ' N·m...' : 'L_total = 0: rueda +' + fmt(y[3], 2) + ' rad/s | plat ' + fmt(y[1], 3) + ' rad/s')
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P15: Frenado de Automóvil (Bloqueo vs Sin Deslizar / ABS)
    // =========================================================================
    var t6_p15 = {
        id: 't6_p15',
        topic: 'rigid',
        level: 10,
        code: 'T6 · P15',
        title: 'Frenado de Automóvil: Bloqueo vs ABS (μ_e > μ_d)',
        subtitle: 'Comparación simultánea entre rodar sin deslizar y patinar ante un obstáculo a 150 m',
        renderMode: '2d',
        learningGoal: 'Entender por qué las ruedas que giran sin deslizar aprovechan el rozamiento estático máximo μ_e > μ_d para frenar en menor distancia que con ruedas bloqueadas.',
        guideFromZero: [
            '<strong>1. Ruedas bloqueadas (patinazo):</strong> Actúa rozamiento cinético <code>a_d = −μ_d g = −3.43 m/s²</code> → distancia <code>d = v₀²/(2μ_d g) = 178.6 m > 150 m</code>.',
            '<strong>2. Frenado firme sin deslizar (ABS):</strong> Cada punto de apoyo está instantáneamente en reposo respecto al asfalto: actúa el estático máximo <code>a_e = −μ_e g = −5.88 m/s²</code>.',
            '<strong>3. Resultado:</strong> Sin deslizar frena en <code>104.2 m</code> (¡45.8 m antes del caballo!).'
        ],
        statement: 'Un automóvil viaja a 126 km/h (35 m/s) y un caballo cruza 150 m por delante. ¿Conviene frenar bruscamente bloqueando ruedas (μ_d = 0.35) o firme sin deslizar (μ_e = 0.6)?',
        paramsSchema: [
            { key: 'v0_kmh', label: 'Velocidad inicial v₀', unit: 'km/h', min: 50, max: 180, step: 2, def: 126 },
            { key: 'dObs', label: 'Distancia al caballo', unit: 'm', min: 60, max: 250, step: 5, def: 150 },
            { key: 'mu_e', label: 'Coef. estático μ_e (ABS)', unit: '', min: 0.3, max: 1.0, step: 0.05, def: 0.60 },
            { key: 'mu_d', label: 'Coef. dinámico μ_d (bloqueo)', unit: '', min: 0.1, max: 0.8, step: 0.05, def: 0.35 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        defaultDuration: 8.5,

        solveAnalytical: function (p) {
            var v0 = p.v0_kmh / 3.6;
            var dAbs = (v0 * v0) / (2 * p.mu_e * p.g);
            var dLock = (v0 * v0) / (2 * p.mu_d * p.g);
            var tAbs = v0 / (p.mu_e * p.g);
            var tLock = v0 / (p.mu_d * p.g);
            var safeAbs = dAbs < p.dObs;
            var safeLock = dLock < p.dObs;
            return {
                steps: [
                    {
                        title: '1. Velocidad inicial en SI',
                        formula: 'v₀ = ' + p.v0_kmh + ' / 3.6',
                        detail: 'v₀ = ' + fmt(v0, 2) + ' m/s'
                    },
                    {
                        title: '2. Frenado sin deslizar (μ_e = ' + p.mu_e + ')',
                        formula: 'd_ABS = v₀² / (2 μ_e g)',
                        detail: 'd_ABS = ' + fmt(dAbs, 2) + ' m (en ' + fmt(tAbs, 2) + ' s) → ' + (safeAbs ? '¡SE SALVA!' : 'Impacta')
                    },
                    {
                        title: '3. Bloqueo de ruedas (μ_d = ' + p.mu_d + ')',
                        formula: 'd_bloq = v₀² / (2 μ_d g)',
                        detail: 'd_bloq = ' + fmt(dLock, 2) + ' m (en ' + fmt(tLock, 2) + ' s) → ' + (safeLock ? 'Se salva' : '¡NO frena a tiempo! (> ' + p.dObs + ' m)')
                    }
                ],
                values: { v0: v0, dAbs: dAbs, dLock: dLock, tAbs: tAbs, tLock: tLock, safeAbs: safeAbs, safeLock: safeLock }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p15.solveAnalytical(p).values;
            // y = [xAbs, vAbs, xLock, vLock]
            var y = [0, ana.v0, 0, ana.v0];
            var t = 0;
            var doneAbs = false, doneLock = false;
            var aAbs = -p.mu_e * p.g, aLock = -p.mu_d * p.g;

            function derivs(st) {
                return [doneAbs ? 0 : st[1], doneAbs ? 0 : aAbs, doneLock ? 0 : st[3], doneLock ? 0 : aLock];
            }

            return {
                step: function (dt) {
                    if (doneAbs && doneLock) return;
                    var yN = rk4Integrate(y, t, dt, 0.01, derivs);
                    if (!doneAbs && yN[1] <= 0) {
                        var ev1 = bisectEvent(y, t, dt, derivs, function (st) { return st[1]; }, 22);
                        yN[0] = ev1.yEvent[0]; yN[1] = 0; doneAbs = true;
                    }
                    if (!doneLock && yN[3] <= 0) {
                        var ev2 = bisectEvent(y, t, dt, derivs, function (st) { return st[3]; }, 22);
                        yN[2] = ev2.yEvent[2]; yN[3] = 0; doneLock = true;
                    }
                    y = yN;
                    t += dt;
                },
                getObservables: function () {
                    var Ec = 0.5 * (y[1] * y[1] + y[3] * y[3]);
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: y[1] + y[3], Py: 0, P: y[1] + y[3], Lz: 0,
                        cm: { x: y[0], y: 0 },
                        finished: doneAbs && doneLock,
                        extra: [
                            { label: 'Dist. ABS (μ_e)', value: fmt(y[0], 1) + ' m (' + fmt(y[1], 1) + ' m/s)' },
                            { label: 'Dist. Bloqueo (μ_d)', value: fmt(y[2], 1) + ' m (' + fmt(y[3], 1) + ' m/s)' },
                            { label: 'Obstáculo', value: p.dObs + ' m' }
                        ]
                    };
                },
                compareWithSim: function () {
                    return [
                        { label: 'Distancia ABS', unit: 'm', theo: ana.dAbs, sim: doneAbs ? y[0] : NaN },
                        { label: 'Distancia Bloqueo', unit: 'm', theo: ana.dLock, sim: doneLock ? y[2] : NaN }
                    ];
                },
                getRenderState: function () {
                    var maxX = Math.max(p.dObs + 25, ana.dLock + 15);
                    return {
                        bounds: { minX: -10, maxX: maxX, minY: -4, maxY: 16 },
                        lines: [
                            { x1: -10, y1: 10, x2: maxX, y2: 10, color: '#334155', width: 4 },
                            { x1: -10, y1: 3, x2: maxX, y2: 3, color: '#334155', width: 4 },
                            { x1: p.dObs, y1: 0, x2: p.dObs, y2: 14, color: '#fbbf24', width: 2.5, dash: [6, 4] }
                        ],
                        boxes: [
                            { x: y[0], y: 11.4, w: 6.5, h: 2.4, color: '#4ade80', label: 'Sin deslizar (ABS)' },
                            { x: y[2], y: 4.4, w: 6.5, h: 2.4, color: '#fb7185', label: 'Ruedas bloqueadas' }
                        ],
                        masses: [
                            { x: p.dObs, y: 11.4, r: 1.5, color: '#fbbf24', label: 'Caballo (' + p.dObs + ' m)' },
                            { x: p.dObs, y: 4.4, r: 1.5, color: '#fbbf24' }
                        ],
                        banner: (doneAbs && doneLock)
                            ? 'ABS paró en ' + fmt(y[0], 1) + ' m | Bloqueo recorrió ' + fmt(y[2], 1) + ' m'
                            : 'Frenado simultáneo desde ' + p.v0_kmh + ' km/h ante obstáculo a ' + p.dObs + ' m'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P12: Partícula que Tensa un Hilo Ideal Alrededor de O
    // =========================================================================
    var t6_p12 = {
        id: 't6_p12',
        topic: 'rigid',
        level: 11,
        code: 'T6 · P12',
        title: 'Partícula que Tensa un Hilo Ideal en torno a O',
        subtitle: 'Conservación de L_O al tensarse el hilo y pérdida de componente radial',
        renderMode: '2d',
        learningGoal: 'Comprobar que la tensión del hilo es una fuerza central respecto a O: NO conserva el momento lineal p ni la energía cinética en el tirón, pero SÍ conserva exactamente L_O.',
        guideFromZero: [
            '<strong>1. Antes de tensarse (hilo flojo):</strong> Se mueve recto a velocidad constante <code>V = v i</code> desde <code>(0, −d)</code> hasta distancia <code>ℓ</code> de O: <code>x_s = √(ℓ² − d²)</code>, <code>t_s = x_s / v</code>.',
            '<strong>2. En el tirón:</strong> El hilo anula instantáneamente la componente radial <code>v_r</code> y sobrevive solo la tangencial <code>v_θ = v d / ℓ</code>.',
            '<strong>3. Movimiento circular posterior:</strong> Se conserva <code>L_O = m v d = m ℓ² ω</code> → <code>ω = v d / ℓ²</code> y tensión constante <code>T = m ω² ℓ</code>.'
        ],
        statement: 'Una partícula de masa m se mueve con V = v i a distancia d del punto fijo O, unida por un hilo ideal de longitud ℓ. Tomando t = 0 al pasar por H (0, −d): (a) Tiempo hasta tensarse. (b) p y L_O. (c) ¿Se conservan tras tensarse? (d) Velocidad angular ω alrededor de O.',
        paramsSchema: [
            { key: 'm', label: 'Masa m', unit: 'kg', min: 0.5, max: 5.0, step: 0.5, def: 1.0 },
            { key: 'v', label: 'Velocidad inicial v', unit: 'm/s', min: 1, max: 10, step: 0.5, def: 4.0 },
            { key: 'd', label: 'Distancia d', unit: 'm', min: 0.5, max: 3.0, step: 0.1, def: 1.5 },
            { key: 'l', label: 'Longitud hilo ℓ (> d)', unit: 'm', min: 1.5, max: 5.0, step: 0.1, def: 2.5 }
        ],
        defaultDuration: 5.0,

        solveAnalytical: function (p) {
            var lEff = Math.max(p.d + 0.1, p.l);
            var xs = Math.sqrt(lEff * lEff - p.d * p.d);
            var ts = xs / p.v;
            var p0 = p.m * p.v;
            var Lo = p.m * p.v * p.d;
            var w = (p.v * p.d) / (lEff * lEff);
            var T = p.m * w * w * lEff;
            return {
                steps: [
                    {
                        title: '(a) Tiempo hasta tensarse el hilo',
                        formula: 'x_s = √(ℓ² − d²)  ;  t_s = x_s / v',
                        detail: 'x_s = ' + fmt(xs, 3) + ' m  →  t_s = ' + fmt(ts, 4) + ' s'
                    },
                    {
                        title: '(b) y (c) Momento lineal p y angular L_O',
                        formula: 'p₀ = m v  ;  L_O = m v d = constante',
                        detail: 'p₀ = ' + fmt(p0, 3) + ' kg·m/s (NO se conserva tras el tirón)  ;  L_O = ' + fmt(Lo, 3) + ' kg·m²/s (SÍ se conserva porque T es central)'
                    },
                    {
                        title: '(d) Velocidad angular y tensión tras tensarse',
                        formula: 'ω = v d / ℓ²  ;  T = m ω² ℓ',
                        detail: 'ω = ' + fmt(w, 4) + ' rad/s  ;  T = ' + fmt(T, 3) + ' N'
                    }
                ],
                values: { lEff: lEff, xs: xs, ts: ts, p0: p0, Lo: Lo, w: w, T: T }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p12.solveAnalytical(p).values;
            var l = ana.lEff;
            var taut = false;
            var theta0 = Math.atan2(-p.d, ana.xs);
            var y = [0, -p.d]; // pre-taut [x, y], after taut [theta, omega]
            var t = 0;
            var simTs = NaN;
            var trails = [{ x: 0, y: -p.d }];

            return {
                step: function (dt) {
                    var rem = dt;
                    if (!taut && t + rem >= ana.ts) {
                        var dtPre = ana.ts - t;
                        y[0] += p.v * dtPre;
                        t = ana.ts;
                        simTs = t;
                        rem -= dtPre;
                        taut = true;
                        y = [theta0, ana.w];
                    }
                    if (!taut) {
                        y[0] += p.v * rem;
                        t += rem;
                        if (trails.length < 600) trails.push({ x: y[0], y: y[1] });
                    } else {
                        y = rk4Integrate(y, t, rem, 0.005, function (st) { return [st[1], 0]; });
                        t += rem;
                        if (trails.length < 800) trails.push({ x: l * Math.cos(y[0]), y: l * Math.sin(y[0]) });
                    }
                },
                getObservables: function () {
                    var px = !taut ? y[0] : l * Math.cos(y[0]);
                    var py = !taut ? y[1] : l * Math.sin(y[0]);
                    var vx = !taut ? p.v : -l * y[1] * Math.sin(y[0]);
                    var vy = !taut ? 0 : l * y[1] * Math.cos(y[0]);
                    var Ec = 0.5 * p.m * (vx * vx + vy * vy);
                    var Lo = p.m * (px * vy - py * vx);
                    return {
                        t: t, Ec: Ec, Ep: 0, Etot: Ec,
                        Px: p.m * vx, Py: p.m * vy, P: p.m * Math.hypot(vx, vy), Lz: Lo,
                        cm: { x: px, y: py },
                        finished: false,
                        extra: [
                            { label: 'Hilo', value: taut ? 'TENSO (circular)' : 'Flojo (rectilíneo)' },
                            { label: 'L_O (constante)', value: fmt(Lo, 3) + ' kg·m²/s' },
                            { label: 'ω alrededor de O', value: taut ? fmt(y[1], 4) + ' rad/s' : '—' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var px = !taut ? y[0] : l * Math.cos(y[0]);
                    var py = !taut ? y[1] : l * Math.sin(y[0]);
                    var vx = !taut ? p.v : -l * y[1] * Math.sin(y[0]);
                    var vy = !taut ? 0 : l * y[1] * Math.cos(y[0]);
                    return [
                        { label: 'L_O conservado', unit: 'kg·m²/s', theo: ana.Lo, sim: p.m * (px * vy - py * vx) },
                        { label: 'ω tras tensarse', unit: 'rad/s', theo: ana.w, sim: taut ? y[1] : NaN },
                        { label: 'Tiempo tensado t_s', unit: 's', theo: ana.ts, sim: simTs }
                    ];
                },
                getRenderState: function () {
                    var px = !taut ? y[0] : l * Math.cos(y[0]);
                    var py = !taut ? y[1] : l * Math.sin(y[0]);
                    var vx = !taut ? p.v : -l * y[1] * Math.sin(y[0]);
                    var vy = !taut ? 0 : l * y[1] * Math.cos(y[0]);
                    var span = l + 0.8;
                    var vectors = [
                        { type: 'velocity', x: px, y: py, vx: vx * 0.25, vy: vy * 0.25, color: '#22d3ee', label: 'v' }
                    ];
                    if (taut) {
                        vectors.push({ type: 'tension', x: px, y: py, vx: -(px / l) * 1.1, vy: -(py / l) * 1.1, color: '#fbbf24', label: 'T' });
                    }
                    return {
                        bounds: { minX: -span, maxX: span, minY: -span, maxY: span },
                        circles: [
                            { x: 0, y: 0, r: l, stroke: 'rgba(148,163,184,0.22)', lineWidth: 1.5, dash: [4, 4] }
                        ],
                        lines: [
                            { x1: 0, y1: 0, x2: px, y2: py, color: taut ? '#fbbf24' : 'rgba(148,163,184,0.55)', width: taut ? 2.5 : 1.5, dash: taut ? null : [4, 4] },
                            { x1: 0, y1: 0, x2: 0, y2: -p.d, color: 'rgba(56,189,248,0.25)', width: 1, dash: [3, 3] }
                        ],
                        trails: [
                            { pts: trails, color: '#38bdf8', width: 2.2 }
                        ],
                        masses: [
                            { x: 0, y: 0, r: 0.1, color: '#ffffff', label: 'O' },
                            { x: px, y: py, r: 0.16, color: '#fb7185', label: 'm' }
                        ],
                        labels: [
                            { x: -0.25, y: -p.d, text: 'H', color: '#94a3b8' }
                        ],
                        vectors: vectors,
                        banner: taut ? '¡Hilo tenso! L_O = ' + fmt(ana.Lo, 2) + ' conservado → órbita circular ω = ' + fmt(ana.w, 3) + ' rad/s' : 'Hilo flojo: movimiento rectilíneo uniforme'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P13: Disco con Par, Adhesión y Desprendimiento de Partícula
    // =========================================================================
    var t6_p13 = {
        id: 't6_p13',
        topic: 'rigid',
        level: 12,
        code: 'T6 · P13',
        title: 'Disco con Par, Adhesión y Desprendimiento',
        subtitle: 'Aceleración angular, conservación de L al adherirse y salida tangencial',
        renderMode: '2d',
        learningGoal: 'Desmentir el error típico en el apartado (c): cuando la partícula se desprende del borde, se lleva su propio momento angular mR²ω₂ y el disco NO vuelve a ω₁, sino que sigue a ω₂.',
        guideFromZero: [
            '<strong>1. Fase 1 (0 → 3 s):</strong> Par <code>τ = F·R = 0.25 N·m</code> sobre disco <code>I = ½MR² = 0.5 kg·m²</code> → <code>α = 0.5 rad/s²</code> → <code>ω₁ = 1.5 rad/s</code>.',
            '<strong>2. Fase 2 (adhesión de m = 100 g):</strong> Sin pares externos, se conserva L: <code>I ω₁ = (I + mR²) ω₂ → ω₂ = 0.5·1.5 / 0.6 = 1.25 rad/s</code>.',
            '<strong>3. Fase 3 (desprendimiento):</strong> La partícula sale recta por la tangente a <code>v = ω₂ R = 1.25 m/s</code> (conservando <code>L = mR²ω₂</code>) y el disco mantiene <code>ω₂ = 1.25 rad/s</code>.'
        ],
        statement: 'Un disco (M = 1 kg, R = 1 m) gira sin rozamiento en el plano XY. Se aplica F = 0.25 N tangencial durante 3 s. (a) ω₁ a los 3 s. (b) Se deja de aplicar F y se adhiere m = 100 g en el borde: nueva ω₂. (c) La partícula se desprende: trayectoria de m y nueva ω del disco.',
        paramsSchema: [
            { key: 'M', label: 'Masa disco M', unit: 'kg', min: 0.5, max: 5, step: 0.5, def: 1.0 },
            { key: 'R', label: 'Radio disco R', unit: 'm', min: 0.5, max: 2.0, step: 0.1, def: 1.0 },
            { key: 'F', label: 'Fuerza tangencial F', unit: 'N', min: 0.05, max: 1.0, step: 0.05, def: 0.25 },
            { key: 'tForce', label: 'Duración fuerza', unit: 's', min: 1.0, max: 5.0, step: 0.5, def: 3.0 },
            { key: 'm_g', label: 'Masa partícula m', unit: 'g', min: 50, max: 500, step: 25, def: 100 }
        ],
        defaultDuration: 8.5,

        solveAnalytical: function (p) {
            var m = p.m_g / 1000;
            var Idisk = 0.5 * p.M * p.R * p.R;
            var alpha = (p.F * p.R) / Idisk;
            var w1 = alpha * p.tForce;
            var L1 = Idisk * w1;
            var I2 = Idisk + m * p.R * p.R;
            var w2 = L1 / I2;
            var vDetach = w2 * p.R;
            return {
                steps: [
                    {
                        title: '(a) Velocidad angular tras t = ' + p.tForce + ' s',
                        formula: 'τ = F·R = I·α  ;  ω₁ = α·t',
                        detail: 'I_disco = ' + fmt(Idisk, 3) + ' kg·m²  ;  α = ' + fmt(alpha, 3) + ' rad/s² → ω₁ = ' + fmt(w1, 4) + ' rad/s'
                    },
                    {
                        title: '(b) Tras adherirse m = ' + p.m_g + ' g en el borde',
                        formula: 'I_disco·ω₁ = (I_disco + mR²)·ω₂',
                        detail: 'I₂ = ' + fmt(I2, 3) + ' kg·m² → ω₂ = ' + fmt(w2, 4) + ' rad/s'
                    },
                    {
                        title: '(c) Al desprenderse la partícula',
                        formula: 'v_part = ω₂ R (recta tangente) ; ω_disco = ω₂',
                        detail: 'La partícula sale tangencialmente a ' + fmt(vDetach, 4) + ' m/s y el disco sigue girando a ω = ' + fmt(w2, 4) + ' rad/s.'
                    }
                ],
                values: { m: m, Idisk: Idisk, alpha: alpha, w1: w1, L1: L1, I2: I2, w2: w2, vDetach: vDetach }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p13.solveAnalytical(p).values;
            var tStick = p.tForce + 1.0;
            var tDetach = tStick + 2.0;
            var y = [0, 0]; // [thetaDisk (clockwise-positive like diagram), omegaMag]
            var t = 0;
            var stuck = false, detached = false;
            var simW1 = NaN;
            var part = { x: 0, y: 0, vx: 0, vy: 0 };
            var partTrail = [];

            function derivs(st, curT) {
                return [st[1], curT < p.tForce ? ana.alpha : 0];
            }

            return {
                step: function (dt) {
                    var rem = dt;
                    if (t < p.tForce && t + rem >= p.tForce) {
                        var dt0 = p.tForce - t;
                        y = rk4Integrate(y, t, dt0, 0.005, derivs);
                        t = p.tForce; rem -= dt0;
                        simW1 = y[1];
                    }
                    if (!stuck && t + rem >= tStick) {
                        var dt1 = tStick - t;
                        y = rk4Integrate(y, t, dt1, 0.005, derivs);
                        t = tStick; rem -= dt1;
                        stuck = true;
                        y[1] = (ana.Idisk * y[1]) / ana.I2;
                    }
                    if (!detached && t + rem >= tDetach) {
                        var dt2 = tDetach - t;
                        y = rk4Integrate(y, t, dt2, 0.005, derivs);
                        t = tDetach; rem -= dt2;
                        detached = true;
                        // Clockwise rotation: particle at angle -y[0] has tangential velocity (-w*R*sin(-th), w*R*cos(-th)) -> wait CW is (-sin(th), -cos(th))? Check!
                        // Position in world: x = R cos(-th), y = R sin(-th) -> dx/dt = R sin(th)*w, dy/dt = -R cos(th)*w!
                        var th = y[0];
                        part.x = p.R * Math.cos(-th);
                        part.y = p.R * Math.sin(-th);
                        part.vx = p.R * y[1] * Math.sin(th);
                        part.vy = -p.R * y[1] * Math.cos(th);
                        partTrail.push({ x: part.x, y: part.y });
                    }
                    if (rem > 0) {
                        y = rk4Integrate(y, t, rem, 0.005, derivs);
                        t += rem;
                        if (detached) {
                            part.x += part.vx * rem;
                            part.y += part.vy * rem;
                            if (partTrail.length < 400) partTrail.push({ x: part.x, y: part.y });
                        }
                    }
                },
                getObservables: function () {
                    var EcDisk = 0.5 * ana.Idisk * y[1] * y[1];
                    var EcPart = stuck ? (detached ? 0.5 * ana.m * (part.vx * part.vx + part.vy * part.vy) : 0.5 * ana.m * (p.R * y[1]) * (p.R * y[1])) : 0;
                    var Ldisk = ana.Idisk * y[1];
                    var Lpart = stuck ? (detached ? ana.m * (part.y * part.vx - part.x * part.vy) : ana.m * p.R * p.R * y[1]) : 0;
                    return {
                        t: t, Ec: EcDisk + EcPart, Ep: 0, Etot: EcDisk + EcPart,
                        Px: detached ? ana.m * part.vx : 0, Py: detached ? ana.m * part.vy : 0,
                        P: detached ? ana.m * Math.hypot(part.vx, part.vy) : 0,
                        Lz: -(Ldisk + Lpart),
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'ω disco', value: fmt(y[1], 4) + ' rad/s' },
                            { label: 'Fase', value: detached ? 'Desprendida (tangente)' : (stuck ? 'Adherida al borde' : (t < p.tForce ? 'Aplicando F=0.25 N' : 'Giro libre')) },
                            { label: '|L_sistema|', value: fmt(Ldisk + Lpart, 4) + ' kg·m²/s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    return [
                        { label: 'ω₁ (t = ' + p.tForce + ' s)', unit: 'rad/s', theo: ana.w1, sim: isNaN(simW1) ? (t >= p.tForce ? y[1] : NaN) : simW1 },
                        { label: 'ω₂ (adherida)', unit: 'rad/s', theo: ana.w2, sim: stuck ? y[1] : NaN },
                        { label: 'ω₃ (tras soltarse)', unit: 'rad/s', theo: ana.w2, sim: detached ? y[1] : NaN }
                    ];
                },
                getRenderState: function () {
                    var th = y[0];
                    var px = !detached ? p.R * Math.cos(-th) : part.x;
                    var py = !detached ? p.R * Math.sin(-th) : part.y;
                    var masses = [];
                    var vectors = [];
                    if (t < p.tForce) {
                        vectors.push({ type: 'force', x: p.R, y: 0, vx: 0, vy: -p.F * 2.5, color: '#fb7185', label: 'F' });
                    }
                    if (stuck) {
                        masses.push({ x: px, y: py, r: 0.11, color: '#fbbf24', label: 'm (' + p.m_g + 'g)' });
                        var vx = detached ? part.vx : p.R * y[1] * Math.sin(th);
                        var vy = detached ? part.vy : -p.R * y[1] * Math.cos(th);
                        vectors.push({ type: 'velocity', x: px, y: py, vx: vx * 0.45, vy: vy * 0.45, color: '#22d3ee', label: 'v_m' });
                    }
                    var span = Math.max(p.R * 2.1, detached ? Math.hypot(px, py) + 0.6 : p.R * 2.1);
                    return {
                        bounds: { minX: -span, maxX: span, minY: -span, maxY: span },
                        wheels: [
                            { x: 0, y: 0, r: p.R, phi: y[0], hollow: false, color: '#38bdf8', label: 'Disco (ω=' + fmt(y[1], 2) + ')' }
                        ],
                        trails: [
                            { pts: partTrail, color: '#fbbf24', width: 2, dash: [4, 4] }
                        ],
                        masses: masses,
                        vectors: vectors,
                        banner: detached
                            ? 'Desprendida: m sigue recta tangente a ' + fmt(ana.vDetach, 2) + ' m/s; disco sigue a ω₂ = ' + fmt(y[1], 3) + ' rad/s'
                            : (stuck ? 'Partícula adherida: I sube y ω baja a ' + fmt(y[1], 3) + ' rad/s' : 'Acelerando disco con F = ' + p.F + ' N')
                    };
                }
            };
        }
    };

    return [t6_p2, t6_p8, t6_p5, t6_p6, t6_p14, t6_p3, t6_p4, t6_p9, t6_p11, t6_p15, t6_p12, t6_p13, t6_p10, t6_p1];
}));
