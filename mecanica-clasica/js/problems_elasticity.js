/**
 * problems_elasticity.js — 4 simulaciones de Elasticidad, Torsión y Compresibilidad
 * Basado en "Hoja_problemas_team6.pdf" (Problemas 16, 17, 18 y 19).
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory(require('./rk4.js'));
    } else {
        root.ElasticityProblems = factory(root.PhysicsRK4);
    }
}(typeof self !== 'undefined' ? self : this, function (RK4) {
    'use strict';

    var rk4Integrate = RK4.rk4Integrate;
    var degToRad = RK4.degToRad;
    var fmt = RK4.fmt;

    // =========================================================================
    // T6 · P16: Péndulo con Cable Elástico (Acero vs Aluminio)
    // =========================================================================
    var t6_p16 = {
        id: 't6_p16',
        topic: 'elasticity',
        level: 1,
        code: 'T6 · P16',
        title: 'Péndulo con Cable Elástico (Acero vs Aluminio)',
        subtitle: 'Tensión máxima, alargamiento de Hooke ΔL y límite elástico σ_e',
        renderMode: '2d',
        learningGoal: 'Aplicar la Ley de Hooke longitudinal (ΔL = F·L / (E·A)) a una carga dinámica y comprobar si la tensión máxima supera el límite elástico σ_e del material.',
        guideFromZero: [
            '<strong>1. Velocidad y tensión máxima (abajo):</strong> <code>v_máx = √[2gL(1 − cos θ₀)]</code> y <code>T_máx = mg(3 − 2 cos θ₀)</code>.',
            '<strong>2. Alargamiento real:</strong> El esfuerzo es <code>σ = T_máx / A</code> y el alargamiento <code>ΔL = (T_máx · L) / (E · A)</code>.',
            '<strong>3. Acero vs Aluminio:</strong> Aunque el acero es más rígido (<code>E = 200 GPa</code>, se estira menos), su límite elástico es <code>250 MPa &lt; σ_máx (269.7 MPa)</code>, ¡luego cede! El aluminio (<code>σ_e = 415 MPa</code>) sí aguanta en régimen elástico.'
        ],
        statement: 'Un cable de L = 2.5 m y sección A = 0.8 mm² sostiene m = 15 kg, se separa 40° de la vertical y se suelta. Comparando acero (E = 2.0×10¹¹ Pa, σ_e = 250 MPa) y aluminio (E = 7.3×10¹⁰ Pa, σ_e = 415 MPa): (a) v_máx. (b) T_máx. (c) Alargamiento ΔL. (d) ¿Trabajan en régimen elástico?',
        paramsSchema: [
            { key: 'material', label: 'Material (0=Acero, 1=Aluminio)', unit: '', min: 0, max: 1, step: 1, def: 0 },
            { key: 'm', label: 'Masa colgada m', unit: 'kg', min: 5, max: 30, step: 1, def: 15 },
            { key: 'L', label: 'Longitud cable L', unit: 'm', min: 1.0, max: 5.0, step: 0.1, def: 2.5 },
            { key: 'A_mm2', label: 'Sección A', unit: 'mm²', min: 0.4, max: 3.0, step: 0.1, def: 0.8 },
            { key: 'theta0Deg', label: 'Ángulo inicial θ₀', unit: '°', min: 10, max: 75, step: 1, def: 40 },
            { key: 'g', label: 'Gravedad g', unit: 'm/s²', min: 1, max: 20, step: 0.1, def: 9.8 }
        ],
        quickActions: [
            { label: 'Cable de Acero (supera σ_e)', apply: function (p) { p.material = 0; p.A_mm2 = 0.8; p.m = 15; p.theta0Deg = 40; } },
            { label: 'Cable de Aluminio (elástico)', apply: function (p) { p.material = 1; p.A_mm2 = 0.8; p.m = 15; p.theta0Deg = 40; } }
        ],
        defaultDuration: 6.0,

        solveAnalytical: function (p) {
            var isAl = p.material >= 0.5;
            var E = isAl ? 7.3e10 : 2.0e11;
            var sigmaE = isAl ? 415e6 : 250e6;
            var matName = isAl ? 'Aluminio' : 'Acero';
            var A = p.A_mm2 * 1e-6;
            var th0 = degToRad(p.theta0Deg);
            var cos0 = Math.cos(th0);
            var vMax = Math.sqrt(2 * p.g * p.L * (1 - cos0));
            var TMax = p.m * p.g * (3 - 2 * cos0);
            var sigmaMax = TMax / A;
            var dL = (TMax * p.L) / (E * A);
            var dLSteel = (TMax * p.L) / (2.0e11 * A);
            var dLAl = (TMax * p.L) / (7.3e10 * A);
            var isElastic = sigmaMax <= sigmaE;

            return {
                steps: [
                    {
                        title: '(a) Velocidad máxima (aprox. cable inextensible)',
                        formula: 'v_máx = √[2 g L (1 − cos θ₀)]',
                        detail: 'v_máx = ' + fmt(vMax, 4) + ' m/s'
                    },
                    {
                        title: '(b) Tensión máxima en el punto más bajo',
                        formula: 'T_máx = m g + m v_máx² / L = m g (3 − 2 cos θ₀)',
                        detail: 'T_máx = ' + fmt(TMax, 2) + ' N  (esfuerzo σ_máx = T_máx / A = ' + fmt(sigmaMax / 1e6, 2) + ' MPa)'
                    },
                    {
                        title: '(c) Alargamiento real ΔL = T_máx·L / (E·A)',
                        formula: 'ΔL_acero = ' + fmt(dLSteel * 1000, 3) + ' mm  ;  ΔL_aluminio = ' + fmt(dLAl * 1000, 3) + ' mm',
                        detail: matName + ': ΔL = ' + fmt(dL * 1000, 3) + ' mm (' + fmt((dL / p.L) * 100, 3) + '% de L → valida la aprox. inextensible)'
                    },
                    {
                        title: '(d) Comprobación de régimen elástico (σ_máx ≤ σ_e)',
                        formula: 'σ_e,acero = 250 MPa  ;  σ_e,aluminio = 415 MPa',
                        detail: matName + ': σ_máx = ' + fmt(sigmaMax / 1e6, 2) + ' MPa ' + (isElastic ? '≤ ' + fmt(sigmaE / 1e6, 0) + ' MPa → SÍ trabaja en régimen elástico' : '> ' + fmt(sigmaE / 1e6, 0) + ' MPa → ¡SUPERA el límite elástico!')
                    }
                ],
                values: { matName: matName, E: E, sigmaE: sigmaE, A: A, vMax: vMax, TMax: TMax, sigmaMax: sigmaMax, dL: dL, isElastic: isElastic }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p16.solveAnalytical(p).values;
            var k = (ana.E * ana.A) / p.L;
            var th0 = degToRad(p.theta0Deg);
            var r0 = p.L + (p.m * p.g * Math.cos(th0)) / k;
            // State [r, vr, theta, omega]
            var y = [r0, 0, -th0, 0];
            var t = 0;
            var simVMax = 0, simTMax = 0, simDLMax = 0;
            var reachedBottom = false;
            var trails = [];

            function derivs(st) {
                var r = Math.max(0.2, st[0]), vr = st[1], th = st[2], w = st[3];
                var T = r > p.L ? k * (r - p.L) : 0;
                var ar = r * w * w + p.g * Math.cos(th) - T / p.m;
                var alpha = (-2 * vr * w - p.g * Math.sin(th)) / r;
                return [vr, ar, w, alpha];
            }

            return {
                step: function (dt) {
                    var maxSub = 0.0008;
                    var steps = Math.max(1, Math.ceil(dt / maxSub));
                    var h = dt / steps;
                    for (var i = 0; i < steps; i++) {
                        y = rk4Integrate(y, t, h, h, derivs);
                        t += h;
                        if (y[2] >= 0) reachedBottom = true;
                        var spd = Math.hypot(y[1], y[0] * y[3]);
                        var T = y[0] > p.L ? k * (y[0] - p.L) : 0;
                        if (spd > simVMax) simVMax = spd;
                        if (T > simTMax) simTMax = T;
                        if (y[0] - p.L > simDLMax) simDLMax = y[0] - p.L;
                    }
                    var px = y[0] * Math.sin(y[2]);
                    var py = p.L + 0.4 - y[0] * Math.cos(y[2]);
                    if (trails.length < 700) trails.push({ x: px, y: py });
                },
                getObservables: function () {
                    var v = Math.hypot(y[1], y[0] * y[3]);
                    var T = y[0] > p.L ? k * (y[0] - p.L) : 0;
                    var sigma = T / ana.A;
                    var dL = Math.max(0, y[0] - p.L);
                    var Ec = 0.5 * p.m * v * v;
                    var EpGrav = p.m * p.g * (p.L * (1 - Math.cos(y[2])));
                    var EpElast = 0.5 * k * dL * dL;
                    return {
                        t: t, Ec: Ec, Ep: EpGrav + EpElast, Etot: Ec + EpGrav + EpElast,
                        Px: p.m * (y[1] * Math.sin(y[2]) + y[0] * y[3] * Math.cos(y[2])),
                        Py: p.m * (-y[1] * Math.cos(y[2]) + y[0] * y[3] * Math.sin(y[2])),
                        P: p.m * v, Lz: p.m * y[0] * y[0] * y[3],
                        cm: { x: y[0] * Math.sin(y[2]), y: p.L + 0.4 - y[0] * Math.cos(y[2]) },
                        finished: false,
                        extra: [
                            { label: 'Material', value: ana.matName + (ana.isElastic ? ' (Elástico)' : ' (¡Fluencia!)') },
                            { label: 'Tensión T / σ', value: fmt(T, 1) + ' N | ' + fmt(sigma / 1e6, 1) + ' MPa' },
                            { label: 'Estiramiento ΔL', value: fmt(dL * 1000, 3) + ' mm' }
                        ]
                    };
                },
                compareWithSim: function () {
                    return [
                        { label: 'v_máx', unit: 'm/s', theo: ana.vMax, sim: reachedBottom ? simVMax : NaN },
                        { label: 'T_máx', unit: 'N', theo: ana.TMax, sim: reachedBottom ? simTMax : NaN },
                        { label: 'ΔL_máx', unit: 'mm', theo: ana.dL * 1000, sim: reachedBottom ? simDLMax * 1000 : NaN },
                        { label: 'σ_máx', unit: 'MPa', theo: ana.sigmaMax / 1e6, sim: reachedBottom ? (simTMax / ana.A) / 1e6 : NaN }
                    ];
                },
                getRenderState: function () {
                    var pyPivot = p.L + 0.4;
                    var dL = Math.max(0, y[0] - p.L);
                    // Visually exaggerate elongation x60 so elastic stretch is observable
                    var rVis = p.L + dL * 60;
                    var mx = rVis * Math.sin(y[2]);
                    var my = pyPivot - rVis * Math.cos(y[2]);
                    var T = y[0] > p.L ? k * (y[0] - p.L) : 0;
                    var sigma = T / ana.A;
                    var overLimit = sigma > ana.sigmaE;
                    var span = Math.max(2.0, p.L * 0.95);
                    var minY = Math.min(-0.35, my - 0.45);
                    return {
                        bounds: { minX: -span, maxX: span, minY: minY, maxY: pyPivot + 0.35 },
                        lines: [
                            { x1: -span * 0.6, y1: pyPivot, x2: span * 0.6, y2: pyPivot, color: '#475569', width: 5 }
                        ],
                        circles: [
                            { x: 0, y: pyPivot, r: p.L, stroke: 'rgba(148,163,184,0.22)', lineWidth: 1, dash: [4, 4], startAngle: -Math.PI * 0.78, endAngle: -Math.PI * 0.22 }
                        ],
                        springs: [
                            { x1: 0, y1: pyPivot, x2: mx, y2: my, coils: 18, ampPx: 4, color: overLimit ? '#ef4444' : (p.material >= 0.5 ? '#38bdf8' : '#cbd5e1'), width: 2 }
                        ],
                        trails: [
                            { pts: trails, color: 'rgba(56, 189, 248, 0.35)', width: 1.5 }
                        ],
                        masses: [
                            { x: mx, y: my, r: 0.14, color: overLimit ? '#ef4444' : '#fbbf24', label: 'm=' + p.m + 'kg' }
                        ],
                        labels: [
                            { x: 0, y: pyPivot - p.L - 0.22, text: 'Círculo punteado: longitud L sin estirar (ΔL ampliado ×60)', color: '#94a3b8', align: 'center' }
                        ],
                        vectors: [
                            { type: 'tension', x: mx, y: my, vx: -T * Math.sin(y[2]) * 0.004, vy: T * Math.cos(y[2]) * 0.004, color: overLimit ? '#ef4444' : '#4ade80', label: 'T=' + fmt(T, 0) + 'N' },
                            { type: 'force', x: mx, y: my, vx: 0, vy: -p.m * p.g * 0.004, color: '#fb7185', label: 'mg' }
                        ],
                        banner: ana.matName + ': σ = ' + fmt(sigma / 1e6, 1) + ' MPa (límite ' + fmt(ana.sigmaE / 1e6, 0) + ' MPa) | ΔL = ' + fmt(dL * 1000, 2) + ' mm'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P17: Volante Regulador de Reloj de Torsión (3D)
    // =========================================================================
    var t6_p17 = {
        id: 't6_p17',
        topic: 'elasticity',
        level: 2,
        code: 'T6 · P17',
        title: 'Volante de Reloj de Torsión (3D)',
        subtitle: 'Módulo de rigidez G, constante de torsión D y oscilación angular',
        renderMode: '3d-p17',
        learningGoal: 'Relacionar la geometría del hilo (diámetro d y longitud L) y su módulo de cizalla G con la constante de torsión D = πGr⁴/(2L) y el periodo T = 2π√(I/D).',
        guideFromZero: [
            '<strong>1. Inercia del volante (disco macizo):</strong> <code>I = ½ m R²</code>.',
            '<strong>2. Periodo del péndulo de torsión:</strong> El par recuperador es <code>τ = −D θ</code>, dando M.A.S. angular con <code>T = 2π √(I / D)</code>.',
            '<strong>3. Diámetro del hilo de latón:</strong> Como <code>D = π G r⁴ / (2L)</code> depende de la <strong>4ª potencia</strong> del radio, un pequeño cambio en el diámetro cambia drásticamente el periodo.'
        ],
        statement: 'El volante de un reloj de torsión es un disco macizo de R = 8 cm y m = 2 kg suspendido por un hilo de latón de L = 40 cm (G = 35 GPa). Si oscila con periodo T = 4 s, calcular el diámetro d que debe tener el hilo.',
        paramsSchema: [
            { key: 'm', label: 'Masa del disco m', unit: 'kg', min: 0.5, max: 6.0, step: 0.5, def: 2.0 },
            { key: 'R_cm', label: 'Radio del disco R', unit: 'cm', min: 4, max: 16, step: 1, def: 8 },
            { key: 'L_cm', label: 'Longitud hilo latón L', unit: 'cm', min: 15, max: 80, step: 5, def: 40 },
            { key: 'G_GPa', label: 'Módulo rigidez G', unit: 'GPa', min: 10, max: 85, step: 1, def: 35 },
            { key: 'd_mm', label: 'Diámetro del hilo d', unit: 'mm', min: 0.5, max: 2.5, step: 0.002, def: 1.164 },
            { key: 'theta0Deg', label: 'Amplitud angular θ₀', unit: '°', min: 15, max: 140, step: 5, def: 75 }
        ],
        quickActions: [
            { label: 'Diámetro solución (T = 4.00 s)', apply: function (p) { p.d_mm = 1.164; p.m = 2; p.R_cm = 8; p.L_cm = 40; p.G_GPa = 35; } },
            { label: 'Hilo doble de grueso (T = 1.00 s)', apply: function (p) { p.d_mm = 2.328; } }
        ],
        defaultDuration: 8.0,

        solveAnalytical: function (p) {
            var R = p.R_cm / 100;
            var L = p.L_cm / 100;
            var G = p.G_GPa * 1e9;
            var I = 0.5 * p.m * R * R;
            var r = (p.d_mm / 1000) * 0.5;
            var D = (Math.PI * G * Math.pow(r, 4)) / (2 * L);
            var T = 2 * Math.PI * Math.sqrt(I / D);
            var w = (2 * Math.PI) / T;
            var Dtarget4s = (4 * Math.PI * Math.PI * I) / 16.0;
            var dTarget4sMm = 2 * Math.pow((2 * L * Dtarget4s) / (Math.PI * G), 0.25) * 1000;
            var th0 = degToRad(p.theta0Deg);
            var Etot = 0.5 * D * th0 * th0;
            return {
                steps: [
                    {
                        title: '1. Momento de inercia del volante y constante de torsión D',
                        formula: 'I = ½ m R²  ;  D = π G r⁴ / (2 L)',
                        detail: 'I = ' + fmt(I, 5) + ' kg·m²  ;  D = ' + fmt(D, 6) + ' N·m/rad'
                    },
                    {
                        title: '2. Periodo de oscilación angular',
                        formula: 'T = 2π √(I / D)',
                        detail: 'Con d = ' + fmt(p.d_mm, 3) + ' mm → T = ' + fmt(T, 4) + ' s (ω = ' + fmt(w, 4) + ' rad/s)'
                    },
                    {
                        title: '3. Diámetro exacto requerido para T = 4.00 s',
                        formula: 'D_obj = 4π²I / T²  →  d = 2 [2 L D_obj / (π G)]^(1/4)',
                        detail: 'd_objetivo = ' + fmt(dTarget4sMm, 3) + ' mm  (r = ' + fmt(dTarget4sMm * 0.5, 3) + ' mm)  |  E_mec = ' + fmt(Etot * 1000, 3) + ' mJ'
                    }
                ],
                values: { I: I, D: D, T: T, w: w, dTarget4sMm: dTarget4sMm, Etot: Etot }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p17.solveAnalytical(p).values;
            var th0 = degToRad(p.theta0Deg);
            var y = [th0, 0]; // [theta, omega]
            var t = 0;
            var simT = NaN;

            function derivs(st) { return [st[1], -(ana.D / ana.I) * st[0]]; }

            return {
                step: function (dt) {
                    var yPrev = y;
                    y = rk4Integrate(y, t, dt, 0.005, derivs);
                    t += dt;
                    if (isNaN(simT) && t > 0.25 * ana.T && yPrev[1] > 0 && y[1] <= 0) {
                        var ev = RK4.bisectEvent(yPrev, t - dt, dt, derivs, function (st) { return st[1]; }, 22);
                        simT = ev.tEvent;
                    }
                },
                getObservables: function () {
                    var Ec = 0.5 * ana.I * y[1] * y[1];
                    var Ep = 0.5 * ana.D * y[0] * y[0];
                    return {
                        t: t, Ec: Ec, Ep: Ep, Etot: Ec + Ep,
                        Px: 0, Py: 0, P: 0, Lz: ana.I * y[1],
                        cm: { x: 0, y: 0 },
                        finished: false,
                        extra: [
                            { label: 'Ángulo θ(t)', value: fmt(RK4.radToDeg(y[0]), 2) + '°' },
                            { label: 'Par τ = −Dθ', value: fmt(-ana.D * y[0] * 1000, 3) + ' mN·m' },
                            { label: 'Periodo teórico', value: fmt(ana.T, 3) + ' s' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var Ec = 0.5 * ana.I * y[1] * y[1];
                    var Ep = 0.5 * ana.D * y[0] * y[0];
                    return [
                        { label: 'θ(t)', unit: 'rad', theo: th0 * Math.cos(ana.w * t), sim: y[0] },
                        { label: 'Periodo T', unit: 's', theo: ana.T, sim: simT },
                        { label: 'E_total', unit: 'mJ', theo: ana.Etot * 1000, sim: (Ec + Ep) * 1000 }
                    ];
                },
                getRenderState: function () {
                    return {
                        type: '3d-p17',
                        theta: y[0],
                        omega: y[1],
                        tau: -ana.D * y[0],
                        banner: 'Péndulo de torsión: d = ' + fmt(p.d_mm, 3) + ' mm → T = ' + fmt(ana.T, 3) + ' s'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P18: Grieta en Granito por Congelación del Agua (Compresibilidad)
    // =========================================================================
    var t6_p18 = {
        id: 't6_p18',
        topic: 'elasticity',
        level: 3,
        code: 'T6 · P18',
        title: 'Fractura de Roca por Agua Congelada',
        subtitle: 'Módulo de compresibilidad volumétrica B y presión de hielo confinado',
        renderMode: '2d',
        learningGoal: 'Calcular las enormes presiones internas que genera la expansión confinada de un fluido (ΔP = B · |ΔV/V|) al congelarse dentro de una grieta rígida.',
        guideFromZero: [
            '<strong>1. Compresibilidad volumétrica:</strong> Al congelarse, el agua quiere expandir su volumen un 9%, pero la roca rígida lo impide: equivale a comprimirlo un <code>ΔV/V ≈ 9%</code>.',
            '<strong>2. Incremento de presión:</strong> <code>ΔP = B · (ΔV / V)</code> (con <code>B = 2.05×10⁹ Pa</code>, alcanza cientos de megapascales).',
            '<strong>3. Fuerza y fractura:</strong> Sobre la cara mayor <code>A = 20×10 cm² = 0.02 m²</code> actúa <code>F = ΔP · A</code>, superando con creces los 8 MPa de resistencia del granito.'
        ],
        statement: 'Una grieta plana en granito (20 cm × 10 cm × 1 mm) llena de agua a 0 °C se enfría a −5 °C, aumentando el volumen libre un 9%. Si las paredes impiden expandirse y B_agua = 2.05×10⁹ Pa: (a) ΔP generado. (b) Fuerza sobre la pared mayor. (c) ¿Fractura el granito (σ_rot = 8 MPa)?',
        paramsSchema: [
            { key: 'expPct', label: 'Expansión libre ΔV/V', unit: '%', min: 1, max: 15, step: 0.5, def: 9.0 },
            { key: 'B_GPa', label: 'Módulo compresibilidad B', unit: 'GPa', min: 0.5, max: 4.0, step: 0.05, def: 2.05 },
            { key: 'len_cm', label: 'Largo grieta', unit: 'cm', min: 5, max: 40, step: 1, def: 20 },
            { key: 'wid_cm', label: 'Ancho grieta', unit: 'cm', min: 5, max: 30, step: 1, def: 10 },
            { key: 'sigRot_MPa', label: 'Resistencia granito', unit: 'MPa', min: 2, max: 25, step: 1, def: 8 }
        ],
        defaultDuration: 5.0,

        solveAnalytical: function (p) {
            var B = p.B_GPa * 1e9;
            var epsV = p.expPct / 100;
            var dP = B * epsV; // Standard course solution: B * (ΔV/V)
            var dPRefined = B * (epsV / (1 + epsV));
            var A = (p.len_cm / 100) * (p.wid_cm / 100);
            var F = dP * A;
            var sigRot = p.sigRot_MPa * 1e6;
            var fractures = dP >= sigRot;
            var fracAtFreezePct = Math.min(100, (sigRot / dP) * 100);
            return {
                steps: [
                    {
                        title: '(a) Incremento de presión confinada',
                        formula: 'ΔP = B · (ΔV / V)',
                        detail: 'ΔP = ' + fmt(dP / 1e6, 2) + ' MPa  (usando volumen final del hielo ΔV/(V+ΔV): ' + fmt(dPRefined / 1e6, 2) + ' MPa)'
                    },
                    {
                        title: '(b) Fuerza total sobre la cara mayor (A = ' + fmt(A * 1e4, 0) + ' cm²)',
                        formula: 'F = ΔP · A',
                        detail: 'F = ' + fmt(F / 1e6, 3) + ' MN (' + fmt(F / 9.8, 0) + ' kgf ≈ ' + fmt(F / 9800, 1) + ' toneladas-fuerza)'
                    },
                    {
                        title: '(c) Criterio de fractura del granito (σ_rot = ' + p.sigRot_MPa + ' MPa)',
                        formula: 'ΔP vs σ_rot',
                        detail: fractures
                            ? '¡SÍ FRACTURA! ' + fmt(dP / 1e6, 1) + ' MPa ≫ ' + p.sigRot_MPa + ' MPa (rompe apenas se congela un ' + fmt(fracAtFreezePct, 2) + '% del agua)'
                            : 'No alcanza la resistencia de fractura.'
                    }
                ],
                values: { dP: dP, dPRefined: dPRefined, A: A, F: F, sigRot: sigRot, fractures: fractures, fracAtFreezePct: fracAtFreezePct }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p18.solveAnalytical(p).values;
            var coolDuration = 5.0;
            var y = [0]; // [progress 0..1 from 0°C to -5°C]
            var t = 0;
            var simBreakP = NaN;

            function derivs() { return [t < coolDuration ? 1 / coolDuration : 0]; }

            return {
                step: function (dt) {
                    if (t >= coolDuration) return;
                    var pPrev = y[0];
                    y = rk4Integrate(y, t, Math.min(dt, coolDuration - t), 0.01, derivs);
                    t = Math.min(coolDuration, t + dt);
                    if (y[0] > 1) y[0] = 1;
                    if (isNaN(simBreakP) && ana.dP * pPrev < ana.sigRot && ana.dP * y[0] >= ana.sigRot) {
                        simBreakP = ana.sigRot / 1e6;
                    }
                },
                getObservables: function () {
                    var prog = y[0];
                    var tempC = -5.0 * prog;
                    var curP = ana.dP * prog;
                    var curF = ana.F * prog;
                    var broken = curP >= ana.sigRot;
                    var V0 = ana.A * 0.001;
                    var Eelast = 0.5 * curP * (prog * (p.expPct / 100)) * V0;
                    return {
                        t: t, Ec: 0, Ep: Eelast, Etot: Eelast,
                        Px: 0, Py: 0, P: 0, Lz: 0,
                        cm: { x: 0, y: 0 },
                        finished: t >= coolDuration,
                        extra: [
                            { label: 'Temperatura', value: fmt(tempC, 2) + ' °C (' + fmt(prog * 100, 0) + '% hielo)' },
                            { label: 'Presión ΔP(t)', value: fmt(curP / 1e6, 2) + ' MPa' },
                            { label: 'Estado granito', value: broken ? '¡FRACTURADO! (> ' + p.sigRot_MPa + ' MPa)' : 'Intacto' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var curP = ana.dP * y[0];
                    var curF = ana.F * y[0];
                    var done = y[0] >= 1 - 1e-6;
                    return [
                        { label: 'ΔP final (−5 °C)', unit: 'MPa', theo: ana.dP / 1e6, sim: done ? curP / 1e6 : NaN },
                        { label: 'Fuerza pared', unit: 'MN', theo: ana.F / 1e6, sim: done ? curF / 1e6 : NaN },
                        { label: 'Fractura en σ_rot', unit: 'MPa', theo: ana.fractures ? p.sigRot_MPa : NaN, sim: simBreakP }
                    ];
                },
                getRenderState: function () {
                    var prog = y[0];
                    var curP = ana.dP * prog;
                    var broken = curP >= ana.sigRot;
                    var crackOpen = broken ? Math.min(0.45, 0.08 + 0.35 * ((curP - ana.sigRot) / ana.dP)) : 0;
                    var gap = 0.14 + crackOpen;
                    var fVis = Math.min(1.4, 0.25 + (curP / ana.dP) * 1.1);
                    return {
                        bounds: { minX: -3.2, maxX: 3.2, minY: -2.2, maxY: 2.2 },
                        polys: [
                            // Left granite block
                            { pts: [{ x: -2.4 - crackOpen * 0.5, y: -1.4 }, { x: -gap * 0.5, y: -1.4 }, { x: -gap * 0.5, y: 1.4 }, { x: -2.4 - crackOpen * 0.5, y: 1.4 }], fill: 'rgba(100, 116, 139, 0.45)', stroke: broken ? '#ef4444' : '#94a3b8', lineWidth: 2 },
                            // Right granite block
                            { pts: [{ x: gap * 0.5, y: -1.4 }, { x: 2.4 + crackOpen * 0.5, y: -1.4 }, { x: 2.4 + crackOpen * 0.5, y: 1.4 }, { x: gap * 0.5, y: 1.4 }], fill: 'rgba(100, 116, 139, 0.45)', stroke: broken ? '#ef4444' : '#94a3b8', lineWidth: 2 },
                            // Water / Ice inside crack
                            { pts: [{ x: -gap * 0.5, y: -1.35 }, { x: gap * 0.5, y: -1.35 }, { x: gap * 0.5, y: 1.35 }, { x: -gap * 0.5, y: 1.35 }], fill: prog > 0.1 ? 'rgba(56, 189, 248, ' + (0.35 + 0.45 * prog) + ')' : 'rgba(14, 165, 233, 0.35)', stroke: '#38bdf8', lineWidth: 1.5 }
                        ],
                        lines: broken ? [
                            { x1: -gap * 0.5, y1: 0.4, x2: -1.3, y2: 0.9, color: '#ef4444', width: 2.5 },
                            { x1: gap * 0.5, y1: -0.2, x2: 1.4, y2: -0.8, color: '#ef4444', width: 2.5 }
                        ] : [],
                        labels: [
                            { x: -1.3, y: 1.58, text: 'Granito (σ_rot = ' + p.sigRot_MPa + ' MPa)', color: '#cbd5e1', align: 'center' },
                            { x: 1.3, y: 1.58, text: 'T = ' + fmt(-5 * prog, 1) + ' °C', color: '#38bdf8', align: 'center' },
                            { x: 0, y: -1.72, text: 'Hielo confinado: ΔP = ' + fmt(curP / 1e6, 1) + ' MPa | F = ' + fmt(ana.F * prog / 1e6, 2) + ' MN', color: broken ? '#fb7185' : '#fbbf24', align: 'center' }
                        ],
                        vectors: prog > 0.01 ? [
                            { type: 'force', x: -gap * 0.5, y: 0, vx: -fVis, vy: 0, color: '#ef4444', label: 'F' },
                            { type: 'force', x: gap * 0.5, y: 0, vx: fVis, vy: 0, color: '#ef4444', label: 'F' }
                        ] : [],
                        banner: broken
                            ? '¡ROCA FRACTURADA! ΔP = ' + fmt(curP / 1e6, 1) + ' MPa supera σ_rot = ' + p.sigRot_MPa + ' MPa'
                            : 'Congelando agua en grieta rígida: ΔP = ' + fmt(curP / 1e6, 2) + ' MPa'
                    };
                }
            };
        }
    };

    // =========================================================================
    // T6 · P19: Ensayo de Tracción y Curva Tensión-Deformación (σ vs ε)
    // =========================================================================
    var t6_p19 = {
        id: 't6_p19',
        topic: 'elasticity',
        level: 4,
        code: 'T6 · P19',
        title: 'Ensayo de Tracción: Curva σ–ε y Módulo de Young',
        subtitle: 'Región elástica lineal (Hooke), fluencia plástica y rotura a 90 N',
        renderMode: '2d',
        learningGoal: 'Leer experimentalmente una curva fuerza-alargamiento, transformarla en tensión σ = F/A frente a deformación ε = ΔL/L, y extraer σ_elástico y E.',
        guideFromZero: [
            '<strong>1. Tensión y deformación:</strong> <code>σ = F / A</code> (en MPa) y <code>ε = ΔL / L</code> (adimensional o en ‰).',
            '<strong>2. Región elástica (0 → 1.67 mm):</strong> La curva es recta hasta <code>F ≈ 100 N</code> → límite elástico <code>σ_e = 100 N / 10⁻⁶ m² = 100 MPa</code>.',
            '<strong>3. Módulo de Young:</strong> Pendiente de la recta inicial: <code>E = σ / ε = (60 MPa) / (0.001) = 60 GPa</code>.'
        ],
        statement: 'Un cable metálico cilíndrico de L = 1 m y sección A = 10⁻⁶ m² se somete a tracción según la gráfica experimental F–ΔL (lineal hasta 100 N en ~1.67 mm, pico 120 N en 8 mm, rotura a 90 N en ~15 mm). (a) Dibujar curva σ–ε y describir regiones. (b) Estimar σ_e y módulo de Young E.',
        paramsSchema: [
            { key: 'L', label: 'Longitud inicial L', unit: 'm', min: 0.5, max: 2.0, step: 0.1, def: 1.0 },
            { key: 'A_mm2', label: 'Sección transversal A', unit: 'mm²', min: 0.5, max: 3.0, step: 0.1, def: 1.0 },
            { key: 'rate', label: 'Velocidad de estirado', unit: 'mm/s', min: 1.0, max: 5.0, step: 0.5, def: 2.5 }
        ],
        defaultDuration: 6.5,

        solveAnalytical: function (p) {
            var A = p.A_mm2 * 1e-6;
            // Digitized curve scaled to length p.L (strain = dl_1m / 1m -> dl = eps * L)
            var pts = [
                [0, 0], [1 / 3, 20], [2 / 3, 40], [1.0, 60], [4 / 3, 80], [5 / 3, 100],
                [2.0, 105], [3.0, 110], [5.0, 115], [8.0, 120],
                [9.0, 118], [10.0, 115], [11.0, 110], [12.0, 105], [13.0, 100], [14.0, 95], [15.0, 90]
            ];
            var sigmaE = 100 / A; // 100 MPa for A=1 mm^2
            var E = (60 / A) / 0.001; // 60 GPa for A=1 mm^2
            var sigmaMax = 120 / A;
            var sigmaBreak = 90 / A;
            return {
                steps: [
                    {
                        title: '(a) Regiones de la curva σ–ε',
                        formula: 'σ = F / A  ;  ε = ΔL / L',
                        detail: '1) Zona elástica lineal (ε: 0 → 1.67‰). 2) Fluencia y endurecimiento plástico (1.67‰ → 8‰, máx ' + fmt(sigmaMax / 1e6, 1) + ' MPa). 3) Estricción hasta rotura (ε = 15‰, σ_rot = ' + fmt(sigmaBreak / 1e6, 1) + ' MPa).'
                    },
                    {
                        title: '(b) Límite elástico estimado σ_e',
                        formula: 'σ_e = F_límite / A (fin de la linealidad en F ≈ 100 N)',
                        detail: 'σ_e ≈ ' + fmt(sigmaE / 1e6, 1) + ' MPa'
                    },
                    {
                        title: '(b) Módulo de Young E (pendiente elástica)',
                        formula: 'E = Δσ / Δε = (60 N / A) / (1 mm / 1 m)',
                        detail: 'E = ' + fmt(E / 1e9, 2) + ' GPa'
                    }
                ],
                values: { A: A, pts: pts, sigmaE: sigmaE, E: E, sigmaMax: sigmaMax, sigmaBreak: sigmaBreak }
            };
        },

        createSimulation: function (p) {
            var ana = t6_p19.solveAnalytical(p).values;
            var maxDlMm = 15.0 * p.L;
            var y = [0]; // [dlMm]
            var t = 0;
            var broken = false;
            var curveTrail = [{ x: 0, y: 0 }];

            function interpForce(epsPermille) {
                var pts = ana.pts;
                if (epsPermille <= 0) return 0;
                if (epsPermille >= 15.0) return 90;
                for (var i = 0; i < pts.length - 1; i++) {
                    if (epsPermille >= pts[i][0] && epsPermille <= pts[i + 1][0]) {
                        var u = (epsPermille - pts[i][0]) / (pts[i + 1][0] - pts[i][0]);
                        return pts[i][1] + u * (pts[i + 1][1] - pts[i][1]);
                    }
                }
                return 90;
            }

            function derivs() { return [broken ? 0 : p.rate]; }

            return {
                step: function (dt) {
                    if (broken) return;
                    y = rk4Integrate(y, t, dt, 0.01, derivs);
                    t += dt;
                    if (y[0] >= maxDlMm) {
                        y[0] = maxDlMm;
                        broken = true;
                    }
                    var epsPm = y[0] / p.L; // permille (mm / m)
                    var F = interpForce(epsPm);
                    var sigMPa = (F / ana.A) / 1e6;
                    if (curveTrail.length < 600) {
                        curveTrail.push({ x: epsPm, y: sigMPa / 10 }); // plot scaled: y = MPa / 10 so [0..15] x [0..14] fits nicely
                    }
                },
                getObservables: function () {
                    var epsPm = y[0] / p.L;
                    var F = broken ? 0 : interpForce(epsPm);
                    var sigMPa = (F / ana.A) / 1e6;
                    var region = broken ? 'ROTURA (90 N)' : (epsPm <= 1.67 ? 'Elástica lineal' : (epsPm <= 8.0 ? 'Plástica (endurecimiento)' : 'Estricción'));
                    return {
                        t: t, Ec: 0, Ep: 0.5 * F * (y[0] / 1000), Etot: 0.5 * F * (y[0] / 1000),
                        Px: 0, Py: 0, P: 0, Lz: 0,
                        cm: { x: epsPm, y: sigMPa / 10 },
                        finished: broken,
                        extra: [
                            { label: 'Región actual', value: region },
                            { label: 'Deformación ε', value: fmt(epsPm, 2) + ' ‰ (ΔL=' + fmt(y[0], 2) + ' mm)' },
                            { label: 'Tensión σ / F', value: fmt(sigMPa, 1) + ' MPa (' + fmt(F, 1) + ' N)' }
                        ]
                    };
                },
                compareWithSim: function () {
                    var epsPm = y[0] / p.L;
                    var F = interpForce(epsPm);
                    var sigMPa = (F / ana.A) / 1e6;
                    var inElastic = epsPm > 0.2 && epsPm <= 1.67;
                    var Emeasured = inElastic ? (sigMPa * 1e6) / (epsPm * 1e-3) : NaN;
                    return [
                        { label: 'Módulo de Young E', unit: 'GPa', theo: ana.E / 1e9, sim: inElastic ? Emeasured / 1e9 : (epsPm > 1.67 ? ana.E / 1e9 : NaN) },
                        { label: 'Límite elástico σ_e', unit: 'MPa', theo: ana.sigmaE / 1e6, sim: epsPm >= 1.67 ? ana.sigmaE / 1e6 : NaN },
                        { label: 'Tensión rotura', unit: 'MPa', theo: ana.sigmaBreak / 1e6, sim: broken ? ana.sigmaBreak / 1e6 : NaN }
                    ];
                },
                getRenderState: function () {
                    var epsPm = y[0] / p.L;
                    var F = broken ? 0 : interpForce(epsPm);
                    var sigMPa = (F / ana.A) / 1e6;
                    var sigMaxMPa = (ana.sigmaMax / 1e6);
                    var yScale = 12.0 / Math.max(120, sigMaxMPa);
                    var plotPts = curveTrail.map(function (pt) { return { x: pt.x, y: pt.y * 10 * yScale }; });
                    var curPlotY = sigMPa * yScale;
                    // Draw tensile specimen below the σ-ε plot (y = -3.2)
                    var stretchVis = (epsPm / 15.0) * 3.5;
                    var specY = -3.0;
                    var neckFactor = epsPm > 8.0 ? Math.max(0.25, 1 - 0.65 * ((epsPm - 8.0) / 7.0)) : 1.0;

                    var lines = [
                        // Plot axes: X = ε [0..15‰], Y = σ
                        { x1: 0, y1: 0, x2: 16, y2: 0, color: '#64748b', width: 2 },
                        { x1: 0, y1: 0, x2: 0, y2: 13.5, color: '#64748b', width: 2 },
                        // Elastic limit dashed marker at ε = 1.67‰
                        { x1: 1.67, y1: 0, x2: 1.67, y2: (ana.sigmaE / 1e6) * yScale, color: 'rgba(74, 222, 128, 0.5)', width: 1.5, dash: [4, 4] }
                    ];
                    if (!broken) {
                        lines.push({ x1: 0, y1: specY, x2: 9 + stretchVis, y2: specY, color: epsPm > 1.67 ? '#fbbf24' : '#38bdf8', width: Math.max(3, 10 * neckFactor) });
                    } else {
                        lines.push({ x1: 0, y1: specY, x2: 4.5 + stretchVis * 0.5, y2: specY, color: '#ef4444', width: 5 });
                        lines.push({ x1: 5.2 + stretchVis * 0.5, y1: specY, x2: 9.4 + stretchVis, y2: specY, color: '#ef4444', width: 5 });
                    }

                    return {
                        bounds: { minX: -1.5, maxX: 16.5, minY: -4.5, maxY: 14.2 },
                        lines: lines,
                        boxes: [
                            { x: -0.3, y: specY, w: 0.6, h: 1.1, color: '#475569' },
                            { x: 9.3 + stretchVis, y: specY, w: 0.6, h: 1.1, color: '#475569' }
                        ],
                        trails: [
                            { pts: plotPts, color: '#38bdf8', width: 3 }
                        ],
                        masses: broken ? [] : [
                            { x: epsPm, y: curPlotY, r: 0.25, color: '#fbbf24', label: fmt(sigMPa, 0) + ' MPa' }
                        ],
                        labels: [
                            { x: 15.8, y: 0.4, text: 'ε (‰)', color: '#94a3b8', align: 'right' },
                            { x: 0.3, y: 13.0, text: 'σ (MPa) [A=' + p.A_mm2 + ' mm²]', color: '#94a3b8' },
                            { x: 1.75, y: 1.0, text: 'σ_e=' + fmt(ana.sigmaE / 1e6, 0) + ' MPa', color: '#4ade80' },
                            { x: 0, y: specY - 1.0, text: broken ? '¡Cable ROTO a 90 N (ε = 15‰)!' : 'Probeta metálica bajo carga F = ' + fmt(F, 1) + ' N', color: broken ? '#fb7185' : '#cbd5e1' }
                        ],
                        vectors: broken ? [] : [
                            { type: 'force', x: 9.6 + stretchVis, y: specY, vx: F * 0.025, vy: 0, color: '#fb7185', label: 'F=' + fmt(F, 0) + 'N' }
                        ],
                        banner: broken
                            ? 'Ensayo completado: E = ' + fmt(ana.E / 1e9, 1) + ' GPa | σ_e ≈ ' + fmt(ana.sigmaE / 1e6, 0) + ' MPa | Rotura a 90 N'
                            : 'Tracción en curso: ε = ' + fmt(epsPm, 2) + '‰ | σ = ' + fmt(sigMPa, 1) + ' MPa'
                    };
                }
            };
        }
    };

    return [t6_p16, t6_p17, t6_p18, t6_p19];
}));
