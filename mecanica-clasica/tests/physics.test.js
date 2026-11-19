/**
 * physics.test.js — Verificación exhaustiva de los 13 problemas (analítico vs RK4),
 * leyes de conservación y casos límite. Ejecutar con: node tests/physics.test.js
 */
'use strict';

var assert = require('assert');
var RK4 = require('../js/rk4.js');
var Catalog = require('../js/problems_catalog.js');

var passed = 0;
var failed = 0;

function test(name, fn) {
    try {
        fn();
        passed++;
        console.log('  [PASS] ' + name);
    } catch (err) {
        failed++;
        console.error('  [FAIL] ' + name + ': ' + err.message);
    }
}

function approx(a, b, tol, msg) {
    var diff = Math.abs(a - b);
    assert(isFinite(a) && isFinite(b) && diff <= tol,
        (msg || '') + ' esperado ' + b + ', obtenido ' + a + ' (diff=' + diff + ', tol=' + tol + ')');
}

function runSteps(sim, totalT, dt) {
    var steps = Math.round(totalT / dt);
    for (var i = 0; i < steps; i++) sim.step(dt);
}

console.log('=== Tests del integrador RK4 ===');

test('RK4 integra oscilador armónico simple con error < 1e-6', function () {
    // y'' = -y, y(0)=1, y'(0)=0 -> y(2*pi)=1, y'(2*pi)=0
    var y = [1, 0];
    var dt = 0.01;
    var T = 2 * Math.PI;
    var steps = 628;
    var t = 0;
    for (var i = 0; i < steps; i++) {
        y = RK4.rk4Step(y, t, dt, function (s) { return [s[1], -s[0]]; });
        t += dt;
    }
    var rem = T - t;
    y = RK4.rk4Step(y, t, rem, function (s) { return [s[1], -s[0]]; });
    approx(y[0], 1.0, 1e-6, 'cos(2pi)');
    approx(y[1], 0.0, 1e-6, '-sin(2pi)');
});

test('RK4 manejo de dt=0 y vector vacío', function () {
    assert.deepStrictEqual(RK4.rk4Step([], 0, 0.1, function () { return []; }), []);
    assert.deepStrictEqual(RK4.rk4Step([3, 4], 0, 0, function () { return [1, 1]; }), [3, 4]);
});

console.log('\n=== Tests Sistemas de Partículas (Problemas T5) ===');

test('T5-P3: CM bajo fuerzas externas coincide con analítico y conserva τ_interna nula', function () {
    var prob = Catalog.getById('t5_p3');
    var p = Catalog.getDefaultParams(prob);
    var sim = prob.createSimulation(p);
    runSteps(sim, 4.0, 0.01);
    var obs = sim.getObservables();
    // xcm(0) = 1.5, ax = 8/16 = 0.5 -> xcm(4) = 1.5 + 0.5*0.5*16 = 5.5
    // ycm(0) = 1.875, ay = 6/16 = 0.375 -> ycm(4) = 1.875 + 0.5*0.375*16 = 4.875
    approx(obs.cm.x, 5.5, 1e-6, 'x_CM(4s)');
    approx(obs.cm.y, 4.875, 1e-6, 'y_CM(4s)');
    approx(obs.Px, 32.0, 1e-6, 'Px(4s)');
    approx(obs.Py, 24.0, 1e-6, 'Py(4s)');
});

test('T5-P9: Biconservación en choque elástico (Caso 1, Caso 2 y Caso 3)', function () {
    var prob = Catalog.getById('t5_p9');
    var cases = [
        { m1: 1, m2: 2, f1: -1 / 3, f2: 2 / 3 },
        { m1: 2, m2: 1, f1: 1 / 3, f2: 4 / 3 },
        { m1: 1.5, m2: 1.5, f1: 0, f2: 1 }
    ];
    cases.forEach(function (c, idx) {
        var p = Catalog.getDefaultParams(prob);
        p.m1 = c.m1; p.m2 = c.m2; p.l = 1.5; p.theta0Deg = 60; p.e = 1.0;
        var ana = prob.solveAnalytical(p).values;
        var v1Expected = Math.sqrt(p.g * p.l);
        approx(ana.v1, v1Expected, 1e-9, 'v1 caso ' + (idx + 1));
        approx(ana.v1f, c.f1 * v1Expected, 1e-9, 'v1f caso ' + (idx + 1));
        approx(ana.v2f, c.f2 * v1Expected, 1e-9, 'v2f caso ' + (idx + 1));

        var sim = prob.createSimulation(p);
        var E0 = sim.getObservables().Etot;
        runSteps(sim, 1.2, 0.01); // crosses first collision around 0.65 s
        var obs = sim.getObservables();
        approx(obs.Etot, E0, 1e-4, 'Conservación E_tot en choque elástico caso ' + (idx + 1));
        var cmp = sim.compareWithSim();
        approx(cmp[0].sim, cmp[0].theo, 1e-3, 'v1 pre-choque RK4 caso ' + (idx + 1));
        approx(cmp[1].sim, cmp[1].theo, 1e-3, 'v1f post-choque RK4 caso ' + (idx + 1));
        approx(cmp[2].sim, cmp[2].theo, 1e-3, 'v2f post-choque RK4 caso ' + (idx + 1));
    });
});

test('T5-P6: Camión y coche choque 2D inelástico y frenado con μ=0.2', function () {
    var prob = Catalog.getById('t5_p6');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    approx(ana.V, Math.sqrt(200 * 200 + 40 * 40) / 11, 1e-6, 'V post-choque');
    approx(ana.thetaDeg, RK4.radToDeg(Math.atan(0.2)), 1e-6, 'ángulo');
    approx(ana.dist, (ana.V * ana.V) / (2 * 0.2 * 9.8), 1e-6, 'distancia frenado');

    var sim = prob.createSimulation(p);
    runSteps(sim, 14.0, 0.01);
    var obs = sim.getObservables();
    assert.strictEqual(obs.finished, true, 'Debe haberse detenido antes de 14 s');
    var cmp = sim.compareWithSim();
    approx(cmp[0].sim, cmp[0].theo, 1e-4, 'V post-choque sim');
    approx(cmp[1].sim, cmp[1].theo, 1e-2, 'distancia frenado RK4');
    approx(cmp[2].sim, cmp[2].theo, 1e-2, 'pérdida energía kJ');
});

test('T5-P12: Balón y bola de nieve en la cima -> altura en x=28 m', function () {
    var prob = Catalog.getById('t5_p12');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    approx(ana.yAtGoal, 2.6922, 1e-3, 'y en portería');
    assert.strictEqual(ana.isGoal, false, 'Pasa por encima de 2.4 m -> no gol');

    var sim = prob.createSimulation(p);
    runSteps(sim, 3.2, 0.005);
    var cmp = sim.compareWithSim();
    approx(cmp[0].sim, cmp[0].theo, 1e-4, 'x_cima');
    approx(cmp[2].sim, cmp[2].theo, 1e-4, 'Vx post-choque');
    approx(cmp[3].sim, cmp[3].theo, 1e-3, 'y en portería RK4');
});

test('T5-P16: Explosión de proyectil en cima -> CM sigue parábola intacta', function () {
    var prob = Catalog.getById('t5_p16');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    // v1x = 3 * 60 * sqrt(3) = 311.7691
    approx(ana.v1x, 180 * Math.sqrt(3), 1e-4, 'v1x');
    approx(ana.v2y, -33.3804, 1e-3, 'v2y');
    approx(ana.v1y, 66.7608, 1e-3, 'v1y');

    var sim = prob.createSimulation(p);
    // Run until t = tExp + 2.0 s (both fragments still in the air!)
    runSteps(sim, ana.tExp + 2.0, 0.01);
    var obsAir = sim.getObservables();
    var tAir = obsAir.t;
    var expectedCmX = (120 * Math.cos(Math.PI / 6)) * tAir;
    var expectedCmY = (120 * Math.sin(Math.PI / 6)) * tAir - 0.5 * 9.8 * tAir * tAir;
    approx(obsAir.cm.x, expectedCmX, 0.05, 'CM x en aire coincide con parábola sin explotar');
    approx(obsAir.cm.y, expectedCmY, 0.05, 'CM y en aire coincide con parábola sin explotar');

    // Finish simulation
    runSteps(sim, 16.0, 0.01);
    var cmp = sim.compareWithSim();
    approx(cmp[2].sim, cmp[2].theo, 0.5, 'Alcance fragmento 1');
});

test('T5-P15: Mancuerna libre captura 3ª masa -> Vcm=0, ωf=3/4ω, ΔEc=-25%', function () {
    var prob = Catalog.getById('t5_p15');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    approx(ana.omegaF, 0.75 * p.omega, 1e-9, 'omega_f = 3/4 omega');
    approx(ana.loss, 0.25 * ana.EcBefore, 1e-9, '25% pérdida Ec');

    var sim = prob.createSimulation(p);
    runSteps(sim, 2.0, 0.005);
    var obs = sim.getObservables();
    approx(obs.P, 0, 1e-9, 'Momento lineal total = 0');
    approx(obs.Lz, ana.Lcm, 1e-6, 'Conservación L_CM');
    approx(obs.cm.x, p.L / 6, 1e-9, 'x_CM = L/6');
});

console.log('\n=== Tests Sólido Rígido (Hoja 6) ===');

test('T6-P2: Cilindro macizo gana a hueco con rozamiento; empatan sin rozamiento', function () {
    var prob = Catalog.getById('t6_p2');
    var p = Catalog.getDefaultParams(prob);
    var anaRoll = prob.solveAnalytical(p).values;
    assert(anaRoll.tSolid < anaRoll.tHollow, 'Macizo debe llegar antes que hueco');
    approx(anaRoll.aSolid, (2 / 3) * 9.8 * 0.5, 1e-6, 'a_macizo con alpha=60° vertical');
    approx(anaRoll.aHollow, (1 / 2) * 9.8 * 0.5, 1e-6, 'a_hueco con alpha=60° vertical');

    var sim = prob.createSimulation(p);
    var E0 = sim.getObservables().Etot;
    runSteps(sim, 3.0, 0.005);
    var obs = sim.getObservables();
    assert.strictEqual(obs.finished, true);
    approx(obs.Etot, E0, 1e-3, 'Conservación E_tot en rodadura');
    var cmp = sim.compareWithSim();
    approx(cmp[0].sim, cmp[0].theo, 1e-3, 't_macizo RK4');
    approx(cmp[1].sim, cmp[1].theo, 1e-3, 't_hueco RK4');

    // Frictionless test
    p.withFriction = 0;
    var anaSlide = prob.solveAnalytical(p).values;
    approx(anaSlide.tSolid, anaSlide.tHollow, 1e-9, 'Empate sin rozamiento');
});

test('T6-P8: Cilindro con cuerda y bloque 8 kg -> T=38.08 N, I=0.17 kg·m², t=5.000 s', function () {
    var prob = Catalog.getById('t6_p8');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    approx(ana.a, 5.04, 1e-6, 'a = 5.04 m/s²');
    approx(ana.T, 38.08, 1e-6, 'T = 38.08 N');
    approx(ana.tFinal, 5.0, 1e-6, 't = 5.0 s');
    approx(ana.EcBlock + ana.EcCyl, ana.dEp, 1e-6, 'Conservación energía exacta');

    var sim = prob.createSimulation(p);
    runSteps(sim, 5.5, 0.005);
    var obs = sim.getObservables();
    assert.strictEqual(obs.finished, true);
    var cmp = sim.compareWithSim();
    approx(cmp[0].sim, 5.0, 1e-3, 't_descenso RK4');
    approx(cmp[1].sim, 2540.16, 0.1, 'Ec_bloque');
    approx(cmp[2].sim, 2399.04, 0.1, 'Ec_cilindro');
});

test('T6-P3: Plataforma encogiendo brazos -> L constante, ω_f = 41/26 rev/s, Ec sube', function () {
    var prob = Catalog.getById('t6_p3');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    approx(ana.I0, 9.84, 1e-6, 'I0 = 9.84');
    approx(ana.If, 6.24, 1e-6, 'If = 6.24');
    approx(ana.revF, 41 / 26, 1e-6, 'rev_f = 41/26 rev/s');
    assert(ana.Ecf > ana.Ec0, 'Ec debe aumentar al encoger brazos');

    var sim = prob.createSimulation(p);
    var L0 = sim.getObservables().Lz;
    runSteps(sim, 4.5, 0.01);
    var obs = sim.getObservables();
    approx(obs.Lz, L0, 1e-6, 'L conservado durante retracción');
    var cmp = sim.compareWithSim();
    approx(cmp[1].sim, ana.wf, 1e-5, 'wf simulada');
});

test('T6-P4: Plataforma lanzamiento + caminata -> ωa=0.10 rad/s, ωb=1/6 rad/s, L_total=0', function () {
    var prob = Catalog.getById('t6_p4');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    approx(ana.wa, 0.10, 1e-9, 'wa = 0.10 rad/s');
    approx(ana.wb, 1 / 6, 1e-9, 'wb = 1/6 rad/s');

    var sim = prob.createSimulation(p);
    runSteps(sim, 6.0, 0.01);
    var obs = sim.getObservables();
    approx(obs.Lz, 0, 1e-6, 'L_total = 0 en todo momento');
    var cmp = sim.compareWithSim();
    approx(cmp[1].sim, ana.wb, 1e-6, 'wb simulada');
});

test('T6-P11: Fuerza a altura r -> inversión f_r en r=R/2 y caso r=R', function () {
    var prob = Catalog.getById('t6_p11');
    var p = Catalog.getDefaultParams(prob); // rRatio = 1.0
    var anaR = prob.solveAnalytical(p).values;
    approx(anaR.aRoll, (4 * p.F) / (3 * p.M), 1e-6, 'a(r=R) = 4F/(3M)');
    approx(anaR.frRoll, p.F / 3, 1e-6, 'fr(r=R) = +F/3');

    p.rRatio = 0.5;
    approx(prob.solveAnalytical(p).values.frRoll, 0, 1e-9, 'fr(r=R/2) = 0');

    p.rRatio = 0.0;
    approx(prob.solveAnalytical(p).values.frRoll, -p.F / 3, 1e-9, 'fr(r=0) = -F/3');

    // Test slipping regime when mu_e is very low
    p.rRatio = 1.0; p.mu_e = 0.02;
    var simSlip = prob.createSimulation(p);
    runSteps(simSlip, 1.0, 0.01);
    var vc = parseFloat(simSlip.getObservables().extra[2].value);
    assert(Math.abs(vc) > 0.01, 'Debe haber velocidad de deslizamiento cuando μ_e es insuficiente');
});

test('T6-P10: Choque masa-barra libre -> ω=V/L, Vcm=V/3, CIR en x=-L/6', function () {
    var prob = Catalog.getById('t6_p10');
    var p = Catalog.getDefaultParams(prob);
    var ana = prob.solveAnalytical(p).values;
    approx(ana.Vcm, p.V / 3, 1e-9, 'V_CM = V/3');
    approx(ana.w, p.V / p.L, 1e-9, 'ω = V/L');
    approx(ana.xCir, -p.L / 6, 1e-9, 'CIR en -L/6');
    approx(ana.loss, (1 / 6) * p.m * p.V * p.V, 1e-9, 'ΔEc = -1/6 m V^2');

    var sim = prob.createSimulation(p);
    runSteps(sim, 2.0, 0.005);
    var obs = sim.getObservables();
    approx(obs.P, p.m * p.V, 1e-6, 'Conservación P');
    approx(obs.Lz, (p.L / 3) * p.m * p.V, 1e-6, 'Conservación L_CM');
});

test('T6-P1: Varilla inclinada 3D -> I_eje, L_O no paralelo a ω, principal en α=90°', function () {
    var prob = Catalog.getById('t6_p1');
    var p = Catalog.getDefaultParams(prob); // alpha = 35°
    var ana = prob.solveAnalytical(p).values;
    var sinA = Math.sin(RK4.degToRad(35)), cosA = Math.cos(RK4.degToRad(35));
    var Irod = (1 / 3) * p.m * p.a * p.a;
    approx(ana.Ieje, Irod * sinA * sinA, 1e-9, 'I_eje');
    approx(ana.Lmag, Irod * p.omega * sinA, 1e-9, '|L_O|');
    approx(ana.tauMag, Irod * p.omega * p.omega * sinA * cosA, 1e-9, '|τ|');
    assert.strictEqual(ana.isPrincipal, false, 'α=35° NO es eje principal');

    var sim = prob.createSimulation(p);
    runSteps(sim, 1.5, 0.01);
    var rs = sim.getRenderState();
    // L_O debe ser perpendicular a u siempre (L . u == 0)
    var dotLU = rs.L[0] * rs.u[0] + rs.L[1] * rs.u[1] + rs.L[2] * rs.u[2];
    approx(dotLU, 0, 1e-9, 'L_O siempre perpendicular a la varilla u');

    p.alphaDeg = 90;
    var ana90 = prob.solveAnalytical(p).values;
    assert.strictEqual(ana90.isPrincipal, true, 'α=90° SÍ es eje principal');
    approx(ana90.tauMag, 0, 1e-9, 'τ = 0 en α=90°');
});

console.log('\n=== Tests de robustez y renderState (todos los 35 problemas y extremos) ===');

test('Todos los 35 problemas generan analítico, observables, comparativa y renderState sin NaN en t=0..T', function () {
    assert.strictEqual(Catalog.all.length, 35, 'Catálogo debe tener 35 simulaciones');
    Catalog.all.forEach(function (prob) {
        var defP = Catalog.getDefaultParams(prob);
        var ana = prob.solveAnalytical(defP);
        assert(ana && Array.isArray(ana.steps) && ana.steps.length >= 2, prob.id + ' debe tener pasos');

        var sim = prob.createSimulation(defP);
        for (var step = 0; step <= 60; step++) {
            var obs = sim.getObservables();
            assert(isFinite(obs.t) && isFinite(obs.Ec) && isFinite(obs.Ep) && isFinite(obs.Etot), prob.id + ' observables finitos');
            var rs = sim.getRenderState();
            assert(rs && typeof rs.banner === 'string', prob.id + ' renderState válido');
            if (rs.masses) {
                rs.masses.forEach(function (m) {
                    assert(isFinite(m.x) && isFinite(m.y) && (m.r === undefined || isFinite(m.r)), prob.id + ' masa finita');
                });
            }
            if (rs.wheels) {
                rs.wheels.forEach(function (w) {
                    assert(isFinite(w.x) && isFinite(w.y) && isFinite(w.phi), prob.id + ' rueda finita');
                });
            }
            var cmp = sim.compareWithSim();
            assert(Array.isArray(cmp) && cmp.length >= 2, prob.id + ' compareWithSim válido');
            sim.step(0.1);
        }

        // Test extreme min & max parameter bounds
        var minP = {};
        var maxP = {};
        prob.paramsSchema.forEach(function (s) {
            minP[s.key] = s.min;
            maxP[s.key] = s.max;
        });
        [minP, maxP].forEach(function (ep, idx) {
            var eAna = prob.solveAnalytical(ep);
            assert(eAna && eAna.steps.length >= 2, prob.id + ' extremo ' + idx);
            var eSim = prob.createSimulation(ep);
            eSim.step(0.05);
            var eObs = eSim.getObservables();
            assert(isFinite(eObs.Ec) && isFinite(eObs.Etot), prob.id + ' extremo ' + idx + ' finito');
        });
    });
});

console.log('\n=== Tests de regresión y casos extremos ===');

test('T5-P9 extremo (m1>>m2, θ0=85°): detecta múltiples choques incluso tras vuelta completa', function () {
    var prob = Catalog.getById('t5_p9');
    var p = Catalog.getDefaultParams(prob);
    p.m1 = 10; p.m2 = 0.5; p.theta0Deg = 85; p.e = 1.0;
    var sim = prob.createSimulation(p);
    var E0 = sim.getObservables().Etot;
    runSteps(sim, 4.0, 0.01);
    var obs = sim.getObservables();
    var collisions = parseInt(obs.extra[2].value, 10);
    assert(collisions >= 2, 'Debe detectar colisión tras vuelta completa (choques=' + collisions + ')');
    approx(obs.Etot, E0, 1e-4, 'Energía conservada tras múltiples choques');
});

test('T5-P12 casos extremos: Vx<0 mantiene bola en bounds y usa x_cima/y_cima RK4 reales', function () {
    var prob = Catalog.getById('t5_p12');
    var p = Catalog.getDefaultParams(prob);
    p.v0 = 8; p.alphaDeg = 75; p.mBall = 0.1; p.mSnow = 1.5; p.vSnow = 25; p.betaDeg = 0;
    var sim = prob.createSimulation(p);
    runSteps(sim, 2.5, 0.005);
    var obs = sim.getObservables();
    var rs = sim.getRenderState();
    assert(rs.bounds.minX <= obs.cm.x, 'minX debe incluir balón retrocediendo (' + rs.bounds.minX + ' <= ' + obs.cm.x + ')');
    assert(obs.extra[1].value !== 'Pendiente', 'Tras aterrizar sin llegar a portería no debe quedar Pendiente');
});

test('T6-P8 signo de giro de rueda horario coherente con cuerda desenrollándose a la derecha', function () {
    var prob = Catalog.getById('t6_p8');
    var p = Catalog.getDefaultParams(prob);
    var sim = prob.createSimulation(p);
    runSteps(sim, 1.0, 0.01);
    var rs = sim.getRenderState();
    assert(rs.wheels[0].phi > 0, 'phi debe ser positivo (giro horario en Canvas 2D) al desenrollarse a la derecha');
});

test('T6-P3 banner correcto cuando r0 < rf (extensión de brazos)', function () {
    var prob = Catalog.getById('t6_p3');
    var p = Catalog.getDefaultParams(prob);
    p.r0 = 0.3; p.rf = 0.8; p.pullDuration = 2.0;
    var sim = prob.createSimulation(p);
    runSteps(sim, 2.5, 0.01);
    var rs = sim.getRenderState();
    assert(rs.banner.indexOf('Extendiendo') >= 0, 'Banner debe indicar extensión cuando r0 < rf: ' + rs.banner);
});

test('Auditoría general: bounds negativos T5-P3, coche dominante T5-P6, separación T6-P2, suelo T6-P8 y CM T6-P10', function () {
    // T5-P3 negative forces
    var p3 = Catalog.getById('t5_p3');
    var par3 = Catalog.getDefaultParams(p3);
    par3.F1x = -20; par3.F2y = -20;
    var s3 = p3.createSimulation(par3);
    runSteps(s3, 3.0, 0.02);
    var rs3 = s3.getRenderState();
    rs3.masses.forEach(function (m) {
        assert(m.x >= rs3.bounds.minX && m.y >= rs3.bounds.minY, 'T5-P3 masas dentro de minX/minY con F < 0');
    });

    // T5-P6 car-dominated crash
    var p6 = Catalog.getById('t5_p6');
    var par6 = Catalog.getDefaultParams(p6);
    par6.M = 2000; par6.vM_kmh = 10; par6.m = 5000; par6.vm_kmh = 200;
    var s6 = p6.createSimulation(par6);
    runSteps(s6, 8.0, 0.02);
    var rs6 = s6.getRenderState();
    assert(rs6.masses[0].y <= rs6.bounds.maxY, 'T5-P6 conjunto dentro de maxY cuando Py >> Px');

    // T6-P2 steep angle perpendicular separation
    var p2 = Catalog.getById('t6_p2');
    var par2 = Catalog.getDefaultParams(p2);
    par2.alphaDeg = 20; par2.R = 0.6;
    var rs2 = p2.createSimulation(par2).getRenderState();
    var w0 = rs2.wheels[0], w1 = rs2.wheels[1];
    assert(Math.hypot(w1.x - w0.x, w1.y - w0.y) >= 2.5 * par2.R, 'T6-P2 cilindros no solapan en ángulo agudo');

    // T6-P8 block rests flush on ground at finish
    var p8 = Catalog.getById('t6_p8');
    var s8 = p8.createSimulation(Catalog.getDefaultParams(p8));
    runSteps(s8, 5.5, 0.01);
    var rs8 = s8.getRenderState();
    var boxBottom = rs8.boxes[0].y - 0.5 * rs8.boxes[0].h;
    approx(boxBottom, 0.3, 1e-6, 'T6-P8 base del bloque al ras del suelo y=0.3');

    // T6-P10 CM trail continuous from t=0 and V_CM = V/3 pre-collision
    var p10 = Catalog.getById('t6_p10');
    var par10 = Catalog.getDefaultParams(p10);
    var s10 = p10.createSimulation(par10);
    assert(s10.getObservables().extra[0].value.indexOf('1.000') >= 0, 'T6-P10 V_CM ya es V/3=1.000 antes del choque');
    var cm0y = s10.getRenderState().trails[0].pts[0].y;
    s10.step(0.05);
    var cm1y = s10.getRenderState().trails[0].pts[1].y;
    assert(cm1y > cm0y, 'T6-P10 trail CM sube monótonamente desde t=0');
});

console.log('\n=== Tests de Fundamentos (T1-T4), Nuevos T5, Nuevos T6 y Elasticidad ===');

test('Fundamentos F1-F4: Tiro parabólico, Plano inclinado, Oscilador M.A.S. y Órbita Kepler', function () {
    // F1
    var f1 = Catalog.getById('f_p1');
    var s1 = f1.createSimulation(Catalog.getDefaultParams(f1));
    runSteps(s1, 4.5, 0.01);
    var c1 = s1.compareWithSim();
    approx(c1[2].sim, (25 * 25) / 9.8, 0.05, 'F1 alcance 45 deg');

    // F2
    var f2 = Catalog.getById('f_p2');
    var p2 = Catalog.getDefaultParams(f2);
    var s2 = f2.createSimulation(p2);
    runSteps(s2, 3.0, 0.01);
    assert.strictEqual(s2.getObservables().finished, true, 'F2 desliza hasta la base');

    // F3
    var f3 = Catalog.getById('f_p3');
    var p3 = Catalog.getDefaultParams(f3);
    var s3 = f3.createSimulation(p3);
    var E30 = s3.getObservables().Etot;
    runSteps(s3, 4.0, 0.01);
    approx(s3.getObservables().Etot, E30, 1e-4, 'F3 conserva energía en M.A.S.');

    // F4
    var f4 = Catalog.getById('f_p4');
    var p4 = Catalog.getDefaultParams(f4);
    var s4 = f4.createSimulation(p4);
    var L40 = s4.getObservables().Lz;
    var E40 = s4.getObservables().Etot;
    runSteps(s4, 3.0, 0.01);
    approx(s4.getObservables().Lz, L40, 1e-4, 'F4 conserva momento angular');
    approx(s4.getObservables().Etot, E40, 1e-3, 'F4 conserva energía orbital');
});

test('Nuevos Sistemas de Partículas: T5-P1, T5-P2, T5-P5, T5-P8, T5-P10, T5-P11, T5-P14', function () {
    var p1 = Catalog.getById('t5_p1');
    var a1 = p1.solveAnalytical(Catalog.getDefaultParams(p1)).values;
    approx(a1.a12x, 10 / 6, 1e-9, 'T5-P1 a_CM(1,2)');

    var p2 = Catalog.getById('t5_p2');
    var a2 = p2.solveAnalytical(Catalog.getDefaultParams(p2)).values;
    approx(a2.dcm, Math.cos(RK4.degToRad(75)) / 14, 1e-6, 'T5-P2 |R_CM|');

    var p5 = Catalog.getById('t5_p5');
    var a5 = p5.solveAnalytical(Catalog.getDefaultParams(p5)).values;
    approx(a5.yEmpty, 1.7, 1e-6, 'T5-P5 vaso vacío y_CM=1.7 cm');
    approx(a5.yFull, 2.7069, 1e-3, 'T5-P5 vaso 3/4 y_CM');

    var p8 = Catalog.getById('t5_p8');
    var s8 = p8.createSimulation(Catalog.getDefaultParams(p8));
    runSteps(s8, 4.5, 0.01);
    approx(s8.compareWithSim()[2].sim, s8.compareWithSim()[2].theo, 0.1, 'T5-P8 h1 nueva');

    var p10 = Catalog.getById('t5_p10');
    var s10 = p10.createSimulation(Catalog.getDefaultParams(p10));
    runSteps(s10, 1.2, 0.01);
    approx(s10.compareWithSim()[0].sim, 18.284, 0.05, 'T5-P10 alcance manzana');

    var p11 = Catalog.getById('t5_p11');
    var s11 = p11.createSimulation(Catalog.getDefaultParams(p11));
    runSteps(s11, 4.0, 0.01);
    approx(s11.compareWithSim()[0].sim, 0.25, 1e-3, 'T5-P11 h_f=0.25 m');
    approx(s11.compareWithSim()[1].sim, 29.4, 0.1, 'T5-P11 pérdida=29.4 J');

    var p14 = Catalog.getById('t5_p14');
    var a14 = p14.solveAnalytical(Catalog.getDefaultParams(p14)).values;
    approx(a14.vc2, Math.sqrt(181), 1e-6, 'T5-P14 velocidad cuervo');
});

test('Nuevos Sólido Rígido y Elasticidad: T6-P5, P6, P9, P12, P13, P14, P15, P16, P17, P18, P19', function () {
    approx(Catalog.getById('t6_p5').solveAnalytical(Catalog.getDefaultParams(Catalog.getById('t6_p5'))).values.xEndMan, 20 / 11, 1e-6, 'T6-P5 distancia orilla');
    approx(Catalog.getById('t6_p6').solveAnalytical(Catalog.getDefaultParams(Catalog.getById('t6_p6'))).values.Fmin, 9.8 * Math.sqrt(0.05) / 0.2, 1e-5, 'T6-P6 F_min escalón');

    var p9 = Catalog.getById('t6_p9');
    var s9 = p9.createSimulation(Catalog.getDefaultParams(p9));
    runSteps(s9, 1.5, 0.01);
    approx(s9.getObservables().Lz, 0, 1e-6, 'T6-P9 L_total=0');
    approx(s9.compareWithSim()[0].sim, -2 * Math.PI / 3, 1e-4, 'T6-P9 omega_p');

    var p12 = Catalog.getById('t6_p12');
    var s12 = p12.createSimulation(Catalog.getDefaultParams(p12));
    runSteps(s12, 2.0, 0.01);
    approx(s12.compareWithSim()[0].sim, 6.0, 1e-4, 'T6-P12 L_O conservado');

    var p13 = Catalog.getById('t6_p13');
    var s13 = p13.createSimulation(Catalog.getDefaultParams(p13));
    runSteps(s13, 7.0, 0.01);
    approx(s13.compareWithSim()[2].sim, 1.25, 1e-4, 'T6-P13 disco mantiene omega2=1.25 tras soltar m');

    approx(Catalog.getById('t6_p14').solveAnalytical(Catalog.getDefaultParams(Catalog.getById('t6_p14'))).values.muCrit, Math.SQRT2 - 1, 1e-6, 'T6-P14 mu=sqrt(2)-1');
    approx(Catalog.getById('t6_p15').solveAnalytical(Catalog.getDefaultParams(Catalog.getById('t6_p15'))).values.safeAbs, true, 0, 'T6-P15 ABS frena a tiempo');

    // Elasticidad P16-P19
    var p16 = Catalog.getById('t6_p16');
    var par16 = Catalog.getDefaultParams(p16);
    var a16 = p16.solveAnalytical(par16).values;
    assert.strictEqual(a16.isElastic, false, 'T6-P16 acero supera 250 MPa');
    par16.material = 1;
    assert.strictEqual(p16.solveAnalytical(par16).values.isElastic, true, 'T6-P16 aluminio sí trabaja en régimen elástico');

    var p17 = Catalog.getById('t6_p17');
    var a17 = p17.solveAnalytical(Catalog.getDefaultParams(p17)).values;
    approx(a17.dTarget4sMm, 1.164, 1e-2, 'T6-P17 diámetro hilo 1.164 mm');

    var p18 = Catalog.getById('t6_p18');
    var a18 = p18.solveAnalytical(Catalog.getDefaultParams(p18)).values;
    approx(a18.dP / 1e6, 184.5, 1e-2, 'T6-P18 ΔP = 184.5 MPa');
    assert.strictEqual(a18.fractures, true, 'T6-P18 fractura el granito');

    var p19 = Catalog.getById('t6_p19');
    var a19 = p19.solveAnalytical(Catalog.getDefaultParams(p19)).values;
    approx(a19.E / 1e9, 60.0, 1e-6, 'T6-P19 E = 60 GPa');
    approx(a19.sigmaE / 1e6, 100.0, 1e-6, 'T6-P19 sigma_e = 100 MPa');
});

console.log('\n=========================================');
console.log('Resumen: ' + passed + ' PASSED, ' + failed + ' FAILED');
console.log('=========================================');
if (failed > 0) process.exit(1);
