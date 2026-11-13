/**
 * app.js — Controlador principal del Laboratorio Interactivo de Mecánica Clásica.
 */
(function () {
    'use strict';

    var Catalog = window.ProblemsCatalog;
    var RK4 = window.PhysicsRK4;
    var fmt = RK4.fmt;

    // DOM refs
    var listParticlesEl = document.getElementById('list-particles');
    var listRigidEl = document.getElementById('list-rigid');
    var canvas2d = document.getElementById('canvas-2d');
    var container3d = document.getElementById('container-3d');
    var vectorTogglesEl = document.getElementById('vector-toggles');

    var btnPlay = document.getElementById('btn-play');
    var btnStep = document.getElementById('btn-step');
    var btnReset = document.getElementById('btn-reset');
    var btnPreset = document.getElementById('btn-preset');
    var btnResetView = document.getElementById('btn-reset-view');
    var speedSelect = document.getElementById('speed-select');
    var timeDisplay = document.getElementById('time-display');
    var modeBadge = document.getElementById('mode-badge');

    var probCodeEl = document.getElementById('prob-code');
    var probTitleEl = document.getElementById('prob-title');
    var probStatementEl = document.getElementById('prob-statement');
    var probGoalEl = document.getElementById('prob-goal');
    var quickActionsCard = document.getElementById('quick-actions-card');
    var quickActionsEl = document.getElementById('quick-actions');
    var paramsContainerEl = document.getElementById('params-container');
    var telemetryGridEl = document.getElementById('telemetry-grid');
    var cmpTbodyEl = document.getElementById('cmp-tbody');
    var stepsContainerEl = document.getElementById('steps-container');
    var guideGoalEl = document.getElementById('guide-goal');
    var guideContainerEl = document.getElementById('guide-container');

    var chkVel = document.getElementById('chk-vel');
    var chkMom = document.getElementById('chk-mom');
    var chkForce = document.getElementById('chk-force');
    var chkTrails = document.getElementById('chk-trails');
    var chkCm = document.getElementById('chk-cm');

    // Renderers & Charts
    var r2d = new window.Renderer2D(canvas2d);
    var r3d = null; // lazy init when first 3D problem is opened

    var energyChart = new window.ConservationCharts.ChartSeries(
        document.getElementById('chart-energy'),
        'Energías (J)',
        [
            { key: 'Ec', label: 'Ec', color: '#38bdf8', width: 2 },
            { key: 'Ep', label: 'Ep', color: '#a78bfa', width: 2 },
            { key: 'Etot', label: 'Etot', color: '#fbbf24', width: 2.2, dash: [5, 3] }
        ]
    );

    var momentumChart = new window.ConservationCharts.ChartSeries(
        document.getElementById('chart-momentum'),
        'Momentos P, L',
        [
            { key: 'P', label: '|P|', color: '#fb7185', width: 2 },
            { key: 'Lz', label: 'Lz', color: '#4ade80', width: 2 }
        ]
    );

    // State
    var currentProb = Catalog.all[0];
    var currentParams = Catalog.getDefaultParams(currentProb);
    var sim = null;
    var playing = false;
    var lastFrameTs = null;
    var chartAccum = 0;

    function getRenderOptions() {
        return {
            showVelocity: chkVel.checked,
            showMomentum: chkMom.checked,
            showForce: chkForce.checked,
            showTrails: chkTrails.checked,
            showCM: chkCm.checked
        };
    }

    // Build Left Sidebar Curriculum
    function buildSidebar() {
        function renderGroup(list, container) {
            container.innerHTML = '';
            list.forEach(function (prob) {
                var btn = document.createElement('button');
                btn.className = 'problem-item' + (prob.id === currentProb.id ? ' active' : '');
                btn.dataset.id = prob.id;
                var is3d = prob.renderMode.indexOf('3d') === 0;
                btn.innerHTML =
                    '<div class="problem-item-top">' +
                    '<span class="lvl-tag">Nivel ' + prob.level + ' · ' + prob.code + '</span>' +
                    '<span class="dim-tag ' + (is3d ? 'is3d' : '') + '">' + (is3d ? '3D' : '2D') + '</span>' +
                    '</div>' +
                    '<div class="problem-item-title">' + prob.title + '</div>';
                btn.addEventListener('click', function () {
                    selectProblem(prob.id);
                });
                container.appendChild(btn);
            });
        }
        renderGroup(Catalog.particles, listParticlesEl);
        renderGroup(Catalog.rigid, listRigidEl);
    }

    function updateSidebarActive() {
        var items = document.querySelectorAll('.problem-item');
        for (var i = 0; i < items.length; i++) {
            items[i].classList.toggle('active', items[i].dataset.id === currentProb.id);
        }
    }

    function updateModeBadge() {
        var isDef = Catalog.isDefaultPreset(currentProb, currentParams);
        modeBadge.textContent = isDef ? 'Valores del problema' : 'Modo experimental';
        modeBadge.className = 'mode-badge ' + (isDef ? 'preset' : 'custom');
    }

    function renderParamsUI() {
        paramsContainerEl.innerHTML = '';
        currentProb.paramsSchema.forEach(function (schema) {
            var row = document.createElement('div');
            row.className = 'param-row';
            var val = currentParams[schema.key];
            row.innerHTML =
                '<div class="param-label">' +
                '<span>' + schema.label + '</span>' +
                '<strong id="pval-' + schema.key + '">' + fmt(val, schema.step < 0.1 ? 2 : 1) + (schema.unit ? ' ' + schema.unit : '') + '</strong>' +
                '</div>' +
                '<input type="range" id="pslider-' + schema.key + '" min="' + schema.min + '" max="' + schema.max + '" step="' + schema.step + '" value="' + val + '">';

            var slider = row.querySelector('input');
            slider.addEventListener('input', function () {
                currentParams[schema.key] = parseFloat(slider.value);
                var valEl = document.getElementById('pval-' + schema.key);
                valEl.textContent = fmt(currentParams[schema.key], schema.step < 0.1 ? 2 : 1) + (schema.unit ? ' ' + schema.unit : '');
                updateModeBadge();
                resetSimulation(false);
                renderAnalytical();
            });
            paramsContainerEl.appendChild(row);
        });

        // Quick actions
        if (currentProb.quickActions && currentProb.quickActions.length > 0) {
            quickActionsCard.style.display = 'block';
            quickActionsEl.innerHTML = '';
            currentProb.quickActions.forEach(function (qa) {
                var b = document.createElement('button');
                b.className = 'btn';
                b.style.fontSize = '11px';
                b.style.padding = '5px 9px';
                b.textContent = qa.label;
                b.addEventListener('click', function () {
                    qa.apply(currentParams);
                    renderParamsUI();
                    updateModeBadge();
                    resetSimulation(false);
                    renderAnalytical();
                });
                quickActionsEl.appendChild(b);
            });
        } else {
            quickActionsCard.style.display = 'none';
        }
    }

    function renderAnalytical() {
        var ana = currentProb.solveAnalytical(currentParams);
        stepsContainerEl.innerHTML = '';
        ana.steps.forEach(function (st) {
            var div = document.createElement('div');
            div.className = 'step-card';
            div.innerHTML =
                '<h4>' + st.title + '</h4>' +
                '<div class="step-formula">' + st.formula + '</div>' +
                '<div class="step-detail">' + st.detail + '</div>';
            stepsContainerEl.appendChild(div);
        });
    }

    function renderGuide() {
        guideGoalEl.innerHTML = '<strong>Objetivo:</strong> ' + currentProb.learningGoal;
        guideContainerEl.innerHTML = '';
        currentProb.guideFromZero.forEach(function (item) {
            var d = document.createElement('div');
            d.className = 'guide-item';
            d.innerHTML = item;
            guideContainerEl.appendChild(d);
        });
    }

    function updateTelemetryAndComparison() {
        if (!sim) return;
        var obs = sim.getObservables();
        timeDisplay.textContent = 't = ' + obs.t.toFixed(2) + ' s';

        var cells = [
            { label: 'E_cinética', value: fmt(obs.Ec, 2) + ' J' },
            { label: 'E_potencial', value: fmt(obs.Ep, 2) + ' J' },
            { label: '|P| lineal', value: fmt(obs.P, 2) + ' kg·m/s' },
            { label: 'L_z angular', value: fmt(obs.Lz, 2) + ' kg·m²/s' }
        ];
        if (obs.extra) {
            obs.extra.forEach(function (ex) { cells.push(ex); });
        }
        telemetryGridEl.innerHTML = cells.map(function (c) {
            return '<div class="tele-cell"><span>' + c.label + '</span><strong>' + c.value + '</strong></div>';
        }).join('');

        // Comparison table
        var rows = sim.compareWithSim();
        cmpTbodyEl.innerHTML = rows.map(function (r) {
            var theoStr = isFinite(r.theo) ? fmt(r.theo, 3) + (r.unit ? ' ' + r.unit : '') : '—';
            var simStr = isFinite(r.sim) ? fmt(r.sim, 3) : 'En curso...';
            var errHtml = '—';
            if (isFinite(r.theo) && isFinite(r.sim)) {
                var denom = Math.max(1e-6, Math.abs(r.theo));
                var pct = Math.abs(r.sim - r.theo) / denom * 100;
                if (Math.abs(r.theo) < 1e-6 && Math.abs(r.sim) < 1e-4) pct = 0;
                var cls = pct < 0.5 ? 'err-ok' : 'err-warn';
                errHtml = '<span class="' + cls + '">' + pct.toFixed(3) + '%</span>';
            }
            return '<tr><td>' + r.label + '</td><td>' + theoStr + '</td><td>' + simStr + '</td><td>' + errHtml + '</td></tr>';
        }).join('');
    }

    function ensureViewportMode() {
        var is3d = currentProb.renderMode.indexOf('3d') === 0;
        if (is3d) {
            canvas2d.style.display = 'none';
            container3d.style.display = 'block';
            vectorTogglesEl.style.display = 'none';
            if (!r3d) {
                r3d = new window.Renderer3D(container3d);
            }
            r3d.resize();
        } else {
            container3d.style.display = 'none';
            canvas2d.style.display = 'block';
            vectorTogglesEl.style.display = 'flex';
        }
    }

    function drawFrame() {
        if (!sim) return;
        var rs = sim.getRenderState();
        if (currentProb.renderMode.indexOf('3d') === 0) {
            if (r3d) r3d.render(rs);
        } else {
            r2d.render(rs, getRenderOptions());
        }
        energyChart.draw();
        momentumChart.draw();
    }

    function resetSimulation(keepPlaying) {
        playing = !!keepPlaying;
        btnPlay.textContent = playing ? '⏸ Pausar' : '▶ Iniciar';
        sim = currentProb.createSimulation(currentParams);
        energyChart.clear();
        momentumChart.clear();
        chartAccum = 0;

        var obs = sim.getObservables();
        energyChart.push(obs.t, { Ec: obs.Ec, Ep: obs.Ep, Etot: obs.Etot });
        momentumChart.push(obs.t, { P: obs.P, Lz: obs.Lz });

        updateTelemetryAndComparison();
        drawFrame();
    }

    function selectProblem(id) {
        currentProb = Catalog.getById(id);
        currentParams = Catalog.getDefaultParams(currentProb);
        updateSidebarActive();

        probCodeEl.textContent = 'Nivel ' + currentProb.level + ' · ' + currentProb.code;
        probTitleEl.textContent = currentProb.title;
        probStatementEl.textContent = currentProb.statement;
        probGoalEl.innerHTML = '<strong>Qué aprenderás:</strong> ' + currentProb.learningGoal;

        ensureViewportMode();
        r2d.resetView();
        if (r3d && currentProb.renderMode.indexOf('3d') === 0) {
            r3d.resetView(currentProb.renderMode);
        }

        // Smart default speed for long-flight projectile explosion (T5-P16 ~ 22s)
        speedSelect.value = (currentProb.id === 't5_p16') ? '2' : '1';

        updateModeBadge();
        renderParamsUI();
        renderAnalytical();
        renderGuide();
        resetSimulation(false);
    }

    // Main animation loop
    function loop(ts) {
        if (lastFrameTs === null) lastFrameTs = ts;
        var realDt = Math.min(0.05, (ts - lastFrameTs) / 1000);
        lastFrameTs = ts;

        if (playing && sim) {
            var speed = parseFloat(speedSelect.value) || 1;
            var simDt = realDt * speed;
            sim.step(simDt);
            var obs = sim.getObservables();

            chartAccum += realDt;
            if (chartAccum >= 0.03 || obs.finished) {
                chartAccum = 0;
                energyChart.push(obs.t, { Ec: obs.Ec, Ep: obs.Ep, Etot: obs.Etot });
                momentumChart.push(obs.t, { P: obs.P, Lz: obs.Lz });
            }

            updateTelemetryAndComparison();
            if (obs.finished) {
                playing = false;
                btnPlay.textContent = '▶ Iniciar';
            }
        }

        drawFrame();
        requestAnimationFrame(loop);
    }

    // Event bindings
    btnPlay.addEventListener('click', function () {
        if (sim && sim.getObservables().finished) {
            resetSimulation(true);
            return;
        }
        playing = !playing;
        btnPlay.textContent = playing ? '⏸ Pausar' : '▶ Iniciar';
    });

    btnStep.addEventListener('click', function () {
        playing = false;
        btnPlay.textContent = '▶ Iniciar';
        if (!sim) return;
        var speed = parseFloat(speedSelect.value) || 1;
        sim.step(0.05 * speed);
        var obs = sim.getObservables();
        energyChart.push(obs.t, { Ec: obs.Ec, Ep: obs.Ep, Etot: obs.Etot });
        momentumChart.push(obs.t, { P: obs.P, Lz: obs.Lz });
        updateTelemetryAndComparison();
        drawFrame();
    });

    btnReset.addEventListener('click', function () {
        resetSimulation(false);
    });

    btnPreset.addEventListener('click', function () {
        currentParams = Catalog.getDefaultParams(currentProb);
        updateModeBadge();
        renderParamsUI();
        renderAnalytical();
        resetSimulation(false);
    });

    btnResetView.addEventListener('click', function () {
        if (currentProb.renderMode.indexOf('3d') === 0 && r3d) {
            r3d.resetView(currentProb.renderMode);
        } else {
            r2d.resetView();
        }
        drawFrame();
    });

    // Tabs switching
    var tabBtns = document.querySelectorAll('.tab-btn');
    for (var i = 0; i < tabBtns.length; i++) {
        (function (btn) {
            btn.addEventListener('click', function () {
                var target = btn.dataset.tab;
                for (var j = 0; j < tabBtns.length; j++) tabBtns[j].classList.remove('active');
                btn.classList.add('active');
                var contents = document.querySelectorAll('.tab-content');
                for (var k = 0; k < contents.length; k++) contents[k].classList.remove('active');
                document.getElementById('tab-' + target).classList.add('active');
            });
        })(tabBtns[i]);
    }

    window.addEventListener('resize', function () {
        if (r3d) r3d.resize();
        drawFrame();
    });

    // Init
    buildSidebar();
    selectProblem('t5_p3');
    requestAnimationFrame(loop);
})();
