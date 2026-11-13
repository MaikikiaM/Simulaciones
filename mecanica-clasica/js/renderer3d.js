/**
 * renderer3d.js — Motor 3D Three.js para los problemas tridimensionales de Sólido Rígido:
 *  - T6·P1: Varilla inclinada girando (L_O no paralelo a ω, cono de precesión y par τ)
 *  - T6·P3: Plataforma giratoria encogiendo brazos (L constante, ω y E_c crecientes)
 *  - T6·P4: Plataforma con lanzamiento tangencial y caminata radial
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory();
    } else {
        root.Renderer3D = factory();
    }
}(typeof self !== 'undefined' ? self : this, function () {
    'use strict';

    function Renderer3D(container) {
        this.container = container;
        this.ready = false;
        this.currentSceneKey = '';
        this.bannerEl = null;
        this.legendEl = null;

        if (typeof THREE === 'undefined') {
            this._showError('No se pudo cargar Three.js.');
            return;
        }

        try {
            this.scene = new THREE.Scene();
            this.scene.background = new THREE.Color(0x070b14);

            var rect = container.getBoundingClientRect();
            var w = Math.max(300, rect.width || 600);
            var h = Math.max(200, rect.height || 400);

            this.camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 200);
            this.camera.position.set(3.2, 2.3, 3.6);

            this.renderer = new THREE.WebGLRenderer({ antialias: true });
            this.renderer.setSize(w, h);
            this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
            container.innerHTML = '';
            container.appendChild(this.renderer.domElement);

            // Overlay HUD banner + legend inside 3D container
            this.bannerEl = document.createElement('div');
            this.bannerEl.className = 'hud-3d-banner';
            container.appendChild(this.bannerEl);

            this.legendEl = document.createElement('div');
            this.legendEl.className = 'hud-3d-legend';
            container.appendChild(this.legendEl);

            this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
            this.controls.enableDamping = true;
            this.controls.dampingFactor = 0.08;

            // Lights & grid
            var amb = new THREE.AmbientLight(0xffffff, 0.65);
            this.scene.add(amb);
            var dir1 = new THREE.DirectionalLight(0xffffff, 0.85);
            dir1.position.set(5, 8, 6);
            this.scene.add(dir1);
            var dir2 = new THREE.DirectionalLight(0x38bdf8, 0.35);
            dir2.position.set(-5, -3, -4);
            this.scene.add(dir2);

            this.grid = new THREE.GridHelper(12, 16, 0x334155, 0x1e293b);
            this.grid.position.y = -1.3;
            this.scene.add(this.grid);

            this.rootGroup = new THREE.Group();
            this.scene.add(this.rootGroup);
            this.ready = true;
        } catch (err) {
            this._showError('Error inicializando WebGL 3D: ' + err.message);
        }
    }

    Renderer3D.prototype._showError = function (msg) {
        this.container.innerHTML = '<div style="padding:24px;color:#fca5a5;">' + msg + '</div>';
    };

    Renderer3D.prototype.resize = function () {
        if (!this.ready) return;
        var rect = this.container.getBoundingClientRect();
        if (rect.width < 10 || rect.height < 10) return;
        this.camera.aspect = rect.width / rect.height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(rect.width, rect.height);
    };

    Renderer3D.prototype.resetView = function (mode) {
        if (!this.ready) return;
        if (mode === '3d-p4') {
            this.camera.position.set(0, 10.5, 12.5);
            this.controls.target.set(0, 0, -1.0);
        } else if (mode === '3d-p3') {
            this.camera.position.set(2.9, 2.35, 3.6);
            this.controls.target.set(0, 0.95, 0);
        } else {
            this.camera.position.set(2.8, 2.0, 3.0);
            this.controls.target.set(0, 0, 0);
        }
        this.controls.update();
    };

    Renderer3D.prototype._clearRoot = function () {
        while (this.rootGroup.children.length > 0) {
            this.rootGroup.remove(this.rootGroup.children[0]);
        }
        this.objs = {};
    };

    Renderer3D.prototype._buildP1 = function () {
        this._clearRoot();
        this.grid.position.y = -1.5;
        this.resetView('3d-p1');

        // Vertical rigid shaft (semi-transparent so omega vector inside/alongside is clearly visible) + top/bottom bearings
        var shaftGeo = new THREE.CylinderGeometry(0.022, 0.022, 2.8, 16);
        var shaftMat = new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.5, roughness: 0.4, transparent: true, opacity: 0.5 });
        var shaft = new THREE.Mesh(shaftGeo, shaftMat);
        this.rootGroup.add(shaft);

        var bearingGeo = new THREE.CylinderGeometry(0.16, 0.16, 0.12, 24);
        var bearingMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.5, roughness: 0.4 });
        var bTop = new THREE.Mesh(bearingGeo, bearingMat); bTop.position.y = 1.38;
        var bBot = new THREE.Mesh(bearingGeo, bearingMat); bBot.position.y = -1.38;
        this.rootGroup.add(bTop, bBot);

        // Tilted rod (oriented along Y in local group; rodMesh scaled, spheres stay spherical)
        var rodGroup = new THREE.Group();
        var rodGeo = new THREE.CylinderGeometry(0.052, 0.052, 2.0, 20);
        var rodMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, metalness: 0.35, roughness: 0.3 });
        var rodMesh = new THREE.Mesh(rodGeo, rodMat);
        rodGroup.add(rodMesh);

        var capGeo = new THREE.SphereGeometry(0.085, 16, 16);
        var capMat = new THREE.MeshStandardMaterial({ color: 0xfb7185, emissive: 0x4c0519 });
        var c1 = new THREE.Mesh(capGeo, capMat); c1.position.y = 1.0;
        var c2 = new THREE.Mesh(capGeo, capMat); c2.position.y = -1.0;
        rodGroup.add(c1, c2);
        this.rootGroup.add(rodGroup);

        // Center O marker
        var oMesh = new THREE.Mesh(new THREE.SphereGeometry(0.075, 16, 16), new THREE.MeshBasicMaterial({ color: 0xffffff }));
        this.rootGroup.add(oMesh);

        // Arrows: omega (cyan, slightly offset in +X/Z or starting cleanly above center), L_O (gold), tau (magenta)
        var arrowOmega = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0.06, 0.05, 0.06), 1.25, 0x22d3ee, 0.24, 0.13);
        var arrowL = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1.3, 0xfbbf24, 0.24, 0.13);
        var arrowTau = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 1.1, 0xf472b6, 0.22, 0.11);
        this.rootGroup.add(arrowOmega, arrowL, arrowTau);

        // Precession cone ring for tip of L_O
        var ringGeo = new THREE.RingGeometry(0.98, 1.02, 64);
        var ringMat = new THREE.MeshBasicMaterial({ color: 0xfbbf24, side: THREE.DoubleSide, transparent: true, opacity: 0.45 });
        var ring = new THREE.Mesh(ringGeo, ringMat);
        ring.rotation.x = Math.PI / 2;
        this.rootGroup.add(ring);

        this.objs = { rodGroup: rodGroup, rodMesh: rodMesh, c1: c1, c2: c2, arrowOmega: arrowOmega, arrowL: arrowL, arrowTau: arrowTau, ring: ring };
        this.legendEl.innerHTML =
            '<span style="color:#22d3ee">■ ω (eje giro)</span>' +
            '<span style="color:#fbbf24">■ L_O (precesa en cono)</span>' +
            '<span style="color:#f472b6">■ τ = dL_O/dt (cojinetes)</span>';
    };

    Renderer3D.prototype._buildP3 = function () {
        this._clearRoot();
        this.grid.position.y = -0.15;
        this.resetView('3d-p3');

        var platGroup = new THREE.Group();
        var discGeo = new THREE.CylinderGeometry(1.1, 1.1, 0.12, 48);
        var discMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.4, roughness: 0.5 });
        var disc = new THREE.Mesh(discGeo, discMat);
        disc.position.y = -0.06;
        platGroup.add(disc);

        // Reference stripe on turntable so rotation is obvious
        var stripe = new THREE.Mesh(
            new THREE.BoxGeometry(2.15, 0.02, 0.08),
            new THREE.MeshBasicMaterial({ color: 0x38bdf8 })
        );
        stripe.position.y = 0.01;
        platGroup.add(stripe);

        // Stylized person torso + head
        var bodyMat = new THREE.MeshStandardMaterial({ color: 0x60a5fa, roughness: 0.4 });
        var torso = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.15, 1.15, 18), bodyMat);
        torso.position.y = 0.6;
        var head = new THREE.Mesh(new THREE.SphereGeometry(0.16, 18, 18), bodyMat);
        head.position.y = 1.35;
        platGroup.add(torso, head);

        // Arms + weights
        var armMat = new THREE.MeshStandardMaterial({ color: 0x93c5fd, roughness: 0.4 });
        var armL = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 1.0, 12), armMat);
        armL.rotation.z = Math.PI / 2;
        armL.position.y = 1.02;
        var weightMat = new THREE.MeshStandardMaterial({ color: 0xfb7185, emissive: 0x4c0519, metalness: 0.3 });
        var w1 = new THREE.Mesh(new THREE.SphereGeometry(0.12, 18, 18), weightMat);
        var w2 = new THREE.Mesh(new THREE.SphereGeometry(0.12, 18, 18), weightMat);
        w1.position.set(0.8, 1.02, 0);
        w2.position.set(-0.8, 1.02, 0);
        platGroup.add(armL, w1, w2);

        this.rootGroup.add(platGroup);

        var arrowOmega = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0.18, 1.48, 0), 0.75, 0x22d3ee, 0.18, 0.09);
        var arrowL = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(-0.18, 1.48, 0), 0.75, 0xfbbf24, 0.18, 0.09);
        this.rootGroup.add(arrowOmega, arrowL);

        this.objs = { platGroup: platGroup, armL: armL, w1: w1, w2: w2, arrowOmega: arrowOmega, arrowL: arrowL };
        this.legendEl.innerHTML =
            '<span style="color:#22d3ee">■ ω (aumenta al encoger)</span>' +
            '<span style="color:#fbbf24">■ L (constante)</span>' +
            '<span style="color:#fb7185">● Pesas 3 kg</span>';
    };

    Renderer3D.prototype._buildP4 = function () {
        this._clearRoot();
        this.grid.position.y = -0.2;
        this.resetView('3d-p4');

        var platGroup = new THREE.Group();
        var discGeo = new THREE.CylinderGeometry(1, 1, 0.18, 56);
        var discMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.35, roughness: 0.5 });
        var disc = new THREE.Mesh(discGeo, discMat);
        disc.position.y = -0.09;
        platGroup.add(disc);

        // Radial line on platform
        var radial = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.03, 0.06), new THREE.MeshBasicMaterial({ color: 0x38bdf8 }));
        radial.position.set(0.5, 0.015, 0);
        platGroup.add(radial);

        // Man figure (moves along +X local axis)
        var man = new THREE.Group();
        var bodyMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8 });
        var torso = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.2, 1.2, 16), bodyMat);
        torso.position.y = 0.6;
        var head = new THREE.Mesh(new THREE.SphereGeometry(0.18, 16, 16), bodyMat);
        head.position.y = 1.4;
        man.add(torso, head);
        platGroup.add(man);

        this.rootGroup.add(platGroup);

        // Projectile
        var proj = new THREE.Mesh(
            new THREE.SphereGeometry(0.24, 18, 18),
            new THREE.MeshStandardMaterial({ color: 0xfb7185, emissive: 0x4c0519 })
        );
        this.rootGroup.add(proj);

        // Trail line + velocity arrow for thrown projectile
        var trailGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0.65, 0), new THREE.Vector3(0, 0.65, 0)]);
        var trailMat = new THREE.LineBasicMaterial({ color: 0xfb7185, transparent: true, opacity: 0.75 });
        var projTrail = new THREE.Line(trailGeo, trailMat);
        this.rootGroup.add(projTrail);

        var arrowL = new THREE.ArrowHelper(new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 1.8, 0), 1.2, 0xfbbf24, 0.22, 0.12);
        this.rootGroup.add(arrowL);

        this.objs = { platGroup: platGroup, disc: disc, radial: radial, man: man, proj: proj, projTrail: projTrail, arrowL: arrowL };
        this.legendEl.innerHTML =
            '<span style="color:#38bdf8">● Hombre (80 kg)</span>' +
            '<span style="color:#fb7185">● Masa arrojada (3 kg)</span>' +
            '<span style="color:#fbbf24">■ L_plat+hombre (hacia abajo)</span>';
    };

    Renderer3D.prototype.render = function (renderState) {
        if (!this.ready) return;

        var mode = renderState.type;
        if (this.currentSceneKey !== mode) {
            this.currentSceneKey = mode;
            if (mode === '3d-p1') this._buildP1();
            else if (mode === '3d-p3') this._buildP3();
            else if (mode === '3d-p4') this._buildP4();
        }

        if (mode === '3d-p1') {
            var a = renderState.a || 1.0;
            var visA = Math.min(1.22, 0.68 + 0.18 * a);
            // Map physics (x, y, z) -> Three.js (x, z, -y) (right-handed, Z up!)
            var ux = renderState.u[0], uy = renderState.u[2], uz = -renderState.u[1];
            var dirU = new THREE.Vector3(ux, uy, uz).normalize();
            var q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dirU);
            this.objs.rodGroup.quaternion.copy(q);
            this.objs.rodMesh.scale.set(1, visA, 1);
            this.objs.c1.position.y = visA;
            this.objs.c2.position.y = -visA;

            var wLen = Math.max(0.65, Math.min(1.3, 0.45 + 0.11 * renderState.omega));
            this.objs.arrowOmega.setLength(wLen, 0.22, 0.11);

            var Lx = renderState.L[0], Ly = renderState.L[2], Lz = -renderState.L[1];
            var vL = new THREE.Vector3(Lx, Ly, Lz);
            var lMag = vL.length();
            if (lMag > 1e-4) {
                this.objs.arrowL.visible = true;
                var visL = Math.max(0.95, Math.min(1.5, 0.85 + 0.18 * Math.log(1 + lMag)));
                this.objs.arrowL.setDirection(vL.clone().normalize());
                this.objs.arrowL.setLength(visL, 0.22, 0.12);
                var ringR = visL * Math.hypot(Lx, Lz) / lMag;
                var ringY = visL * (Ly / lMag);
                if (ringR > 0.03) {
                    this.objs.ring.visible = true;
                    this.objs.ring.scale.set(ringR, ringR, 1);
                    this.objs.ring.position.y = ringY;
                } else {
                    this.objs.ring.visible = false;
                }
            } else {
                this.objs.arrowL.visible = false;
                this.objs.ring.visible = false;
            }

            var tx = renderState.tau[0], ty = renderState.tau[2], tz = -renderState.tau[1];
            var vTau = new THREE.Vector3(tx, ty, tz);
            var tauMag = vTau.length();
            if (tauMag > 1e-4) {
                this.objs.arrowTau.visible = true;
                var visT = Math.max(0.7, Math.min(1.35, 0.6 + 0.16 * Math.log(1 + tauMag)));
                this.objs.arrowTau.setDirection(vTau.clone().normalize());
                this.objs.arrowTau.setLength(visT, 0.2, 0.1);
            } else {
                this.objs.arrowTau.visible = false;
            }
        } else if (mode === '3d-p3') {
            this.objs.platGroup.rotation.y = renderState.theta;
            var r = renderState.r;
            this.objs.w1.position.x = r;
            this.objs.w2.position.x = -r;
            this.objs.armL.scale.set(1, Math.max(0.2, 2 * r), 1);

            var wLen3 = Math.max(0.35, Math.min(1.25, 0.25 + renderState.omega * 0.045));
            var lLen3 = Math.max(0.4, Math.min(0.95, renderState.L * 0.012));
            this.objs.arrowOmega.setLength(wLen3, 0.18, 0.09);
            this.objs.arrowL.setLength(lLen3, 0.18, 0.09);
        } else if (mode === '3d-p4') {
            var R = renderState.R;
            this.objs.disc.scale.set(R, 1, R);
            this.objs.radial.scale.set(R, 1, 1);
            this.objs.radial.position.x = 0.5 * R;
            this.objs.platGroup.rotation.y = renderState.theta;
            this.objs.man.position.x = renderState.manR;

            if (renderState.thrown) {
                this.objs.proj.visible = true;
                this.objs.projTrail.visible = true;
                // Physics (x, y) -> Three.js (x, 0.65, -y)
                this.objs.proj.position.set(renderState.projPos.x, 0.65, -renderState.projPos.y);
                var posAttr = this.objs.projTrail.geometry.attributes.position;
                posAttr.setXYZ(0, R, 0.65, 0);
                posAttr.setXYZ(1, renderState.projPos.x, 0.65, -renderState.projPos.y);
                posAttr.needsUpdate = true;
                this.objs.arrowL.visible = true;
                this.objs.arrowL.setDirection(new THREE.Vector3(0, -1, 0));
                this.objs.arrowL.setLength(1.4, 0.25, 0.12);
            } else {
                this.objs.proj.visible = false;
                this.objs.projTrail.visible = false;
                this.objs.arrowL.visible = false;
            }
        }

        if (this.bannerEl) {
            this.bannerEl.textContent = renderState.banner || '';
        }

        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    };

    return Renderer3D;
}));
