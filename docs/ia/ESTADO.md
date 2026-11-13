# ESTADO DEL PROYECTO: Simulaciones (Física y Modelos Computacionales)

**Última actualización:** 13/11/2026
**Responsable:** MaikikiaM / Antigravity AI

## 1. Misión y Alcance
Proyectos de computación científica, simulación cuántica, cálculo numérico y visualizaciones 2D/3D interactivas.

## 2. Componentes Activos
- `mecanica-clasica/`: Laboratorio interactivo web (HTML5 Canvas 2D + Three.js 3D + integrador RK4) para aprender desde 0 los 13 problemas seleccionados de `Problemas T5.pdf` (Sistemas de Partículas, 6 niveles) y `Hoja_problemas_team6.pdf` (Sólido Rígido, 7 niveles).
  - `index.html`, `css/styles.css`, `js/app.js`: Interfaz científica en 3 columnas compatible directamente con `file://`.
  - `js/rk4.js`: Integrador RK4 y localización exacta de eventos por bisección.
  - `js/problems_particles.js`, `js/problems_rigid.js`, `js/problems_catalog.js`: Modelos analíticos, EDOs RK4 y guías pedagógicas desde 0.
  - `js/renderer2d.js`, `js/renderer3d.js`, `js/charts.js`: Renderizado 2D/3D en tiempo real y gráficas de conservación ($E_c, E_p, E_{\text{tot}}, |\vec{P}|, L_z$).
  - `tests/physics.test.js`: Suite de verificación numérica en Node.js (20 tests).
- `atomo-cuantico/`: Simulación matemática exacta de orbitales hidrogenoides ($\psi_{nlm}(r, \theta, \phi)$).
  - `quantum_core.py`, `render_atomo_4k.py`, `visor_atomo_3d.html`, `lib/`, `outputs/`.
- `registro.txt`: Bitácora continua del proyecto.

## 3. Estado de Compilación
- `mecanica-clasica/tests/physics.test.js`: **20 PASSED, 0 FAILED** con Node.js.
- `mecanica-clasica/index.html`: Verificado en Microsoft Edge (`file://` + WebGL) sin errores de consola.
- `atomo-cuantico/quantum_core.py` probado con numpy y scipy; `visor_atomo_3d.html` operativo.

## 4. Tareas Prioritarias
- [ ] Enlace remoto con repositorio privado `Simulaciones` (`git push` de `mecanica-clasica/`).
- [ ] Incorporar simulación de osciladores armónicos y potenciales cuánticos 1D/2D.
