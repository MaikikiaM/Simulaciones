# ESTADO DEL PROYECTO: Simulaciones (Física y Modelos Computacionales)

**Última actualización:** 19/11/2026
**Responsable:** MaikikiaM / Antigravity AI

## 1. Misión y Alcance
Proyectos de computación científica, simulación cuántica, cálculo numérico y visualizaciones 2D/3D interactivas compatibles offline con `file://`.

## 2. Componentes Activos
- `mecanica-clasica/`: Laboratorio interactivo web (HTML5 Canvas 2D + Three.js 3D + integrador RK4) con **35 simulaciones** en 4 bloques temáticos:
  - `js/problems_fundamentals.js`: **Fundamentos T1–T4** (4 niveles: tiro parabólico, plano inclinado con rozamiento, oscilador armónico M.A.S., órbita gravitatoria de Kepler).
  - `js/problems_particles.js`: **Sistemas de Partículas T5** (13 niveles de `Problemas T5.pdf`: `P1, P2, P3, P5, P6, P8, P9, P10, P11, P12, P14, P15, P16`).
  - `js/problems_rigid.js`: **Sólido Rígido T6a** (14 niveles de `Hoja_problemas_team6.pdf`: `P1–P6, P8–P15`).
  - `js/problems_elasticity.js`: **Elasticidad y Torsión T6b** (4 niveles de `Hoja_problemas_team6.pdf`: `P16–P19`).
  - `index.html`, `css/styles.css`, `js/app.js`, `js/problems_catalog.js`: Interfaz científica en 3 columnas con filtros por tema.
  - `js/rk4.js`, `js/renderer2d.js`, `js/renderer3d.js`, `js/charts.js`: Integrador RK4/bisección, renderizado 2D/3D y gráficas en vivo.
  - `tests/physics.test.js`: Suite de verificación numérica en Node.js (24 tests).
- `atomo-cuantico/`: Simulación matemática exacta de orbitales hidrogenoides ($\psi_{nlm}(r, \theta, \phi)$).
  - `quantum_core.py`, `render_atomo_4k.py`, `visor_atomo_3d.html`, `lib/`, `outputs/`.
- `registro.txt`: Bitácora continua del proyecto.

## 3. Estado de Compilación
- `mecanica-clasica/tests/physics.test.js`: **24 PASSED, 0 FAILED** con Node.js.
- `mecanica-clasica/index.html`: 35 simulaciones verificadas en Microsoft Edge (`file://` + WebGL) sin errores de consola.
- `atomo-cuantico/quantum_core.py` probado con numpy y scipy; `visor_atomo_3d.html` operativo.

## 4. Tareas Prioritarias
- [ ] Incorporar simulación de potenciales cuánticos 1D/2D en `atomo-cuantico/`.
