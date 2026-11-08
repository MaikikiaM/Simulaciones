"""
visor_spyder_3d.py - Visor 3D y Previsualizador de Gráficos para Spyder
Permite explorar la escultura cuántica v2 directamente dentro de Spyder (IPython / Plots).

INSTRUCCIONES DE USO EN SPYDER:
1. Abre este archivo en Spyder.
2. Presiona F5 (o el botón verde de ejecución ▶).
3. Verás la escultura cuántica en 3D y el render 4K en la pestaña 'Gráficos' (Plots).

TIPS PARA SPYDER:
- Para rotar la figura 3D con el ratón en una ventana flotante interactiva, escribe en la consola IPython:
      %matplotlib qt
  y pulsa F5 de nuevo.
- Para volver a ver las imágenes dentro de la pestaña integrada 'Gráficos' de Spyder:
      %matplotlib inline
  y pulsa F5.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image

# Asegurar importación de quantum_core
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from quantum_core import (
    get_sculpted_atom_config,
    sample_stratified_sculpted_orbs,
    generate_sculpted_nucleus
)

def graficar_atomo_3d_spyder(
    scale_factor: float = 0.35,
    elev: float = 13.5,
    azim: float = -76.0,
    fondo_color: str = "#000000",
    modo_color: str = "trio"
):
    """
    Genera y dibuja una figura 3D interactiva en Spyder con la escultura cuántica v2.
    
    Parámetros:
    - scale_factor: Factor de muestreo (0.35 produce ~13.500 orbes, óptimo para rotación fluida).
    - elev, azim: Ángulo de cámara calibrado para la calcomanía (-76° azimut, 13.5° elevación).
    - fondo_color: Color de fondo ('#000000' negro puro o '#16181D' grafito altavoz).
    - modo_color: 'trio' (Cian + Oro + Magenta) o 'fase' (+psi / -psi).
    """
    rng = np.random.default_rng(2026)
    configs = get_sculpted_atom_config()

    print(f"Muestreando orbes cuánticos estratificados (factor: {scale_factor:.2f})...")
    components = []
    total_pts = 0
    for cfg in configs:
        count = max(int(cfg['count'] * scale_factor), 350)
        comp = sample_stratified_sculpted_orbs(
            name=cfg['name'],
            n=cfg['n'],
            l=cfg['l'],
            Z_eff=cfg['Z_eff'],
            num_orbs=count,
            tau=cfg['tau'],
            component_type=cfg['type'],
            rng=rng,
            apply_cut=cfg['cut']
        )
        comp['color_rim'] = cfg['color_rim']
        comp['color_core'] = cfg['color_core']
        components.append(comp)
        total_pts += len(comp['positions'])

    # Núcleo cuántico
    nuc_count = max(int(650 * scale_factor), 200)
    nucleus = generate_sculpted_nucleus(num_orbs=nuc_count, rng=rng)
    total_pts += len(nucleus['positions'])
    print(f"Total orbes cargados para Spyder: {total_pts}")

    # Crear figura 3D
    fig = plt.figure(figsize=(10, 10), facecolor=fondo_color)
    ax = fig.add_subplot(111, projection='3d', facecolor=fondo_color)

    # Configuración de límites y cámara decal
    lim = 21.0
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()

    # Ocultar cuadrícula y paneles de Matplotlib
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor(fondo_color)
    ax.yaxis.pane.set_edgecolor(fondo_color)
    ax.zaxis.pane.set_edgecolor(fondo_color)
    ax.grid(False)

    # 1. Dibujar Núcleo
    nuc_pos = nucleus['positions']
    ax.scatter(
        nuc_pos[:, 0], nuc_pos[:, 1], nuc_pos[:, 2],
        s=8.0, c='#FFFFFF', alpha=0.98, edgecolors='#FFB000', linewidths=0.5,
        label='Núcleo Incandescente'
    )

    # 2. Dibujar Componentes Orbitales
    labels_map = {
        '1s_core': '1s Core (Cian Profundo)',
        '2s_shell': '2s Capa (Nodo Radial + Corte)',
        '3dz2_poles': '3d_z² Polos (Oro Solar)',
        '3dz2_ring': '3d_z² Anillo Saturno 360°',
        '3dxz_petals': '3d_xz 4 Pétalos Diagonales'
    }

    for comp in components:
        pos = comp['positions']
        p = comp['p']
        phase = comp['phase']
        name = comp['name']
        lbl = labels_map.get(name, name)

        if modo_color == "fase":
            # Modo Fase (+psi Cian, -psi Magenta)
            pos_mask = (phase >= 0)
            neg_mask = (phase < 0)
            if np.any(pos_mask):
                ax.scatter(
                    pos[pos_mask, 0], pos[pos_mask, 1], pos[pos_mask, 2],
                    s=np.clip(p[pos_mask] * 7.0, 1.5, 8.5),
                    c='#00E5FF', alpha=0.68
                )
            if np.any(neg_mask):
                ax.scatter(
                    pos[neg_mask, 0], pos[neg_mask, 1], pos[neg_mask, 2],
                    s=np.clip(p[neg_mask] * 7.0, 1.5, 8.5),
                    c='#FF148A', alpha=0.68
                )
        else:
            # Modo Trío Escultórico
            if name == '3dxz_petals':
                # Dos tonalidades sutiles para los 4 pétalos
                p_pos = (phase >= 0)
                p_neg = (phase < 0)
                ax.scatter(
                    pos[p_pos, 0], pos[p_pos, 1], pos[p_pos, 2],
                    s=np.clip(p[p_pos] * 7.5, 1.8, 9.0),
                    c='#FF148A', alpha=0.72, label=lbl + ' (+)'
                )
                ax.scatter(
                    pos[p_neg, 0], pos[p_neg, 1], pos[p_neg, 2],
                    s=np.clip(p[p_neg] * 7.5, 1.8, 9.0),
                    c='#9D4EDD', alpha=0.72, label=lbl + ' (-)'
                )
            else:
                col = comp['color_rim']
                ax.scatter(
                    pos[:, 0], pos[:, 1], pos[:, 2],
                    s=np.clip(p * 7.5, 1.6, 9.0),
                    c=col, alpha=0.70, label=lbl
                )

    plt.title(
        "Escultura Cuántica v2 - Visor 3D Spyder\n"
        f"Ángulo Calcomanía Altavoz: Azimut {azim}° | Elevación {elev}°",
        color='#F1F5F9', fontsize=12, pad=12
    )

    if modo_color != "fase":
        ax.legend(
            loc='lower right', facecolor='#11141D', edgecolor='#2D3748',
            fontsize=8.5, labelcolor='#E2E8F0', framealpha=0.85
        )

    plt.tight_layout()
    plt.show()

def mostrar_render_4k(modo: str = "trio"):
    """
    Carga y muestra el render final de alta definición directamente en el panel 'Plots' de Spyder.
    - modo: 'trio' (principal), 'fase' (dual cuántico), o 'lamina' (4 paneles de validación).
    """
    out_dir = os.path.join(current_dir, "outputs")
    archivos = {
        'trio': os.path.join(out_dir, "atomo_cuantico_4k.png"),
        'fase': os.path.join(out_dir, "atomo_cuantico_fase_4k.png"),
        'lamina': os.path.join(out_dir, "validacion_composicion.png")
    }

    ruta = archivos.get(modo, archivos['trio'])
    if not os.path.exists(ruta):
        print(f"No se encontró el archivo: {ruta}")
        print("Ejecuta primero render_atomo_4k.py para generarlo.")
        return

    print(f"Cargando render 4K ({modo}) en la pestaña Plots de Spyder...")
    img = Image.open(ruta)

    # Si la imagen tiene transparencia, montarla sobre fondo oscuro para visualizar en Spyder
    if img.mode == 'RGBA':
        fondo = Image.new("RGBA", img.size, (18, 20, 26, 255))
        img_disp = Image.alpha_composite(fondo, img).convert("RGB")
    else:
        img_disp = img

    plt.figure(figsize=(10, 10), facecolor='#0D1117')
    plt.imshow(img_disp)
    plt.axis('off')
    titulos = {
        'trio': 'Átomo Cuántico v2 - Trío Escultórico (Render 4K Ultra-HD)',
        'fase': 'Átomo Cuántico v2 - Dualidad de Fase ±ψ (Render 4K Ultra-HD)',
        'lamina': 'Lámina de Validación de Calcomanía (4 Paneles)'
    }
    plt.title(titulos.get(modo, ''), color='#00E5FF', fontsize=12, pad=12)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("\n" + "=" * 65)
    print(" VISOR CUÁNTICO PARA SPYDER (ÁTOMO ESCULTÓRICO v2)")
    print("=" * 65)
    print("1. Generando vista 3D en Matplotlib...")
    graficar_atomo_3d_spyder()

    print("\n2. Mostrando render 4K principal en la pestaña Plots...")
    mostrar_render_4k(modo="trio")

    print("\n[TIP SPYDER]")
    print("- Para rotar en 3D con el ratón: escribe  %matplotlib qt  en la consola IPython y pulsa F5.")
    print("- Para mostrar la lámina de 4 paneles: escribe  mostrar_render_4k('lamina')")
    print("=" * 65 + "\n")
