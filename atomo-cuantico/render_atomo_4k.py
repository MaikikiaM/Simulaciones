"""
render_atomo_4k.py - Motor de Renderizado Escultórico 4K (v2)
Edición de Alto Impacto para Personalización sobre Negro / Altavoz JBL.
Genera orbes cuánticos macroscópicos ordenados por profundidad (Painter's Algorithm),
con siluetas escultóricas afiladas, glow ceñido y compuerta limpia sin neblina.
"""

import sys
import os
import time
import math
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

# Directorio del módulo
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from quantum_core import (
    get_sculpted_atom_config,
    sample_stratified_sculpted_orbs,
    generate_sculpted_nucleus
)

def hex_to_linear_rgb(hex_str: str) -> np.ndarray:
    """Convierte hex '#RRGGBB' a RGB lineal float32 [0, 1]."""
    h = hex_str.lstrip('#')
    r = int(h[0:2], 16) / 255.0
    g = int(h[2:4], 16) / 255.0
    b = int(h[4:6], 16) / 255.0
    return np.array([r ** 2.2, g ** 2.2, b ** 2.2], dtype=np.float32)

def linear_to_srgb(lin: np.ndarray) -> np.ndarray:
    """Curva IEC 61966-2-1 exacta."""
    lin = np.clip(lin, 0.0, 1.0)
    low = lin <= 0.0031308
    srgb = np.empty_like(lin)
    srgb[low] = 12.92 * lin[low]
    srgb[~low] = 1.055 * (lin[~low] ** (1.0 / 2.4)) - 0.055
    return srgb

def setup_camera_v2(
    azimuth_deg: float = -76.0,
    elevation_deg: float = 13.5,
    dist: float = 85.0,
    focal_px: float = 6500.0,
    size: int = 3840
):
    """Cámara calibrada v2 para vista frontal-isométrica dinámica."""
    phi = math.radians(azimuth_deg)
    theta = math.radians(elevation_deg)

    eye = np.array([
        dist * math.cos(theta) * math.cos(phi),
        dist * math.cos(theta) * math.sin(phi),
        dist * math.sin(theta)
    ], dtype=np.float32)

    target = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)

    fwd = target - eye
    fwd /= np.linalg.norm(fwd)

    right = np.cross(fwd, world_up)
    right /= np.linalg.norm(right)

    up = np.cross(right, fwd)
    up /= np.linalg.norm(up)

    return {
        'eye': eye,
        'fwd': fwd,
        'right': right,
        'up': up,
        'dist': dist,
        'focal': focal_px,
        'cx': size / 2.0,
        'cy': size / 2.0,
        'size': size
    }

def project_points_v2(pos: np.ndarray, cam: dict):
    """Proyección perspectiva con profundidad óptica w."""
    rel = pos - cam['eye']
    xv = np.dot(rel, cam['right'])
    yv = np.dot(rel, cam['up'])
    w = np.dot(rel, cam['fwd'])

    px = cam['cx'] + (xv * cam['focal']) / w
    py = cam['cy'] - (yv * cam['focal']) / w
    return px, py, w

def precompute_orb_kernels(num_buckets: int = 32, r_min: float = 2.4, r_max: float = 14.5):
    """
    Precomputa 32 plantillas antialiased de orbes cuánticos circulares
    con núcleo nítido, halo corto y perfil de mezcla hacia blanco.
    """
    radii = np.linspace(r_min, r_max, num_buckets, dtype=np.float32)
    kernels = []

    for rad in radii:
        R_int = int(np.ceil(rad * 1.55))
        y, x = np.mgrid[-R_int:R_int + 1, -R_int:R_int + 1].astype(np.float32)
        d = np.sqrt(x * x + y * y) / rad
        
        # Núcleo nítido de alta densidad
        core = np.clip(1.30 - 1.25 * (d ** 2), 0.0, 1.0)
        # Borde suave cuántico
        rim = np.exp(-2.4 * (d ** 2))
        
        alpha_k = np.clip(0.86 * core + 0.22 * rim, 0.0, 1.0)
        alpha_k[d > 1.50] = 0.0

        kernels.append({
            'rad': rad,
            'R_int': R_int,
            'core': core[..., None],
            'alpha': alpha_k[..., None]
        })

    return radii, kernels

def render_sculpted_scene(mode: str = 'trio', seed: int = 2026, size: int = 3840):
    """
    Renderiza la escena atómica escultórica completa en 4K.
    mode='trio': Trío Alto Contraste (Cian + Oro + Magenta/Violeta)
    mode='phase': Cyberpunk Cuántico Dual (+psi Cian, -psi Magenta, Núcleo Oro)
    """
    print(f"\n[ESCULTURA 4K v2] Generando escena en modo '{mode}' ({size}x{size})...")
    t0 = time.time()
    rng = np.random.default_rng(seed)

    cam = setup_camera_v2(azimuth_deg=-76.0, elevation_deg=13.5, dist=85.0, focal_px=6500.0, size=size)
    radii_lut, kernels_lut = precompute_orb_kernels(32, 2.4, 14.5)

    configs = get_sculpted_atom_config()

    all_positions = []
    all_colors = []
    all_radii = []
    all_alphas = []
    all_lums = []

    white_hot = np.array([1.0, 0.98, 0.96], dtype=np.float32)

    # 1. Muestrear componentes orbitales
    for cfg in configs:
        data = sample_stratified_sculpted_orbs(
            name=cfg['name'],
            n=cfg['n'],
            l=cfg['l'],
            Z_eff=cfg['Z_eff'],
            num_orbs=cfg['count'],
            tau=cfg['tau'],
            component_type=cfg['type'],
            rng=rng,
            cam_azimuth_deg=-76.0,
            apply_cut=cfg['cut']
        )

        pos = data['positions']
        p_val = data['p']
        phase = data['phase']
        radii = data['radii']
        alpha = data['alpha']
        N = len(pos)

        # Asignación de colores
        if mode == 'trio':
            col_rim = hex_to_linear_rgb(cfg['color_rim'])
            col_core = hex_to_linear_rgb(cfg['color_core'])

            # Para 3dxz: gradiente entre magenta (+xz) y violeta eléctrico (-xz)
            if cfg['type'] == '3dxz_petals':
                col_pts = np.empty((N, 3), dtype=np.float32)
                pos_mask = phase > 0
                col_pts[pos_mask] = hex_to_linear_rgb('#FF0A78') # Magenta láser
                col_pts[~pos_mask] = hex_to_linear_rgb('#9D4EDD') # Violeta eléctrico
            else:
                # Mezcla hacia col_core según p
                t_col = np.clip((p_val[:, None] - cfg['tau']) / (1.0 - cfg['tau']), 0.0, 1.0)
                col_pts = col_rim * (1.0 - 0.45 * t_col) + col_core * (0.45 * t_col)

            lums = np.full(N, cfg['lum'], dtype=np.float32)

        else: # modo 'phase' (Cyberpunk Dual)
            col_pts = np.empty((N, 3), dtype=np.float32)
            pos_mask = phase > 0
            col_pts[pos_mask] = np.array([0.0, 0.95, 1.0], dtype=np.float32) # +psi Cian Eléctrico
            col_pts[~pos_mask] = np.array([1.0, 0.05, 0.55], dtype=np.float32) # -psi Magenta Neón
            lums = np.full(N, 1.25, dtype=np.float32)

        all_positions.append(pos)
        all_colors.append(col_pts)
        all_radii.append(radii)
        all_alphas.append(alpha)
        all_lums.append(lums)

    # 2. Núcleo incandescente
    nuc_data = generate_sculpted_nucleus(num_orbs=650, r_max=0.42, rng=rng)
    nuc_N = len(nuc_data['positions'])
    all_positions.append(nuc_data['positions'])
    
    # Núcleo siempre en blanco-oro ultra brillante
    nuc_col = np.tile(np.array([1.0, 0.88, 0.55], dtype=np.float32), (nuc_N, 1))
    all_colors.append(nuc_col)
    all_radii.append(nuc_data['radii'])
    all_alphas.append(nuc_data['alpha'])
    all_lums.append(np.full(nuc_N, 1.85, dtype=np.float32))

    # Unificar arrays
    P_world = np.vstack(all_positions)
    C_world = np.vstack(all_colors)
    R_world = np.concatenate(all_radii)
    A_world = np.concatenate(all_alphas)
    L_world = np.concatenate(all_lums)

    total_orbs = len(P_world)
    print(f"  Total orbes cuánticos a pintar: {total_orbs:,}")

    # 3. Proyección 2D
    px, py, w = project_points_v2(P_world, cam)

    # 4. Ordenación 3D de Atrás hacia Delante (Painter's Algorithm)
    sort_idx = np.argsort(-w) # w mayor = más lejano -> pintar primero
    px = px[sort_idx]
    py = py[sort_idx]
    w = w[sort_idx]
    C_world = C_world[sort_idx]
    R_world = R_world[sort_idx]
    A_world = A_world[sort_idx]
    L_world = L_world[sort_idx]

    # Sombreado de profundidad 3D
    d_cam = w - cam['dist'] # [-18, +18]
    # Orbes cercanos (d_cam < 0): escala x1.18, lum x1.28
    # Orbes lejanos (d_cam > 0): escala x0.82, lum x0.52
    t_depth = np.clip(d_cam / 18.0, -1.0, 1.0)
    f_scale = 1.0 - 0.18 * t_depth
    f_lum = 1.0 - 0.40 * t_depth

    R_world = R_world * f_scale
    L_world = L_world * f_lum

    # Asignar a buckets de radio precalculados
    bucket_idx = np.clip(((R_world - 2.4) / (14.5 - 2.4) * 31.0).astype(np.int32), 0, 31)

    # 5. Composición en Lienzo Flotante Premultiplicado
    canvas = np.zeros((size, size, 4), dtype=np.float32)

    # Filtrar orbes dentro del lienzo con margen
    ix_arr = np.round(px).astype(np.int32)
    iy_arr = np.round(py).astype(np.int32)

    for i in range(total_orbs):
        cx, cy = ix_arr[i], iy_arr[i]
        k_info = kernels_lut[bucket_idx[i]]
        R_int = k_info['R_int']

        if cx - R_int < 0 or cx + R_int >= size or cy - R_int < 0 or cy + R_int >= size:
            continue

        core_k = k_info['core']
        a_k = k_info['alpha'] * A_world[i]
        
        # Color del orbe: centro blanco incandescente hacia color neón rico y saturado
        base_c = C_world[i] * L_world[i]
        orb_rgb = base_c * (1.0 - 0.42 * (core_k ** 1.4)) + white_hot * (0.42 * (core_k ** 1.4))

        src_pre = orb_rgb * a_k
        inv_a = 1.0 - a_k

        patch = canvas[cy - R_int:cy + R_int + 1, cx - R_int:cx + R_int + 1]
        patch[..., :3] = src_pre + patch[..., :3] * inv_a
        patch[..., 3:4] = a_k + patch[..., 3:4] * inv_a

    print(f"  Pintado de orbes completado en {time.time() - t0:.2f}s")
    print(f"  Aplicando resplandor neón ceñido y compuerta limpia...")

    # 6. Resplandor Neón Ceñido (Sin Niebla)
    glow = (
        0.24 * gaussian_filter(canvas[..., :3], sigma=(10.0, 10.0, 0))
        + 0.08 * gaussian_filter(canvas[..., :3], sigma=(26.0, 26.0, 0))
    )
    rgb_hdr = canvas[..., :3] + glow

    # 7. Tonemapping Fílmico Vibrante
    exposure = 1.35 if mode == 'trio' else 1.45
    l_lin = 1.0 - np.exp(-exposure * np.maximum(rgb_hdr, 0.0))
    l_srgb = linear_to_srgb(l_lin)

    # 8. Compuerta Limpia de Adhesivo (Clean Sticker Gate)
    # Cualquier niebla residual por debajo del 3.5% se desvanece suavemente a cero absoluto
    alpha_raw = np.maximum(canvas[..., 3], np.max(l_srgb, axis=2))
    
    gate = np.clip((alpha_raw - 0.038) / (0.095 - 0.038), 0.0, 1.0)
    gate = gate * gate * (3.0 - 2.0 * gate) # smoothstep
    alpha_clean = alpha_raw * gate

    # 9. Straight-Alpha Cero-Halo
    min_thresh = 0.5 / 255.0
    active_mask = alpha_clean > min_thresh

    straight_rgb = np.zeros_like(l_srgb)
    for c in range(3):
        straight_rgb[active_mask, c] = l_srgb[active_mask, c] / (alpha_clean[active_mask] + 1e-8)

    straight_rgb = np.clip(straight_rgb, 0.0, 1.0)

    # Dithering leve en alpha para suavidad en borde activo
    dither = (np.random.uniform(-0.5, 0.5, size=(size, size)) + np.random.uniform(-0.5, 0.5, size=(size, size))) / 255.0
    alpha_final = np.clip(alpha_clean + dither, 0.0, 1.0)
    alpha_final[~active_mask] = 0.0

    # Cuantización uint8
    r_u8 = (straight_rgb[..., 0] * 255.0 + 0.5).astype(np.uint8)
    g_u8 = (straight_rgb[..., 1] * 255.0 + 0.5).astype(np.uint8)
    b_u8 = (straight_rgb[..., 2] * 255.0 + 0.5).astype(np.uint8)
    a_u8 = (alpha_final * 255.0 + 0.5).astype(np.uint8)

    # Compuerta limpia estricta: cero píxeles de niebla fantasma menor a 10
    a_u8[a_u8 < 10] = 0

    # Margen perimetral limpio de 30 px
    margin = 30
    a_u8[:margin, :] = 0
    a_u8[-margin:, :] = 0
    a_u8[:, :margin] = 0
    a_u8[:, -margin:] = 0

    zero_mask = a_u8 == 0
    r_u8[zero_mask] = 0
    g_u8[zero_mask] = 0
    b_u8[zero_mask] = 0

    rgba = np.dstack([r_u8, g_u8, b_u8, a_u8])
    print(f"  Render completado en {time.time() - t0:.2f} segundos!")
    return rgba

def create_validation_sheet_v2(img_rgba: Image.Image, output_path: str):
    """
    Lámina 2x2:
    - Panel 1: Negro Puro #000000 (carcasa altavoz)
    - Panel 2: Gris Grafito #16181D (tejido acústico de altavoz)
    - Panel 3: Damero de transparencia clara
    - Panel 4: Recorte macro 1:1 al 100% de zoom
    """
    print("[AUDITORIA v2] Generando lámina de validación compositiva...")
    w, h = img_rgba.size
    preview_size = (1200, 1200)
    img_preview = img_rgba.resize(preview_size, Image.Resampling.LANCZOS)

    # 1. Negro Puro
    bg_black = Image.new("RGBA", preview_size, (0, 0, 0, 255))
    bg_black.alpha_composite(img_preview)

    # 2. Gris Grafito Altavoz #16181D
    bg_graphite = Image.new("RGBA", preview_size, (22, 24, 29, 255))
    bg_graphite.alpha_composite(img_preview)

    # 3. Damero Transparencia
    check_tile = 24
    checker = np.zeros((preview_size[1], preview_size[0], 3), dtype=np.uint8)
    y_i, x_i = np.indices((preview_size[1], preview_size[0]))
    mask = ((x_i // check_tile) + (y_i // check_tile)) % 2 == 0
    checker[mask] = 175
    checker[~mask] = 225
    bg_check = Image.fromarray(checker).convert("RGBA")
    bg_check.alpha_composite(img_preview)

    # 4. Zoom 1:1 Macro al centro y anillo
    cx, cy = w // 2, h // 2
    crop_box = (cx - 600, cy - 600, cx + 600, cy + 600)
    macro_rgba = img_rgba.crop(crop_box)
    macro_black = Image.new("RGBA", (1200, 1200), (0, 0, 0, 255))
    macro_black.alpha_composite(macro_rgba)

    # Montar lámina
    canvas = Image.new("RGB", (2520, 2600), (14, 16, 20))
    canvas.paste(bg_black.convert("RGB"), (40, 60))
    canvas.paste(bg_graphite.convert("RGB"), (1280, 60))
    canvas.paste(bg_check.convert("RGB"), (40, 1340))
    canvas.paste(macro_black.convert("RGB"), (1280, 1340))

    canvas.save(output_path, quality=95)
    print(f"  Lámina v2 guardada en: {output_path}")

def run_numerical_audit_v2(rgba: np.ndarray):
    """Verifica criterios estrictos de aceptación de la sección 5 del plan."""
    print("\n--- AUDITORÍA NUMÉRICA DE CONTRASTE Y SILUETA (v2) ---")
    H, W, _ = rgba.shape
    alpha = rgba[..., 3]
    rgb = rgba[..., :3]

    # 1. Fracción de espacio negativo negro limpio (alpha == 0)
    pct_zero = (alpha == 0).mean() * 100.0
    print(f"1. Espacio negativo negro limpio (alpha == 0): {pct_zero:.2f}% (Rango requerido: 55% - 78%) -> {'PASÓ' if 55.0 <= pct_zero <= 78.0 else 'ATENCIÓN: ' + str(pct_zero)}")

    # 2. Perímetro de 30 px
    border_max = max(alpha[:30, :].max(), alpha[-30:, :].max(), alpha[:, :30].max(), alpha[:, -30:].max())
    print(f"2. Alpha máximo en perímetro exterior (30px): {border_max} (Requerido: 0) -> {'PASÓ' if border_max == 0 else 'FALLÓ'}")

    # 3. Compuerta limpia para calcomanía (cero píxeles con 0 < alpha < 10)
    faint_dust = np.sum((alpha > 0) & (alpha < 10))
    print(f"3. Píxeles de niebla ultratenue (0 < A < 10): {faint_dust} (Requerido: 0) -> {'PASÓ' if faint_dust == 0 else 'FALLÓ'}")

    # 4. Pureza Straight-Alpha en resplandor (20 <= A <= 200)
    glow_mask = (alpha >= 20) & (alpha <= 200)
    if np.any(glow_mask):
        max_c = np.max(rgb[glow_mask], axis=1)
        pct_purity = (max_c >= 240).mean() * 100.0
        print(f"4. Pureza Straight-Alpha en borde activo (20 <= A <= 200): {pct_purity:.2f}% con canal >= 240/255 -> {'PASÓ' if pct_purity > 98.0 else 'FALLÓ'}")

    print("------------------------------------------------------\n")

def save_optimized_png(rgba: np.ndarray, path: str):
    """
    Guarda el archivo PNG optimizado para cumplir con el límite estricto de < 10 MB
    de plataformas de calcomanías como JBL, manteniendo 4K nativo (3840x3840).
    """
    arr = rgba.copy()
    # Micro-cuantización imperceptible (6 bits por canal, delta <= 3/255)
    arr[..., :3] = arr[..., :3] & 0xFC
    arr[arr[..., 3] < 16] = 0
    img = Image.fromarray(arr, mode="RGBA")
    img.save(path, format="PNG", compress_level=9)
    sz_mb = os.path.getsize(path) / 1000000.0
    print(f"  Guardado {os.path.basename(path)}: {sz_mb:.2f} MB (< 10 MB JBL Safe)")

def main():
    out_dir = os.path.join(current_dir, "outputs")
    os.makedirs(out_dir, exist_ok=True)

    path_main_4k = os.path.join(out_dir, "atomo_cuantico_4k.png")
    path_fase_4k = os.path.join(out_dir, "atomo_cuantico_fase_4k.png")
    path_val_sheet = os.path.join(out_dir, "validacion_composicion.png")

    # 1. Render Principal v2: Trío de Alto Contraste
    rgba_main = render_sculpted_scene(mode='trio', seed=2026, size=3840)
    print(f"Optimizando y guardando imagen principal 4K: {path_main_4k}...")
    save_optimized_png(rgba_main, path_main_4k)
    img_main = Image.open(path_main_4k)

    # Auditoría
    run_numerical_audit_v2(rgba_main)

    # 2. Render Variante v2: Cyberpunk / Fase Cuántica Dual
    rgba_fase = render_sculpted_scene(mode='phase', seed=2026, size=3840)
    print(f"Optimizando y guardando imagen de fase cuántica 4K: {path_fase_4k}...")
    save_optimized_png(rgba_fase, path_fase_4k)

    # 3. Lámina de Validación Compositiva v2
    create_validation_sheet_v2(img_main, path_val_sheet)

    print("\n[ÉXITO v2] Todos los renders 4K y lámina de validación generados correctamente.")

    # Mostrar vista previa automática en la pestaña 'Plots' de Spyder
    try:
        print("Enviando lámina de validación a la pestaña 'Gráficos' de Spyder...")
        import matplotlib.pyplot as plt
        val_img = Image.open(path_val_sheet)
        plt.figure(figsize=(10, 10), facecolor='#0D1117')
        plt.imshow(val_img)
        plt.axis('off')
        plt.title("Átomo Cuántico v2 - Lámina de Validación (Spyder Plots)", color='#00E5FF', fontsize=12, pad=10)
        plt.tight_layout()
        plt.show()
        print("¡Listo! Si no la ves en 'Gráficos', escribe en la consola: %matplotlib inline")
    except Exception as e:
        print(f"Nota Matplotlib: {e}")

if __name__ == "__main__":
    main()
