"""
quantum_core.py - Motor de Escultura Cuántica sin Solapamiento (v2)
Genera geometrías cuánticas estratificadas (Cáscara Rim-Cloud, Interior y Espina Hero)
basadas en autofunciones exactas R_{n,l}(r) e isodensidades limpias para alto contraste.
"""

import math
import numpy as np
from scipy.special import genlaguerre, factorial

def radial_wavefunction(n: int, l: int, Z_eff: float, r: np.ndarray) -> np.ndarray:
    """Calcula R_{n,l}(r; Z_eff) normalizada."""
    r = np.asarray(r, dtype=np.float64)
    rho = (2.0 * Z_eff * r) / n
    k = n - l - 1
    alpha = 2 * l + 1
    norm = np.sqrt(((2.0 * Z_eff / n) ** 3) * (factorial(k) / (2.0 * n * factorial(n + l))))
    L_val = genlaguerre(k, alpha)(rho)
    return norm * np.exp(-0.5 * rho) * (rho ** l) * L_val

def get_shell_normalized_radial(n: int, l: int, Z_eff: float, n_grid: int = 4096):
    """
    Retorna interpolador de la densidad de capa normalizada S_{n,l}(r) = (r^2 R^2) / max(r^2 R^2) in [0, 1]
    y sampler radial por CDF inversa exacta.
    """
    r_max = (n / Z_eff) * (3.0 * n + 14.0)
    r_grid = np.linspace(0.0001, r_max, n_grid, dtype=np.float64)
    R_vals = radial_wavefunction(n, l, Z_eff, r_grid)
    P_r = (r_grid ** 2) * (R_vals ** 2)
    P_max = P_r.max()
    S_r = P_r / (P_max + 1e-12)

    # CDF trapezoidal acumulada
    cdf = np.zeros_like(r_grid)
    cdf[1:] = np.cumsum(0.5 * (P_r[:-1] + P_r[1:]) * np.diff(r_grid))
    cdf /= cdf[-1]

    def sample_r(size: int, rng: np.random.Generator) -> np.ndarray:
        u = rng.uniform(0.0, 1.0, size=size)
        return np.interp(u, cdf, r_grid)

    def eval_S(r: np.ndarray) -> np.ndarray:
        return np.interp(r, r_grid, S_r, left=0.0, right=0.0)

    return sample_r, eval_S, r_max

def sample_stratified_sculpted_orbs(
    name: str,
    n: int,
    l: int,
    Z_eff: float,
    num_orbs: int,
    tau: float,
    component_type: str,
    rng: np.random.Generator,
    cam_azimuth_deg: float = -76.0,
    apply_cut: bool = False
):
    """
    Muestrea orbes con estratificación escultórica:
    - 45% Rim Cloud (cáscara exterior en [tau, 1.85*tau], radio 4.2 - 6.8 px)
    - 43% Interior (tau > 1.85*tau, radio 3.0 - 5.8 px)
    - 12% Hero Spine (máxima densidad > 0.70, radio 8.0 - 13.5 px)
    """
    sample_r, eval_S, r_max = get_shell_normalized_radial(n, l, Z_eff)
    
    n_rim = int(num_orbs * 0.45)
    n_spine = int(num_orbs * 0.12)
    n_interior = num_orbs - n_rim - n_spine

    cam_phi = math.radians(cam_azimuth_deg)
    half_wedge = math.radians(52.0)

    # Candidatos masivos vectorizados
    batch_size = max(num_orbs * 12, 60000)
    
    cand_pos = []
    cand_p = []
    cand_phase = []
    total_cand = 0

    while total_cand < num_orbs * 3:
        # Generar radios
        r_cand = sample_r(batch_size, rng)
        
        # Direcciones en esfera
        z_hat = rng.uniform(-1.0, 1.0, size=batch_size)
        phi = rng.uniform(0.0, 2.0 * math.pi, size=batch_size)
        rho_xy = np.sqrt(np.maximum(0.0, 1.0 - z_hat ** 2))
        x_hat = rho_xy * np.cos(phi)
        y_hat = rho_xy * np.sin(phi)

        S_vals = eval_S(r_cand)

        if component_type == '1s':
            P_sculpt = S_vals
            phase = np.ones_like(P_sculpt)
            valid_geom = np.ones(batch_size, dtype=bool)

        elif component_type == '2s':
            P_sculpt = S_vals
            # Signo cuántico exacto de 2s (cambia de signo en su nodo)
            R_raw = radial_wavefunction(n, l, Z_eff, r_cand)
            phase = np.sign(R_raw)
            phase[phase == 0.0] = 1.0
            valid_geom = np.ones(batch_size, dtype=bool)

        elif component_type == '3dz2_poles':
            ang_raw = 3.0 * (z_hat ** 2) - 1.0
            # Solo la región de conos polares z > 0 o z < 0
            valid_geom = ang_raw > 0.0
            ang_term = np.where(valid_geom, (ang_raw / 2.0) ** 5, 0.0)
            P_sculpt = S_vals * ang_term
            phase = np.ones_like(P_sculpt) # +psi en polos

        elif component_type == '3dz2_ring':
            ang_raw = 3.0 * (z_hat ** 2) - 1.0
            # Solo región del toroide ecuatorial
            valid_geom = ang_raw < 0.0
            ang_term = np.where(valid_geom, ((-ang_raw) / 1.0) ** 8, 0.0)
            P_sculpt = S_vals * ang_term
            phase = -np.ones_like(P_sculpt) # -psi en toroide ecuatorial

        elif component_type == '3dxz_petals':
            # 4 pétalos diagonales: ángulo relativo a planos x=0 y z=0
            ang_term = np.abs(2.0 * x_hat * z_hat) ** 6
            P_sculpt = S_vals * ang_term
            # Signo cuántico: sgn(x * z)
            phase = np.sign(x_hat * z_hat)
            phase[phase == 0.0] = 1.0
            valid_geom = np.ones(batch_size, dtype=bool)

        else:
            raise ValueError(f"Componente desconocido: {component_type}")

        # Filtro de cuña frontal solo si apply_cut
        if apply_cut:
            phi_cand = np.arctan2(y_hat, x_hat)
            dphi = np.arctan2(np.sin(phi_cand - cam_phi), np.cos(phi_cand - cam_phi))
            in_wedge = (np.abs(dphi) < half_wedge) & (z_hat > -0.35)
            valid_geom = valid_geom & (~in_wedge)

        # Filtro de isodensidad estricto
        pass_mask = valid_geom & (P_sculpt >= tau)
        if np.any(pass_mask):
            n_pass = np.sum(pass_mask)
            sub_r = r_cand[pass_mask]
            sub_x = sub_r * x_hat[pass_mask]
            sub_y = sub_r * y_hat[pass_mask]
            sub_z = sub_r * z_hat[pass_mask]
            sub_pts = np.column_stack([sub_x, sub_y, sub_z])

            cand_pos.append(sub_pts)
            cand_p.append(P_sculpt[pass_mask])
            cand_phase.append(phase[pass_mask])
            total_cand += n_pass

    all_pos = np.vstack(cand_pos)
    all_p = np.concatenate(cand_p)
    all_phase = np.concatenate(cand_phase)

    # 1. Puntos de Cáscara (Rim Cloud)
    rim_mask = (all_p >= tau) & (all_p <= 1.85 * tau)
    idx_rim_cand = np.where(rim_mask)[0]
    if len(idx_rim_cand) < n_rim:
        idx_rim = idx_rim_cand
    else:
        idx_rim = rng.choice(idx_rim_cand, size=n_rim, replace=False)

    # 2. Hero Spine (máxima densidad)
    spine_mask = all_p >= 0.70
    idx_spine_cand = np.where(spine_mask)[0]
    if len(idx_spine_cand) < n_spine:
        # Relajar cota si hay pocos
        idx_spine_cand = np.where(all_p >= 0.50)[0]
    idx_spine = rng.choice(idx_spine_cand, size=min(n_spine, len(idx_spine_cand)), replace=False)

    # 3. Interior
    used_set = set(idx_rim).union(set(idx_spine))
    all_indices = set(range(len(all_pos)))
    avail_indices = np.array(list(all_indices - used_set), dtype=np.int32)
    
    needed_interior = num_orbs - len(idx_rim) - len(idx_spine)
    if len(avail_indices) > needed_interior:
        # Ponderar por p^1.2
        weights = all_p[avail_indices] ** 1.2
        weights /= weights.sum()
        idx_int = rng.choice(avail_indices, size=needed_interior, replace=False, p=weights)
    else:
        idx_int = avail_indices

    # Asignar atributos por estrato
    final_pos = []
    final_p = []
    final_phase = []
    final_rad = []
    final_alpha = []
    final_stratum = [] # 0: rim, 1: int, 2: spine

    # Rim
    final_pos.append(all_pos[idx_rim])
    final_p.append(all_p[idx_rim])
    final_phase.append(all_phase[idx_rim])
    final_rad.append(rng.uniform(4.2, 6.8, size=len(idx_rim)))
    final_alpha.append(rng.uniform(0.85, 0.98, size=len(idx_rim)))
    final_stratum.append(np.zeros(len(idx_rim), dtype=np.int32))

    # Interior
    final_pos.append(all_pos[idx_int])
    final_p.append(all_p[idx_int])
    final_phase.append(all_phase[idx_int])
    final_rad.append(rng.uniform(3.0, 5.6, size=len(idx_int)))
    final_alpha.append(rng.uniform(0.68, 0.82, size=len(idx_int)))
    final_stratum.append(np.ones(len(idx_int), dtype=np.int32))

    # Spine
    final_pos.append(all_pos[idx_spine])
    final_p.append(all_p[idx_spine])
    final_phase.append(all_phase[idx_spine])
    final_rad.append(rng.uniform(8.0, 13.5, size=len(idx_spine)))
    final_alpha.append(np.full(len(idx_spine), 1.0))
    final_stratum.append(np.full(len(idx_spine), 2, dtype=np.int32))

    # Refuerzo especial de caras de corte para 2s
    if apply_cut and component_type == '2s':
        n_faces = int(num_orbs * 0.28)
        face_r = sample_r(n_faces * 3, rng)
        face_S = eval_S(face_r)
        face_mask = (face_S >= tau) & (face_r > 0.4) & (face_r < 4.2)
        valid_face_r = face_r[face_mask][:n_faces]
        
        side = rng.choice([-1.0, 1.0], size=len(valid_face_r))
        phi_f = cam_phi + side * half_wedge + rng.uniform(-math.radians(1.2), math.radians(1.2), size=len(valid_face_r))
        z_h_f = rng.uniform(-0.35, 1.0, size=len(valid_face_r))
        rho_f = np.sqrt(np.maximum(0.0, 1.0 - z_h_f ** 2))
        x_f = valid_face_r * rho_f * np.cos(phi_f)
        y_f = valid_face_r * rho_f * np.sin(phi_f)
        z_f = valid_face_r * z_h_f

        face_pos = np.column_stack([x_f, y_f, z_f])
        face_p = face_S[face_mask][:n_faces]
        
        final_pos.append(face_pos)
        final_p.append(face_p)
        final_phase.append(np.sign(radial_wavefunction(n, l, Z_eff, valid_face_r)))
        final_rad.append(rng.uniform(3.8, 6.2, size=len(face_pos)))
        final_alpha.append(np.full(len(face_pos), 0.95))
        final_stratum.append(np.zeros(len(face_pos), dtype=np.int32))

    return {
        'name': name,
        'positions': np.vstack(final_pos).astype(np.float32),
        'p': np.concatenate(final_p).astype(np.float32),
        'phase': np.concatenate(final_phase).astype(np.float32),
        'radii': np.concatenate(final_rad).astype(np.float32),
        'alpha': np.concatenate(final_alpha).astype(np.float32),
        'stratum': np.concatenate(final_stratum).astype(np.int32)
    }

def generate_sculpted_nucleus(num_orbs: int = 650, r_max: float = 0.42, rng: np.random.Generator = None):
    """
    Núcleo atómico incandescente: orbes compactos de radio 5 - 11 px.
    """
    if rng is None:
        rng = np.random.default_rng(2026)

    u = rng.uniform(0.0, 1.0, size=num_orbs)
    r = r_max * (u ** (1.0 / 2.0))
    z = rng.uniform(-1.0, 1.0, size=num_orbs)
    phi = rng.uniform(0.0, 2.0 * math.pi, size=num_orbs)
    rho = np.sqrt(np.maximum(0.0, 1.0 - z ** 2))
    
    pos = np.column_stack([r * rho * np.cos(phi), r * rho * np.sin(phi), r * z]).astype(np.float32)
    radii = rng.uniform(5.5, 11.0, size=num_orbs).astype(np.float32)
    p = np.exp(-((r / 0.22) ** 2)).astype(np.float32)
    phase = np.ones(num_orbs, dtype=np.float32)
    alpha = np.full(num_orbs, 1.0, dtype=np.float32)

    return {
        'name': 'nucleus',
        'positions': pos,
        'p': p,
        'phase': phase,
        'radii': radii,
        'alpha': alpha,
        'stratum': np.full(num_orbs, 2, dtype=np.int32)
    }

def get_sculpted_atom_config():
    """
    Configuración de la Escultura Cuántica v2:
    Sin colisiones, canales oscuros limpios, contraste máximo sobre negro.
    """
    return [
        {
            'name': '1s_core', 'n': 1, 'l': 0, 'Z_eff': 2.60,
            'count': 2200, 'tau': 0.22, 'type': '1s', 'cut': True,
            'color_rim': '#66FFFF', 'color_core': '#FFFFFF', 'lum': 1.15
        },
        {
            'name': '2s_shell', 'n': 2, 'l': 0, 'Z_eff': 2.25,
            'count': 5800, 'tau': 0.26, 'type': '2s', 'cut': True,
            'color_rim': '#00E5FF', 'color_core': '#CCFFFF', 'lum': 1.25
        },
        {
            'name': '3dz2_poles', 'n': 3, 'l': 2, 'Z_eff': 0.72,
            'count': 9200, 'tau': 0.28, 'type': '3dz2_poles', 'cut': False,
            'color_rim': '#FF9E00', 'color_core': '#FFF3A8', 'lum': 1.30
        },
        {
            'name': '3dz2_ring', 'n': 3, 'l': 2, 'Z_eff': 0.88,
            'count': 8800, 'tau': 0.28, 'type': '3dz2_ring', 'cut': False,
            'color_rim': '#FFB703', 'color_core': '#FFF8CC', 'lum': 1.35
        },
        {
            'name': '3dxz_petals', 'n': 3, 'l': 2, 'Z_eff': 0.68,
            'count': 12000, 'tau': 0.28, 'type': '3dxz_petals', 'cut': False,
            'color_rim': '#FF0A78', 'color_core': '#9D4EDD', 'lum': 1.28
        }
    ]
