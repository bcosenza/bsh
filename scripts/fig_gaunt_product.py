#!/usr/bin/env python3
"""
make_gaunt_product.py
=====================

Generates `figs/gaunt_product.pdf`: three polar plots showing two
vMF-smoothed BSH fields on the equatorial slice of S^2 and their
Gaunt-product composition.

Usage:
    python3 make_gaunt_product.py

Dependencies: numpy, matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------
# Real spherical-harmonic basis at order n = 3 (Cartesian form)
# Matches Table 1 of the paper (§3.2).
# ---------------------------------------------------------------------

def real_sh_table(d):
    """Evaluate the first 9 real SH basis functions at unit vector d.

    d : (..., 3) array of unit vectors.
    Returns a list [Y_00, Y_1-1, Y_10, Y_11, Y_2-2, Y_2-1, Y_20, Y_21, Y_22]
    of arrays of the same shape as d[..., 0].
    """
    x, y, z = d[..., 0], d[..., 1], d[..., 2]
    sqrt_pi = np.sqrt(np.pi)
    sqrt_3_pi = np.sqrt(3.0 / np.pi)
    sqrt_5_pi = np.sqrt(5.0 / np.pi)
    sqrt_15_pi = np.sqrt(15.0 / np.pi)
    return [
        0.5 / sqrt_pi * np.ones_like(x),            # (0,  0)
        0.5 * sqrt_3_pi * y,                         # (1, -1)
        0.5 * sqrt_3_pi * z,                         # (1,  0)
        0.5 * sqrt_3_pi * x,                         # (1,  1)
        0.5 * sqrt_15_pi * x * y,                    # (2, -2)
        0.5 * sqrt_15_pi * y * z,                    # (2, -1)
        0.25 * sqrt_5_pi * (3.0 * z * z - 1.0),      # (2,  0)
        0.5 * sqrt_15_pi * x * z,                    # (2,  1)
        0.25 * sqrt_15_pi * (x * x - y * y),         # (2,  2)
    ]


# Band index for each of the 9 coefficients (needed for per-band scaling).
BAND_OF = [0, 1, 1, 1, 2, 2, 2, 2, 2]


# ---------------------------------------------------------------------
# vMF projection: Dirac coefficients scaled per band
# ---------------------------------------------------------------------

def vmf_zonal_coeffs(kappa, n_order=3):
    """Zonal coefficients a_l of the vMF kernel on S^2."""
    a = [1.0, 1.0 / np.tanh(kappa) - 1.0 / kappa]
    for l in range(1, n_order - 1):
        a.append(a[l - 1] - (2 * l + 1) * a[l] / kappa)
    return a[:n_order]


def project_vmf_lobe(direction_xyz, kappa=2.95):
    """Project a single vMF lobe centred at `direction_xyz` to 9 BSH coeffs."""
    d = np.asarray(direction_xyz, dtype=float)[None, :]  # shape (1, 3)
    basis_vals = real_sh_table(d)                         # list of (1,) arrays
    a = vmf_zonal_coeffs(kappa, n_order=3)
    coeffs = np.array([basis_vals[k][0] * a[BAND_OF[k]] for k in range(9)])
    return coeffs


def reconstruct(coeffs, d):
    """Reconstruct the BSH field at direction grid d (..., 3)."""
    basis_vals = real_sh_table(d)
    out = np.zeros(d.shape[:-1])
    for k in range(9):
        out += coeffs[k] * basis_vals[k]
    return out


# ---------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------

def equatorial_dirs(n_phi=720):
    """Return (phi, d) where d is an (n_phi, 3) array of equatorial unit vectors."""
    phi = np.linspace(0.0, 2.0 * np.pi, n_phi, endpoint=False)
    d = np.stack([np.cos(phi), np.sin(phi), np.zeros_like(phi)], axis=-1)
    return phi, d


def unit_from_phi(phi_angle):
    """Equatorial unit vector at azimuth phi_angle (z = 0)."""
    return np.array([np.cos(phi_angle), np.sin(phi_angle), 0.0])


def main():
    phi, d_grid = equatorial_dirs(n_phi=720)

    # Two input vMF fields placed on the equator
    dir_A = unit_from_phi(np.pi / 3.0)       # 60 degrees
    dir_B = unit_from_phi(2.0 * np.pi / 3.0) # 120 degrees
    kappa = 2.95

    coeffs_A = project_vmf_lobe(dir_A, kappa=kappa)
    coeffs_B = project_vmf_lobe(dir_B, kappa=kappa)

    f_A = reconstruct(coeffs_A, d_grid)
    f_B = reconstruct(coeffs_B, d_grid)
    # Pointwise product in the angular domain = what the Gaunt product
    # computes at the coefficient level (modulo truncation back to n=3).
    f_prod = f_A * f_B

    def clip_pos(x):
        return np.maximum(x, 0.0)

    fields = [clip_pos(f_A), clip_pos(f_B), clip_pos(f_prod)]
    titles = [r"Field $A$", r"Field $B$", r"$A \times B$ (Gaunt product)"]
    colors = ["#2b5a8c", "#2b5a8c", "#8c4a2b"]

    fig, axs = plt.subplots(1, 3,
                            subplot_kw={"projection": "polar"},
                            figsize=(11, 4.0))
    for ax, f, title, color in zip(axs, fields, titles, colors):
        ax.plot(phi, f, color=color, lw=1.8)
        ax.fill(phi, f, color=color, alpha=0.22)
        ax.set_title(title, pad=14, fontsize=12)
        ax.set_rticks([])
        ax.grid(alpha=0.3)
        ax.set_theta_zero_location("E")
        ax.set_theta_direction(1)

    plt.tight_layout()
    plt.savefig("gaunt_product.pdf", bbox_inches="tight")
    plt.savefig("gaunt_product.png", dpi=150, bbox_inches="tight")
    print("Wrote gaunt_product.pdf and gaunt_product.png")


if __name__ == "__main__":
    main()
