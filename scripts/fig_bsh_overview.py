#!/usr/bin/env python3
r"""
fig_bsh_overview.py  ->  figures/bsh_overview.pdf  (Section 3, \label{fig:bsh-overview})

Two-part overview of the BSH representation (journal counterpart of Table 1 + Fig. 1 of
the 2015 version):

  Top row  -- the projection pipeline for one representative group:
              (1) agent direction arrows  ->  (2) SH coefficient vector c_i
                                           ->  (3) reconstructed directional field f-hat
  Bottom row -- three different group orientations yielding three different functions,
                including a *bimodal* group whose two lobes survive projection although
                a vector average would cancel to zero (the expressiveness argument).

All fields are the equatorial (theta = pi/2) slice of the order-3 real SH, lightly
smoothed with the heat-kernel window of Section 4 so the lobes read cleanly.

Run:  python3 fig_bsh_overview.py
"""
import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

# --------------------------------------------------------------------------- style
mpl.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.size": 9.5,
    "axes.linewidth": 0.8,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.03,
    "pdf.fonttype": 42, "ps.fonttype": 42,
})
OUTDIR = os.environ.get("FIGDIR", "figures")
os.makedirs(OUTDIR, exist_ok=True)

ACCENT = "#c1440e"    # lobe / field colour
BARCOL = "#33628f"    # coefficient bars
ARROWC = "#444444"

# ---------------------------------------------- orthonormal real SH, bands l = 0..2
_C = [0.50*np.sqrt(1/np.pi), 0.50*np.sqrt(3/np.pi), 0.50*np.sqrt(3/np.pi),
      0.50*np.sqrt(3/np.pi), 0.50*np.sqrt(15/np.pi), 0.50*np.sqrt(15/np.pi),
      0.25*np.sqrt(5/np.pi), 0.50*np.sqrt(15/np.pi), 0.25*np.sqrt(15/np.pi)]

def real_sh(i, x, y, z):
    x = np.asarray(x, float); y = np.asarray(y, float); z = np.asarray(z, float)
    return [_C[0]*np.ones_like(x), _C[1]*y, _C[2]*z, _C[3]*x, _C[4]*x*y,
            _C[5]*y*z, _C[6]*(3*z*z-1), _C[7]*x*z, _C[8]*(x*x-y*y)][i]

def band_of(i):
    return int(np.floor(np.sqrt(i)))          # i = l(l+1)+m  ==>  l = floor(sqrt(i))

def group_coeffs(angles_deg, sigma=None):
    """Dirac projection of in-plane directions, with optional heat-kernel smoothing."""
    a = np.radians(np.asarray(angles_deg, float))
    x, y, z = np.cos(a), np.sin(a), np.zeros_like(a)
    c = np.array([float(real_sh(i, x, y, z).sum()) for i in range(9)])
    if sigma is not None:
        for i in range(9):
            l = band_of(i)
            c[i] *= np.exp(-sigma**2 * l * (l + 1) / 2.0)
    return c

def reconstruct(c, phi):
    x, y, z = np.cos(phi), np.sin(phi), np.zeros_like(phi)
    f = np.zeros_like(phi)
    for i in range(9):
        f += c[i] * real_sh(i, x, y, z)
    return f

def plot_lobe(ax, angles_deg, title, sigma=0.35, color=ACCENT):
    phi = np.linspace(0, 2*np.pi, 500)
    c = group_coeffs(angles_deg, sigma=sigma)
    r = np.clip(reconstruct(c, phi), 0, None)
    ax.plot(phi, r, color=color, lw=1.4)
    ax.fill(phi, r, color=color, alpha=0.22)
    ax.set_xticklabels([]); ax.set_yticklabels([])
    ax.set_yticks([]); ax.grid(alpha=0.25, lw=0.5)
    ax.set_title(title, fontsize=9.5, pad=6)
    return c

# --------------------------------------------------------------------------- figure
fig = plt.figure(figsize=(7.3, 5.0))
gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 0.92], hspace=0.5, wspace=0.42)

ref_group = [27, 38, 50, 52, 63, 63]          # fan pointing up-right (cf. conference d_i)

# (1) agent directions -------------------------------------------------------------
ax_arr = fig.add_subplot(gs[0, 0])
for a in np.radians(ref_group):
    ax_arr.annotate("", xy=(np.cos(a), np.sin(a)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color=ARROWC, lw=1.3))
ax_arr.set_xlim(-0.25, 1.15); ax_arr.set_ylim(-0.25, 1.15)
ax_arr.set_aspect("equal"); ax_arr.set_xticks([]); ax_arr.set_yticks([])
ax_arr.set_title(r"(1) agent directions $\mathbf{d}_a$", fontsize=9.5, pad=6)

# (2) coefficient vector -----------------------------------------------------------
ax_bar = fig.add_subplot(gs[0, 1])
c_ref = group_coeffs(ref_group, sigma=0.35)
ax_bar.bar(range(9), c_ref, color=BARCOL, width=0.72)
ax_bar.axhline(0, color="k", lw=0.6)
ax_bar.set_xticks(range(9))
ax_bar.set_xlabel(r"coefficient index $i$")
ax_bar.set_title(r"(2) SH coefficients $c_i$", fontsize=9.5, pad=6)
ax_bar.tick_params(labelsize=8)

# (3) reconstructed field ----------------------------------------------------------
ax_lobe = fig.add_subplot(gs[0, 2], projection="polar")
plot_lobe(ax_lobe, ref_group, r"(3) reconstructed field $\hat{f}$", sigma=0.35)

# pipeline arrows between the top panels
fig.text(0.360, 0.735, r"$\longrightarrow$", fontsize=15, ha="center", va="center")
fig.text(0.360, 0.775, "project", fontsize=8, ha="center", va="center")
fig.text(0.655, 0.735, r"$\longrightarrow$", fontsize=15, ha="center", va="center")
fig.text(0.655, 0.775, "reconstruct", fontsize=8, ha="center", va="center")

# bottom row: distinct orientations -> distinct functions --------------------------
ax_r = fig.add_subplot(gs[1, 0], projection="polar")
ax_u = fig.add_subplot(gs[1, 1], projection="polar")
ax_b = fig.add_subplot(gs[1, 2], projection="polar")
plot_lobe(ax_r, [-20, -10, 0, 10, 20],        r"aligned ($\rightarrow$)")
plot_lobe(ax_u, [70, 80, 90, 100, 110],       r"aligned ($\uparrow$)")
plot_lobe(ax_b, [-10, 0, 10, 170, 180, 190],  r"bimodal ($\leftrightarrow$)")

fig.subplots_adjust(left=0.04, right=0.98, top=0.92, bottom=0.07)
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(OUTDIR, f"bsh_overview.{ext}"), dpi=200)
print("wrote", os.path.join(OUTDIR, "bsh_overview.pdf"))
