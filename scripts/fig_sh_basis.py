#!/usr/bin/env python3
r"""
fig_sh_basis.py  ->  figures/sh_basis.pdf   (Figure for Section 3, \label{fig:sh-basis})

Renders the nine real spherical-harmonic basis functions of the first three bands
(l = 0, 1, 2), i.e. y_0 ... y_8, as signed lobe surfaces r = |y_l^m| coloured by sign.
This is the journal counterpart of Table 2 in the 2015 conference version.

The Cartesian forms below are the orthonormal real SH used throughout the paper and are
identical to the constants hard-coded in the SHEval3 kernel (Section 6); regenerating the
figure from the same basis the implementation uses is intentional.

Run:  python3 fig_sh_basis.py        (writes figures/sh_basis.pdf and .png)
"""
import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

# --------------------------------------------------------------------------- style
mpl.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.size": 10,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
    "pdf.fonttype": 42,   # embed editable TrueType text in the PDF
    "ps.fonttype": 42,
})
OUTDIR = os.environ.get("FIGDIR", "figures")
os.makedirs(OUTDIR, exist_ok=True)

# --------------------------------------------- orthonormal real SH, bands l = 0,1,2
_C = [
    0.50 * np.sqrt(1.0 / np.pi),   # i0  (l,m)=(0, 0)      constant
    0.50 * np.sqrt(3.0 / np.pi),   # i1  (1,-1)  * y
    0.50 * np.sqrt(3.0 / np.pi),   # i2  (1, 0)  * z
    0.50 * np.sqrt(3.0 / np.pi),   # i3  (1, 1)  * x
    0.50 * np.sqrt(15.0 / np.pi),  # i4  (2,-2)  * xy
    0.50 * np.sqrt(15.0 / np.pi),  # i5  (2,-1)  * yz
    0.25 * np.sqrt(5.0 / np.pi),   # i6  (2, 0)  * (3z^2 - 1)
    0.50 * np.sqrt(15.0 / np.pi),  # i7  (2, 1)  * xz
    0.25 * np.sqrt(15.0 / np.pi),  # i8  (2, 2)  * (x^2 - y^2)
]

def real_sh(i, x, y, z):
    """Evaluate the i-th real SH (flat index i = l(l+1)+m) on the unit sphere."""
    x = np.asarray(x, float); y = np.asarray(y, float); z = np.asarray(z, float)
    return [
        _C[0] * np.ones_like(x),
        _C[1] * y,
        _C[2] * z,
        _C[3] * x,
        _C[4] * x * y,
        _C[5] * y * z,
        _C[6] * (3.0 * z * z - 1.0),
        _C[7] * x * z,
        _C[8] * (x * x - y * y),
    ][i]

LABELS = [
    r"$y_0^{0}=y_0$",
    r"$y_1^{-1}=y_1$", r"$y_1^{0}=y_2$", r"$y_1^{1}=y_3$",
    r"$y_2^{-2}=y_4$", r"$y_2^{-1}=y_5$", r"$y_2^{0}=y_6$",
    r"$y_2^{1}=y_7$",  r"$y_2^{2}=y_8$",
]
# triangular layout on a 3x5 grid (row-major subplot indices)
POS = [3, 7, 8, 9, 11, 12, 13, 14, 15]

# ------------------------------------------------------------------------ rendering
theta = np.linspace(0.0, np.pi, 60)
phi = np.linspace(0.0, 2.0 * np.pi, 120)
TH, PH = np.meshgrid(theta, phi)
SX = np.sin(TH) * np.cos(PH)
SY = np.sin(TH) * np.sin(PH)
SZ = np.cos(TH)

cmap = plt.get_cmap("RdBu_r")

fig = plt.figure(figsize=(7.4, 4.7))
for k, i in enumerate(range(9)):
    ax = fig.add_subplot(3, 5, POS[k], projection="3d")
    val = real_sh(i, SX, SY, SZ)
    r = np.abs(val)
    X, Y, Z = r * SX, r * SY, r * SZ
    vmax = max(np.max(r), 1e-9)
    norm = mpl.colors.Normalize(vmin=-vmax, vmax=vmax)
    facecolors = cmap(norm(val))
    ax.plot_surface(X, Y, Z, facecolors=facecolors, rstride=1, cstride=1,
                    linewidth=0, antialiased=True, shade=False)
    lim = vmax * 1.02
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.view_init(elev=22, azim=35)
    ax.set_axis_off()
    ax.set_title(LABELS[i], pad=-2, fontsize=10)

fig.subplots_adjust(left=0.01, right=0.99, top=0.98, bottom=0.01, wspace=0.0, hspace=0.05)
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(OUTDIR, f"sh_basis.{ext}"), dpi=200)
print("wrote", os.path.join(OUTDIR, "sh_basis.pdf"))
