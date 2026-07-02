#!/usr/bin/env python3
r"""
fig_ringing.py  ->  figures/ringing_vs_smoothed.pdf  (Section 4, \label{fig:ringing})

Reconstruction of a single agent direction as a function of the angle gamma from that
direction. Plots the spherical Dirichlet kernel

        D_n(gamma) = sum_{l=0}^{n-1} (2l+1)/(4 pi) * P_l(cos gamma)

for n = 3 (the order used in the paper) and, as a reference, n = 8, to show that the
Dirac-kernel reconstruction overshoots at gamma = 0 and oscillates into negative side
lobes -- and that this ringing deepens with the band order. Against these, the smoothed
zonal reconstruction

        S_n(gamma) = sum_{l=0}^{n-1} lambda_l (2l+1)/(4 pi) * P_l(cos gamma),
        lambda_l   = exp(-sigma^2 l(l+1)/2)   (heat-kernel window, Section 4)

is a clean, non-negative lobe. Curves are normalised to unit peak so the shapes are
comparable (the Dirichlet peak grows like n^2/(4 pi)).

Run:  python3 fig_ringing.py
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
    "axes.linewidth": 0.9,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.03,
    "pdf.fonttype": 42, "ps.fonttype": 42,
})
OUTDIR = os.environ.get("FIGDIR", "figures")
os.makedirs(OUTDIR, exist_ok=True)

SIGMA = 0.80          # heat-kernel width (radians); chosen so S_3 is non-negative at n=3

# ------------------------------------------------------------- Legendre by recurrence
def legendre_all(nmax, x):
    P = [np.ones_like(x), x.copy()]
    for l in range(1, nmax):
        P.append(((2*l+1)*x*P[l] - l*P[l-1]) / (l+1))
    return P                                   # P[0..nmax]

def kernel(n, gamma, sigma=None):
    x = np.cos(gamma)
    P = legendre_all(n-1, x)
    s = np.zeros_like(x)
    for l in range(n):
        w = (2*l+1) / (4*np.pi)
        if sigma is not None:
            w *= np.exp(-sigma**2 * l*(l+1) / 2.0)
        s += w * P[l]
    return s

# --------------------------------------------------------------------------- figure
gamma = np.linspace(-np.pi, np.pi, 1441)
deg = np.degrees(gamma)

d3 = kernel(3, gamma);            d3 /= d3.max()
d8 = kernel(8, gamma);            d8 /= d8.max()
s3 = kernel(3, gamma, SIGMA);     s3 /= s3.max()

fig, ax = plt.subplots(figsize=(5.7, 3.3))

# shade the ringing (negative) region of the n=3 Dirac kernel
ax.fill_between(deg, d3, 0, where=(d3 < 0), color="#c1440e", alpha=0.14, linewidth=0)

ax.axhline(0, color="k", lw=0.7)
ax.plot(deg, d8, color="#9aa0a6", lw=1.1, ls="--", label=r"Dirac, $n=8$")
ax.plot(deg, d3, color="#1a1a1a", lw=1.8,           label=r"Dirac, $n=3$")
ax.plot(deg, s3, color="#c1440e", lw=1.8,
        label=r"smoothed, $n=3$ ($\sigma\approx%d^\circ$)" % round(np.degrees(SIGMA)))

ax.annotate("negative side lobes\n(ringing)", xy=(105, d3[np.argmin(np.abs(deg-105))]),
            xytext=(120, -0.42), fontsize=8.5, ha="left",
            arrowprops=dict(arrowstyle="->", color="#c1440e", lw=1.0))

ax.set_xlim(-180, 180); ax.set_ylim(-0.55, 1.05)
ax.set_xticks(np.arange(-180, 181, 60))
ax.set_xlabel(r"angle from agent direction $\gamma$ (deg)")
ax.set_ylabel(r"reconstructed value (unit peak)")
ax.legend(frameon=False, fontsize=8.5, loc="upper right")

fig.tight_layout()
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(OUTDIR, f"ringing_vs_smoothed.{ext}"), dpi=200)
print("wrote", os.path.join(OUTDIR, "ringing_vs_smoothed.pdf"))
