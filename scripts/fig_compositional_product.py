#!/usr/bin/env python3
r"""
fig_compositional_product.py -> figures/compositional_product.pdf
                                 (Section 5, \label{fig:compositional})

Compositional behaviour as a product of directional preference fields. Reads left to
right as the equation "within cone C  x  follow B  x  avoid A (complement)  =  product":
the composite is appreciable only in directions favoured by all three factors at once.

Each factor is drawn as an equatorial polar field in [0,1]; these smooth, band-limited
shapes stand in for order-n BSH fields, and the product is the pointwise product (the
SH-domain form uses the triple-product tensor, Eq. for the SH product in Section 5).

Run:  python3 fig_compositional_product.py
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
    "savefig.pad_inches": 0.03,
    "pdf.fonttype": 42, "ps.fonttype": 42,
})
OUTDIR = os.environ.get("FIGDIR", "figures")
os.makedirs(OUTDIR, exist_ok=True)

FCOL = "#33628f"      # factor fields
PCOL = "#c1440e"      # product field

def angdiff(a, b):
    return (a - b + np.pi) % (2*np.pi) - np.pi

def raised_cos_lobe(phi, center, width):
    """Bump peaking at 1 at `center`, tapering to 0 at +/- `width`, 0 beyond."""
    d = np.abs(angdiff(phi, np.radians(center)))
    w = np.radians(width)
    out = 0.5 * (1 + np.cos(np.pi * d / w))
    out[d > w] = 0.0
    return out

def cosine_field(phi, center):
    """Broad attraction lobe in [0,1], peaking at 1 toward `center`."""
    return 0.5 * (1 + np.cos(angdiff(phi, np.radians(center))))

# --------------------------------------------------------------- factor definitions
phi = np.linspace(0, 2*np.pi, 720)
chi_C = raised_cos_lobe(phi, center=50, width=75)              # "within cone C"
f_B   = cosine_field(phi, center=70)                          # "follow B"
f_A   = raised_cos_lobe(phi, center=140, width=55)            # presence of A
avoid_A = 1.0 - f_A                                           # "avoid A" (complement)
prod = chi_C * f_B * avoid_A                                  # composite

panels = [
    (chi_C,   r"within cone $C$",            FCOL),
    (f_B,     r"follow $B$",                 FCOL),
    (avoid_A, r"avoid $A$ (complement)",     FCOL),
    (prod,    r"product",                    PCOL),
]

# --------------------------------------------------------------------------- figure
fig = plt.figure(figsize=(7.6, 2.45))
axes = [fig.add_subplot(1, 4, k + 1, projection="polar") for k in range(4)]
for ax, (field, title, col) in zip(axes, panels):
    ax.plot(phi, field, color=col, lw=1.4)
    ax.fill(phi, field, color=col, alpha=0.22)
    ax.set_ylim(0, 1.05)
    ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_yticks([])
    ax.grid(alpha=0.25, lw=0.5)
    ax.set_title(title, fontsize=9, pad=8)

# operator glyphs between panels
for x, sym in [(0.278, r"$\times$"), (0.503, r"$\times$"), (0.728, r"$=$")]:
    fig.text(x, 0.47, sym, fontsize=16, ha="center", va="center")

fig.subplots_adjust(left=0.01, right=0.99, top=0.86, bottom=0.02, wspace=0.55)
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(OUTDIR, f"compositional_product.{ext}"), dpi=200)
print("wrote", os.path.join(OUTDIR, "compositional_product.pdf"))
