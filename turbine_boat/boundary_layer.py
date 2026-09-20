#!/usr/bin/env python3
"""
Atmospheric boundary layer wind profile over water.
Log-law profile showing why turbine height above the surface matters.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Log-law parameters ────────────────────────────────────────────────────────
z0    = 0.0002   # roughness length, open water (m)
z_ref = 10.0     # anemometer reference height (m)
U_ref = 5.144    # 10 knots in m/s

def U(z):
    return U_ref * np.log(np.maximum(z, z0) / z0) / np.log(z_ref / z0)

# ── Operating points ──────────────────────────────────────────────────────────
z_low  = 0.30   # turbine height, low trim  (m)
z_high = 1.50   # turbine height, high trim (m)

MS_TO_KT = 1.94384

# ── Colors (match website palette) ───────────────────────────────────────────
BG    = '#0f0e0c'
PAPER = '#1a1814'
INK   = '#e8e0d0'
MUTED = '#7a7060'
GOLD  = '#d4a843'
BLUE  = '#4a8fa8'
RULE  = '#2e2a24'
WATER = '#1a2a38'

# ── Figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(5.5, 6.5), facecolor=BG)
ax.set_facecolor(PAPER)

z_range = np.linspace(0.005, 5.0, 800)
U_kts   = U(z_range) * MS_TO_KT

# Water surface band
ax.axhspan(-0.15, 0.0, color=WATER, zorder=2)
ax.text(0.4, -0.08, 'water surface', color='#4a8fa8', fontsize=8,
        fontfamily='monospace', va='center', zorder=3)

# Boundary layer shading
ax.fill_betweenx(z_range, 0, U_kts, where=(z_range <= 1.5),
                 color=BLUE, alpha=0.06, zorder=1)

# Wind profile curve
ax.plot(U_kts, z_range, color=BLUE, linewidth=2, zorder=4)

# Reference wind (10 kt at 10 m — shown as right-edge label)
ax.annotate('10 kt at 10 m\n(reference)', xy=(10.0, 5.0),
            xytext=(7.8, 4.55), fontsize=8, color=MUTED,
            fontfamily='monospace',
            arrowprops=dict(arrowstyle='->', color=MUTED, lw=0.8))

# ── Low trim point ────────────────────────────────────────────────────────────
U_low_kt = U(z_low) * MS_TO_KT
ax.hlines(z_low, 0, U_low_kt, colors=MUTED, linewidth=1,
          linestyle='--', zorder=5)
ax.vlines(U_low_kt, -0.15, z_low, colors=MUTED, linewidth=1,
          linestyle='--', zorder=5)
ax.plot(U_low_kt, z_low, 'o', color=MUTED, markersize=7, zorder=6)
ax.text(U_low_kt - 0.2, z_low + 0.14,
        f'low trim · {z_low*100:.0f} cm · {U_low_kt:.1f} kt',
        color=MUTED, fontsize=8.5, fontfamily='monospace', ha='right', zorder=7)

# ── High trim point ───────────────────────────────────────────────────────────
U_high_kt = U(z_high) * MS_TO_KT
ax.hlines(z_high, 0, U_high_kt, colors=GOLD, linewidth=1,
          linestyle='--', zorder=5)
ax.vlines(U_high_kt, -0.15, z_high, colors=GOLD, linewidth=1,
          linestyle='--', zorder=5)
ax.plot(U_high_kt, z_high, 'o', color=GOLD, markersize=7, zorder=6)
ax.text(U_high_kt + 0.2, z_high - 0.2,
        f'high trim · {z_high*100:.0f} cm · {U_high_kt:.1f} kt',
        color=GOLD, fontsize=8.5, fontfamily='monospace', ha='left', zorder=7)

# ── Speed difference annotation ───────────────────────────────────────────────
delta = U_high_kt - U_low_kt
ax.annotate('', xy=(U_high_kt, z_low + 0.05),
            xytext=(U_low_kt, z_low + 0.05),
            arrowprops=dict(arrowstyle='<->', color=INK, lw=1.0))
ax.text((U_low_kt + U_high_kt) / 2, z_low + 0.16,
        f'+{delta:.1f} kt', color=INK, fontsize=8,
        fontfamily='monospace', ha='center')

# ── Axes ──────────────────────────────────────────────────────────────────────
ax.set_xlim(0, 11)
ax.set_ylim(-0.15, 5.0)
ax.set_xlabel('Wind speed  (knots)', color=INK, fontsize=10,
              fontfamily='monospace')
ax.set_ylabel('Height above water  (m)', color=INK, fontsize=10,
              fontfamily='monospace')
ax.tick_params(colors=MUTED, labelsize=8.5)
ax.set_xticks(range(0, 12, 2))
ax.set_yticks([0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0])
for spine in ax.spines.values():
    spine.set_color(RULE)
ax.grid(color=RULE, linewidth=0.5, alpha=0.6, zorder=0)

# Tiny footnote
fig.text(0.5, 0.005,
         'Log-law profile · z₀ = 0.2 mm (open water) · U_ref = 10 kt at 10 m',
         color=MUTED, fontsize=7, fontfamily='monospace', ha='center')

plt.tight_layout(rect=[0, 0.02, 1, 1])
plt.savefig('/home/ross/BusinessWebsite/lab/turbine_boat_assets/boundary_layer.png',
            dpi=150, bbox_inches='tight', facecolor=BG)
print("Saved boundary_layer.png")
