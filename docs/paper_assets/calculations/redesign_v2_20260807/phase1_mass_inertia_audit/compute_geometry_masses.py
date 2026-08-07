#!/usr/bin/env python3
"""Phase 1 mass/inertia audit -- computed-from-geometry rows only (reaction
wheels, chassis, legs). Datasheet/vendor-sourced rows (motors, cells,
avionics, payload, solar/antenna/camera/MLI) are not computed here; their
numbers and sources are cited directly in AUDIT_TABLE.md.

Every material density used below is a real, named, cited value (see
comments at each use site). Every geometric dimension not taken directly
from the current model.sdf is an explicit engineering assumption, flagged
as such in both this script's output and AUDIT_TABLE.md -- per the Phase 1
instruction not to let assumptions quietly become "real" numbers.

Run: python3 compute_geometry_masses.py
"""
import math

# ---------------------------------------------------------------------------
# Reaction wheel: annulus (rim), not solid disc. This is the confirmed-wrong
# component (current model.sdf models it as a solid cylinder, r=0.06m,
# L=0.02m, m=0.15kg -- implied density 663 kg/m^3, which is not any real
# structural material; closer to softwood than metal).
#
# Real cubesat/smallsat reaction-wheel practice uses a stainless-steel or
# tungsten rim/spoked flywheel for a large inertia-to-mass ratio (source:
# NASA GSFC CubeSat symposium poster "Reaction Wheel for CubeSat Attitude
# Control"; ASPINA "Reaction wheel for satellite and CubeSat" design note).
# Stainless steel (304) is chosen here as the more practical/lower-cost
# choice of the two documented options; tungsten is noted as a real
# alternative if a later phase needs more inertia per unit mass.
STEEL_DENSITY = 8000.0  # kg/m^3, austenitic stainless steel (304), standard
                         # engineering reference value

# ASSUMPTION (flagged): outer radius kept close to the current model's 0.06m
# visual envelope (fits the existing chassis clearance), reduced slightly to
# 0.05m; rim wall thickness 8mm and axial length 10mm are engineering
# assumptions (compact, manufacturable proportions), not vendor dimensions --
# there is no COTS part for a bespoke 2.5kg-platform reaction wheel.
rw_r_outer = 0.050   # m
rw_wall = 0.008       # m
rw_r_inner = rw_r_outer - rw_wall
rw_length = 0.010     # m

rw_volume = math.pi * (rw_r_outer**2 - rw_r_inner**2) * rw_length
rw_mass = rw_volume * STEEL_DENSITY
rw_izz = 0.5 * rw_mass * (rw_r_outer**2 + rw_r_inner**2)          # spin axis
rw_ixx = rw_mass * (3 * (rw_r_outer**2 + rw_r_inner**2) + rw_length**2) / 12  # transverse

print("=== Reaction wheel (annulus, stainless steel) ===")
print(f"  r_outer={rw_r_outer*1000:.1f}mm  r_inner={rw_r_inner*1000:.1f}mm  "
      f"length={rw_length*1000:.1f}mm  density={STEEL_DENSITY:.0f} kg/m^3")
print(f"  mass = {rw_mass:.4f} kg ({rw_mass*1000:.1f} g)")
print(f"  I_zz (spin axis)  = {rw_izz:.6e} kg*m^2")
print(f"  I_xx (transverse) = {rw_ixx:.6e} kg*m^2")
print(f"  [current model.sdf: solid disc, m=0.15kg, I_zz=2.70e-4 kg*m^2, "
      f"implied density=663 kg/m^3 -- not a real material]")
print()

# ---------------------------------------------------------------------------
# Chassis: "Aluminum 7075-T6 core, CFRP structural panels" per Table I.
# Current model.sdf hull is a 0.2x0.2x0.2m box link but the paper never
# claims it's solid (a solid 7075-T6 cube would be 0.2^3 * 2810 = 22.5kg).
# Computed here as a thin CFRP outer skin over a thin 7075-T6 aluminum edge
# frame -- both real, named materials; panel thickness and frame
# cross-section are explicit engineering assumptions (flagged), not
# vendor/CAD-sourced dimensions.
AL_7075_DENSITY = 2810.0   # kg/m^3 (matmatch.com / theworldmaterial.com,
                            # standard reference value for 7075-T6)
CFRP_DENSITY = 1600.0      # kg/m^3, mid-range of typical structural CFRP
                            # (1400-1900 kg/m^3 range; sciencedirect/
                            # chinacarbonfibers.com references)

chassis_side = 0.2   # m, matches current model.sdf hull box
panel_thickness = 0.001   # m, 1mm CFRP skin -- ASSUMPTION
frame_edge = 0.004         # m, 4x4mm 7075-T6 square-section edge frame -- ASSUMPTION

panel_area = 6 * chassis_side**2
panel_volume = panel_area * panel_thickness
panel_mass = panel_volume * CFRP_DENSITY

frame_length = 12 * chassis_side   # 12 edges of a cube
frame_volume = frame_length * frame_edge**2
frame_mass = frame_volume * AL_7075_DENSITY

chassis_mass = panel_mass + frame_mass

print("=== Chassis (CFRP skin + 7075-T6 edge frame) ===")
print(f"  CFRP panels: {panel_area:.4f} m^2 x {panel_thickness*1000:.1f}mm "
      f"x {CFRP_DENSITY:.0f} kg/m^3 = {panel_mass:.4f} kg")
print(f"  Al 7075-T6 frame: {frame_length:.3f} m of {frame_edge*1000:.0f}x"
      f"{frame_edge*1000:.0f}mm edge x {AL_7075_DENSITY:.0f} kg/m^3 = {frame_mass:.4f} kg")
print(f"  TOTAL chassis mass = {chassis_mass:.4f} kg  "
      f"(Table I currently claims 0.70 kg)")
print()

# ---------------------------------------------------------------------------
# Legs: thigh/calf as thin-wall CFRP tubes (real material; specific wall
# thickness is an ASSUMPTION -- no vendor part for a bespoke leg tube).
# Outer radii taken directly from model.sdf visual geometry (thigh 15mm,
# calf 10mm, both 150mm long) -- those ARE real/current dimensions, not
# assumed.
leg_wall = 0.001  # m, 1mm CFRP wall -- ASSUMPTION

def tube_mass(r_outer, wall, length, density):
    r_inner = r_outer - wall
    volume = math.pi * (r_outer**2 - r_inner**2) * length
    return volume * density, r_inner

thigh_r_outer, thigh_len = 0.015, 0.150   # m, from model.sdf
calf_r_outer, calf_len = 0.010, 0.150     # m, from model.sdf

thigh_mass, thigh_r_inner = tube_mass(thigh_r_outer, leg_wall, thigh_len, CFRP_DENSITY)
calf_mass, calf_r_inner = tube_mass(calf_r_outer, leg_wall, calf_len, CFRP_DENSITY)

print("=== Legs (CFRP tube, per segment) ===")
print(f"  thigh: r_outer={thigh_r_outer*1000:.1f}mm r_inner={thigh_r_inner*1000:.1f}mm "
      f"L={thigh_len*1000:.0f}mm -> {thigh_mass:.4f} kg ({thigh_mass*1000:.1f} g)")
print(f"  calf:  r_outer={calf_r_outer*1000:.1f}mm r_inner={calf_r_inner*1000:.1f}mm "
      f"L={calf_len*1000:.0f}mm -> {calf_mass:.4f} kg ({calf_mass*1000:.1f} g)")
print(f"  [current model.sdf: both segments hardcoded to 0.05 kg each, "
      f"solid-cylinder geometry]")
print(f"  6-segment (3x thigh + 3x calf) total: "
      f"{3*(thigh_mass+calf_mass):.4f} kg  (current model.sdf total: 0.300 kg)")
