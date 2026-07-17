import math
import os

SDF_OUT = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/models/spacehopper/model.sdf"

# ── Geometry Constants ──────────────────────────────────────────────
CHASSIS_SIZE = 0.2
CHASSIS_Z = 0.25
CHASSIS_HALF = CHASSIS_SIZE / 2  # 0.1

THIGH_RADIUS = 0.015
THIGH_LENGTH = 0.15
THIGH_HALF = THIGH_LENGTH / 2

CALF_RADIUS = 0.01
CALF_LENGTH = 0.15
CALF_HALF = CALF_LENGTH / 2

HIP_PITCH = 1.2
KNEE_BEND = 0.8

RW_RADIUS = 0.06
RW_LENGTH = 0.02

# Raised 0.02 -> 0.025 (2026-07-15): larger foot spheres bridge heightmap
# crevices instead of sinking into them (part of the wedge-in jam fix, along
# with removing the thigh/calf cylinder collisions - see the leg section).
FOOT_RADIUS = 0.025

NUM_LEGS = 3

# ── Material helpers ────────────────────────────────────────────────
# Realism pass (2026-07-14): the model previously used flat ambient/diffuse/
# specular only (no PBR), which reads as painted plastic under Ogre2's
# lighting - flat, uniform, no metallic response - and was a big part of
# why the whole robot looked like a toy rather than engineered hardware
# (user feedback: "not as a kids toy... scientifically accurate and
# realistic"). mat() now always emits a PBR metal block with a sensible
# default so every existing call site benefits immediately; mat_pbr() below
# is for hero surfaces (MLI gold/silver foil, structural aluminum, carbon
# fiber, anodized joint housings) that need deliberately tuned
# metalness/roughness to actually read as their real-world material.
def mat_pbr(color, metalness=0.5, roughness=0.5, specular="0.5 0.5 0.5 1"):
    return f"""        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>{specular}</specular>
          <pbr>
            <metal>
              <metalness>{metalness}</metalness>
              <roughness>{roughness}</roughness>
            </metal>
          </pbr>
        </material>"""

def mat(color):
    return mat_pbr(color, metalness=0.5, roughness=0.5)

def mat_emissive(color, emissive):
    return f"""        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
          <emissive>{emissive}</emissive>
        </material>"""

# ── Inertia helpers ─────────────────────────────────────────────────
def inertia_block(m, ixx, iyy, izz):
    return f"""      <inertial>
        <mass>{m}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.6f}</iyy><iyz>0</iyz>
          <izz>{izz:.6f}</izz>
        </inertia>
      </inertial>"""

def box_inertia(m, sx, sy, sz):
    return inertia_block(m, m/12*(sy**2+sz**2), m/12*(sx**2+sz**2), m/12*(sx**2+sy**2))

def cyl_inertia(m, r, l):
    ixx = m/12*(3*r**2 + l**2)
    return inertia_block(m, ixx, ixx, m/2*r**2)

# ── Detail visual generators (visual-only, no collision/inertia) ───

def mli_panels():
    """Gold multi-layer insulation foil wrapping nearly every chassis face --
    real landers (MASCOT, Hayabusa2 itself) are wrapped almost entirely in
    gold MLI foil, not a couple of accent panels. Previously alternated
    gold/silver by face, which read as a much smaller/subtler amount of
    gold than what the user pictured for "real space bots"; now all 6
    faces are gold, and each panel covers more of its face (0.19->0.196)
    for a more fully "wrapped" look with only a thin structural edge
    showing (matches the panel_seams()/hull_bolt reveal at the edges)."""
    out = ""
    gold = "0.85 0.72 0.35 1"
    faces = [
        ("mli_top",    "0 0 0.1015",  "0.196 0.196 0.003"),
        ("mli_bottom", "0 0 -0.1015", "0.196 0.196 0.003"),
        ("mli_front",  "0.1015 0 0",  "0.003 0.196 0.196"),
        ("mli_back",   "-0.1015 0 0", "0.003 0.196 0.196"),
        ("mli_left",   "0 0.1015 0",  "0.196 0.003 0.196"),
        ("mli_right",  "0 -0.1015 0", "0.196 0.003 0.196"),
    ]
    for name, pose, size in faces:
        out += f"""
      <visual name="{name}">
        <pose>{pose} 0 0 0</pose>
        <geometry><box><size>{size}</size></box></geometry>
{mat_pbr(gold, metalness=0.9, roughness=0.15)}
      </visual>"""
    return out

def corner_brackets():
    """Small structural corner brackets at chassis edges."""
    out = ""
    bracket_size = 0.015
    offset = CHASSIS_HALF - bracket_size/2
    idx = 0
    for sx in [-1, 1]:
        for sy in [-1, 1]:
            for sz in [-1, 1]:
                x = sx * offset
                y = sy * offset
                z = sz * offset
                out += f"""
      <visual name="bracket_{idx}">
        <pose>{x} {y} {z} 0 0 0</pose>
        <geometry><box><size>{bracket_size} {bracket_size} {bracket_size}</size></box></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.8, roughness=0.3)}
      </visual>"""
                idx += 1
    return out

def panel_seams():
    """Recessed seam lines between structural panels. Real spacecraft
    chassis are assembled from multiple machined/composite panels, not one
    seamless molded box - visible panel seams are a strong "engineered
    hardware, not a toy" cue that a single flat-colored box completely
    lacks."""
    out = ""
    seam_color = "0.04 0.04 0.045 1"
    faces = [
        ("seam_front", f"{CHASSIS_HALF - 0.0005} 0 0.02", "0.001 0.196 0.003"),
        ("seam_back",  f"{-(CHASSIS_HALF - 0.0005)} 0 0.02", "0.001 0.196 0.003"),
        ("seam_left",  f"0 {CHASSIS_HALF - 0.0005} 0.02", "0.196 0.001 0.003"),
        ("seam_right", f"0 {-(CHASSIS_HALF - 0.0005)} 0.02", "0.196 0.001 0.003"),
    ]
    for name, pose, size in faces:
        out += f"""
      <visual name="{name}">
        <pose>{pose} 0 0 0</pose>
        <geometry><box><size>{size}</size></box></geometry>
{mat_pbr(seam_color, metalness=0.3, roughness=0.85)}
      </visual>"""
    return out

def hull_fasteners():
    """Small bolt heads around the front-face perimeter - the front face
    carries the cameras and reads as the robot's "face", so it's the highest-
    value place to show visible hardware assembly rather than a smooth
    molded shell."""
    out = ""
    bolt_r, bolt_len = 0.003, 0.004
    x = CHASSIS_HALF - 0.001
    edge = CHASSIS_HALF - 0.015
    pts = [(edge, edge), (-edge, edge), (edge, -edge), (-edge, -edge), (0, edge), (0, -edge)]
    for idx, (y, z) in enumerate(pts):
        out += f"""
      <visual name="hull_bolt_{idx}">
        <pose>{x} {y} {z} 0 1.5708 0</pose>
        <geometry><cylinder><radius>{bolt_r}</radius><length>{bolt_len}</length></cylinder></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.85, roughness=0.25)}
      </visual>"""
    return out

def antenna_mast():
    """UHF antenna mast on top of chassis."""
    return f"""
      <!-- UHF whip antenna: dark anodized mast, small flat-plate patch
           antenna instead of a glossy sphere tip (a bright white ball read
           as a toy topper - real whip antennas taper to a plain dark tip) -->
      <visual name="antenna_mast">
        <pose>0.06 0.06 0.122 0 0 0</pose>
        <geometry><cylinder><radius>0.0035</radius><length>0.044</length></cylinder></geometry>
{mat_pbr("0.12 0.12 0.13 1", metalness=0.7, roughness=0.35)}
      </visual>
      <visual name="antenna_base_collar">
        <pose>0.06 0.06 0.101 0 0 0</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.006</length></cylinder></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.8, roughness=0.3)}
      </visual>
      <visual name="antenna_tip">
        <pose>0.06 0.06 0.1445 0 0 0</pose>
        <geometry><cylinder><radius>0.0015</radius><length>0.001</length></cylinder></geometry>
{mat_pbr("0.05 0.05 0.05 1", metalness=0.6, roughness=0.5)}
      </visual>
      <!-- S-Band patch antenna (flat plate, per Research_Paper.md §4.2) -->
      <visual name="antenna_patch">
        <pose>-0.04 -0.06 0.101 0 0 0.4</pose>
        <geometry><box><size>0.03 0.03 0.003</size></box></geometry>
{mat_pbr("0.5 0.51 0.55 1", metalness=0.6, roughness=0.35)}
      </visual>
      <visual name="antenna_patch_frame">
        <pose>-0.04 -0.06 0.0995 0 0 0.4</pose>
        <geometry><box><size>0.034 0.034 0.001</size></box></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.75, roughness=0.3)}
      </visual>
      <!-- Comms status LED at the antenna base -->
      <visual name="comms_led">
        <pose>0.06 0.048 0.101 0 0 0</pose>
        <geometry><sphere><radius>0.005</radius></sphere></geometry>
{mat_emissive("0.0 0.4 0.8 1", "0.0 0.5 1.0 1")}
      </visual>
{led_light("comms_led_light", "0.06 0.048 0.101 0 0 0", "0.0 0.5 1.0 1", led_range=0.05)}"""

def headlight(name, pose, direction="1 0 0"):
    """Bright forward-facing floodlight (real spotlight, not a cosmetic
    LED). Ryugu's ambient light is set to near-zero (0.01 0.01 0.01) for
    scientific accuracy - no atmosphere means nothing scatters sunlight
    into shadow - so without local illumination the nav/hazcams would
    have nothing to image whenever the robot itself isn't in direct sun.
    Real landers operating in shadowed regions (e.g. lunar PSR concepts)
    carry floodlights for exactly this reason. Big attenuation range
    (1.2m) and a wide-ish cone so it actually lights up the ground the
    cameras are looking at, not just a tiny spot."""
    return f"""
      <light type="spot" name="{name}">
        <pose>{pose}</pose>
        <diffuse>1.0 0.98 0.9 1</diffuse>
        <specular>1.0 0.98 0.9 1</specular>
        <intensity>6.0</intensity>
        <direction>{direction}</direction>
        <attenuation>
          <range>1.2</range>
          <constant>0.1</constant>
          <linear>1.0</linear>
          <quadratic>1.5</quadratic>
        </attenuation>
        <spot>
          <inner_angle>0.4</inner_angle>
          <outer_angle>0.7</outer_angle>
          <falloff>1.0</falloff>
        </spot>
        <cast_shadows>false</cast_shadows>
        <visualize>false</visualize>
      </light>"""

def camera_housing():
    """Navigation camera on front face, plus two floodlights so the
    cameras have something to actually image away from direct sun."""
    return f"""
      <!-- Nav-cam floodlights (real spotlights, forward-facing) -->
{headlight("headlight_left", "0.11 0.06 0.03 0 0 0")}
{headlight("headlight_right", "0.11 -0.06 0.03 0 0 0")}
      <visual name="headlight_left_lens">
        <pose>0.113 0.06 0.03 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.006</radius><length>0.004</length></cylinder></geometry>
{mat_emissive("0.9 0.9 0.8 1", "1.0 1.0 0.9 1")}
      </visual>
      <visual name="headlight_right_lens">
        <pose>0.113 -0.06 0.03 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.006</radius><length>0.004</length></cylinder></geometry>
{mat_emissive("0.9 0.9 0.8 1", "1.0 1.0 0.9 1")}
      </visual>""" + f"""
      <!-- Navigation camera -->
      <visual name="camera_body">
        <pose>0.105 0 0.02 0 0 0</pose>
        <geometry><box><size>0.015 0.025 0.025</size></box></geometry>
{mat_pbr("0.06 0.06 0.07 1", metalness=0.55, roughness=0.45)}
      </visual>
      <visual name="camera_lens">
        <pose>0.114 0 0.02 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.003</length></cylinder></geometry>
{mat_pbr("0.02 0.02 0.06 1", metalness=0.9, roughness=0.1)}
      </visual>
      <!-- Science camera (bottom-mounted, downward-facing) -->
      <visual name="sci_camera_body">
        <pose>0 -0.04 -0.105 0 0 0</pose>
        <geometry><box><size>0.02 0.02 0.015</size></box></geometry>
{mat_pbr("0.06 0.06 0.07 1", metalness=0.55, roughness=0.45)}
      </visual>
      <visual name="sci_camera_lens">
        <pose>0 -0.04 -0.114 0 0 0</pose>
        <geometry><cylinder><radius>0.007</radius><length>0.003</length></cylinder></geometry>
{mat_pbr("0.02 0.02 0.06 1", metalness=0.9, roughness=0.1)}
      </visual>
      <!-- Stereo hazard-avoidance cameras (small, flanking the nav camera -
           real rovers/hoppers pair a wide nav cam with a small stereo set
           for local depth/obstacle sensing, e.g. Perseverance's Hazcams) -->
      <visual name="hazcam_left_body">
        <pose>0.100 0.035 0.02 0 0 0</pose>
        <geometry><box><size>0.008 0.012 0.012</size></box></geometry>
{mat_pbr("0.06 0.06 0.07 1", metalness=0.55, roughness=0.45)}
      </visual>
      <visual name="hazcam_left_lens">
        <pose>0.107 0.035 0.02 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.004</radius><length>0.002</length></cylinder></geometry>
{mat_pbr("0.02 0.02 0.06 1", metalness=0.9, roughness=0.1)}
      </visual>
      <visual name="hazcam_right_body">
        <pose>0.100 -0.035 0.02 0 0 0</pose>
        <geometry><box><size>0.008 0.012 0.012</size></box></geometry>
{mat_pbr("0.06 0.06 0.07 1", metalness=0.55, roughness=0.45)}
      </visual>
      <visual name="hazcam_right_lens">
        <pose>0.107 -0.035 0.02 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.004</radius><length>0.002</length></cylinder></geometry>
{mat_pbr("0.02 0.02 0.06 1", metalness=0.9, roughness=0.1)}
      </visual>"""

def thermal_louvers():
    """Thermal dissipation louvers on chassis side."""
    out = ""
    num_louvers = 5
    spacing = 0.03
    y_start = -0.06
    for i in range(num_louvers):
        y = y_start + i * spacing
        out += f"""
      <visual name="louver_{i}">
        <pose>-0.102 {y} 0.03 0 0 0</pose>
        <geometry><box><size>0.003 0.025 0.005</size></box></geometry>
{mat_pbr("0.4 0.41 0.44 1", metalness=0.7, roughness=0.3)}
      </visual>"""
    return out

def led_light(name, pose, diffuse, led_range=0.12, intensity=3.0):
    """Real point light so an LED actually illuminates its surroundings -
    emissive material alone only colors the sphere itself, it doesn't cast
    light into the scene. First attempt at this (earlier 2026-07-14) was
    reverted because Gazebo's GUI draws a wireframe gizmo at every light's
    position by default, and 6 small lights produced a confusing criss-
    crossing mesh with no obvious way to turn it off. Root cause found:
    sdf::Light has a real, documented <visualize> element (SDF 1.10 spec,
    default true) specifically for this - "If true, the light is
    visualized in the GUI". Setting it false suppresses the gizmo without
    affecting the light's actual illumination. <cast_shadows> stays false
    to keep the render cost of many small lights low.
    """
    return f"""
      <light type="point" name="{name}">
        <pose>{pose}</pose>
        <diffuse>{diffuse}</diffuse>
        <specular>{diffuse}</specular>
        <intensity>{intensity}</intensity>
        <attenuation>
          <range>{led_range}</range>
          <constant>0.15</constant>
          <linear>2.5</linear>
          <quadratic>6</quadratic>
        </attenuation>
        <cast_shadows>false</cast_shadows>
        <visualize>false</visualize>
      </light>"""

def status_led():
    """Status LED strip on chassis face - each LED is a small emissive
    sphere plus a matching point light so it actually lights up, not just
    looks colored."""
    out = ""
    colors = [
        ("0.0 0.8 0.0 1", "0.0 1.0 0.0 1"),  # green - power
        ("0.0 0.4 0.8 1", "0.0 0.5 1.0 1"),  # blue - comms
        ("0.8 0.6 0.0 1", "1.0 0.8 0.0 1"),  # amber - science
    ]
    for i, (color, emissive) in enumerate(colors):
        x = -0.06 + i * 0.025
        pose = f"{x} 0.102 -0.06 0 0 0"
        out += f"""
      <visual name="led_{i}">
        <pose>{pose}</pose>
        <geometry><sphere><radius>0.007</radius></sphere></geometry>
{mat_emissive(color, emissive)}
      </visual>
{led_light(f"led_{i}_light", pose, emissive)}"""
    return out

def battery_visuals():
    """12 Ni-MH cell visuals."""
    out = ""
    x0, y0 = -0.04, -0.06
    for i in range(3):
        for j in range(4):
            x = x0 + i * 0.04
            y = y0 + j * 0.04
            out += f"""
      <visual name="battery_{i}_{j}">
        <pose>{x} {y} 0.05 0 0 0</pose>
        <geometry><cylinder><radius>0.015</radius><length>0.06</length></cylinder></geometry>
{mat("0.8 0.4 0.1 1")}
      </visual>"""
    return out

def rw_housing():
    """Protective ring housings around each reaction wheel, each with a
    small status LED (real point light) so the trio reads as active
    avionics rather than blank metal rings."""
    out = f"""
      <!-- RW protective housings -->
      <visual name="rw_housing_x">
        <pose>0 0 0 0 {math.pi/2} 0</pose>
        <geometry><cylinder><radius>0.065</radius><length>0.005</length></cylinder></geometry>
{mat_pbr("0.2 0.2 0.22 1", metalness=0.75, roughness=0.3)}
      </visual>
      <visual name="rw_housing_y">
        <pose>0 0 0 {math.pi/2} 0 0</pose>
        <geometry><cylinder><radius>0.065</radius><length>0.005</length></cylinder></geometry>
{mat_pbr("0.2 0.2 0.22 1", metalness=0.75, roughness=0.3)}
      </visual>
      <visual name="rw_housing_z">
        <pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>0.065</radius><length>0.005</length></cylinder></geometry>
{mat_pbr("0.2 0.2 0.22 1", metalness=0.75, roughness=0.3)}
      </visual>"""
    rw_leds = [
        ("rw_led_x", "0.066 0 0 0 1.5708 0", "0.0 0.5 1.0 1"),
        ("rw_led_y", "0 0.066 0 1.5708 0 0", "0.0 0.5 1.0 1"),
        ("rw_led_z", "0 0 0.066 0 0 0", "0.0 0.5 1.0 1"),
    ]
    for name, pose, color in rw_leds:
        out += f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><sphere><radius>0.006</radius></sphere></geometry>
{mat_emissive(color, color)}
      </visual>
{led_light(f"{name}_light", pose, color, led_range=0.05)}"""
    return out

def drill_housing():
    """Fixed turret/mounting bracket for the drill, attached to base_link
    (unlike the drill shaft itself, which is on the moving drill_link).
    Found 2026-07-14: the drill shaft was a bare rigid rod that only
    translates within its 0.1m joint range - even at the "retracted" rest
    position it still hung fully exposed below the chassis, since nothing
    ever visually contained it. This turret gives the mechanism something to
    disappear into, and drill_link's rest pose (see DRILL_REST_Z below) was
    raised to tuck the shaft up inside it.
    """
    return f"""
      <!-- ── Drill mounting turret (fixed housing, shaft retracts up into it) ── -->
      <visual name="drill_turret">
        <pose>0 0 {-CHASSIS_HALF - 0.0125} 0 0 0</pose>
        <geometry><cylinder><radius>0.028</radius><length>0.035</length></cylinder></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.75, roughness=0.35)}
      </visual>
      <visual name="drill_turret_plate">
        <pose>0 0 {-CHASSIS_HALF - 0.002} 0 0 0</pose>
        <geometry><box><size>0.06 0.06 0.004</size></box></geometry>
{mat_pbr("0.25 0.25 0.28 1", metalness=0.6, roughness=0.4)}
      </visual>
      <!-- Science/sampler status LED -->
      <visual name="drill_led">
        <pose>0.024 0.024 {-CHASSIS_HALF - 0.003} 0 0 0</pose>
        <geometry><sphere><radius>0.006</radius></sphere></geometry>
{mat_emissive("0.8 0.6 0.0 1", "1.0 0.8 0.0 1")}
      </visual>
{led_light("drill_led_light", f"0.024 0.024 {-CHASSIS_HALF - 0.003} 0 0 0", "1.0 0.8 0.0 1", led_range=0.05)}"""

def leg_cable_harness(length, tube_radius, cable_radius=0.0025):
    """Thin cable/wire conduit running along a leg tube, offset just off its
    surface - real actuated legs always carry visible external wiring to
    the joint motors. A perfectly smooth, unadorned leg tube is one of the
    clearest "toy" tells: nothing to route wiring through internally at this
    scale, and no visible wiring externally either."""
    offset = tube_radius + cable_radius + 0.001
    return f"""
      <visual name="cable_harness">
        <pose>{offset} 0 {length / 2} 0 0 0</pose>
        <geometry><cylinder><radius>{cable_radius}</radius><length>{length * 0.85}</length></cylinder></geometry>
{mat_pbr("0.03 0.03 0.03 1", metalness=0.1, roughness=0.7)}
      </visual>"""

def joint_housing(name, pose):
    """Motor housing cylinder at a joint location."""
    return f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.02</length></cylinder></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.75, roughness=0.35)}
      </visual>"""


# ════════════════════════════════════════════════════════════════════
#  BUILD THE SDF
# ════════════════════════════════════════════════════════════════════

sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="spacehopper">
    <!-- Ryugu's gravity (0.000114 m/s^2) produces velocities small enough that DART's
         default auto-sleep threshold puts the body to sleep mid-flight, permanently
         freezing it in the air. Must stay awake for micro-gravity hops to ever land. -->
    <allow_auto_disable>false</allow_auto_disable>

    <!-- ══════════ BASE CHASSIS ══════════ -->
    <link name="base_link">
      <pose>0 0 {CHASSIS_Z} 0 0 0</pose>
{box_inertia(1.35, CHASSIS_SIZE, CHASSIS_SIZE, CHASSIS_SIZE)}

      <!-- Main hull - brushed aluminum structural panel -->
      <visual name="hull_visual">
        <geometry><box><size>{CHASSIS_SIZE} {CHASSIS_SIZE} {CHASSIS_SIZE}</size></box></geometry>
{mat_pbr("0.3 0.31 0.35 1", metalness=0.75, roughness=0.28)}
      </visual>
{panel_seams()}
{hull_fasteners()}

      <!-- ── MLI thermal blankets ── -->
{mli_panels()}

      <!-- ── Structural corner brackets ── -->
{corner_brackets()}

      <!-- ── Antenna ── -->
{antenna_mast()}

      <!-- ── Cameras ── -->
{camera_housing()}

      <!-- ── Thermal louvers ── -->
{thermal_louvers()}

      <!-- ── Status LEDs ── -->
{status_led()}

      <!-- Internal visuals removed to prevent clipping -->

      <!-- ── Reaction wheel housings ── -->
{rw_housing()}

      <!-- ── Drill mounting turret ── -->
{drill_housing()}

      <!-- ── Chassis collision ── -->
      <collision name="collision">
        <geometry><box><size>{CHASSIS_SIZE} {CHASSIS_SIZE} {CHASSIS_SIZE}</size></box></geometry>
      </collision>

      <!-- ── IMU sensor ── -->
      <!-- No explicit <topic> override: an explicit "imu" topic is treated as a
           literal, unscoped gz-transport topic name, so it published to global
           /imu on every spawned instance - ros_gz_bridge was listening on the
           per-entity-scoped /model/<name>/imu, which never matched, so IMU data
           silently never reached attitude_controller.py or landing_controller.py
           (found 2026-07-14: their state machines never progressed past FLIGHT
           because they're entirely IMU-driven). Omitting <topic> lets gz-sim
           auto-generate the properly scoped per-entity default topic, matching
           how the joint controllers and odometry publisher already behave.
           launch.py's bridge config was updated to match the auto-generated name. -->
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <visualize>true</visualize>
      </sensor>

      <!-- LIDAR removed to prevent performance stuttering -->
    </link>

    <!-- Was "ignition-gazebo-imu-system" / "ignition::gazebo::systems::Imu" (old
         v6/Citadel-era naming). This machine has both the old v6 and current v8
         (Harmonic) IMU plugin libraries installed side-by-side, and gz-sim
         silently loaded the old v6 one instead of erroring - a real version
         mismatch against the v8 engine everything else here runs on (physics,
         odometry, joint controllers all already used gz-sim8/"gz::sim::systems"
         naming). That mismatch is the root cause of "Unknown message type"
         warnings from ros_gz_bridge and why IMU data never actually reached
         attitude_controller.py/landing_controller.py despite the sensor
         "working" and the topic existing. Found & fixed 2026-07-14. -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <!-- Real position/velocity feedback - previously nonexistent, which is why
         swarm_manager.py had to literally assume the robot was always at the origin
         when computing distance/yaw to a target, and why monitor_height.py/
         monitor_joints.py (which expect this) were dead code. -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <!-- Generic frame labels (was scout_1/... - cosmetic strings in the
           odometry message header; nothing consumes TF frames in this
           project, and the TOPIC is derived from the spawned entity name at
           runtime, so one model.sdf serves all three scouts). -->
      <odom_frame>spacehopper/odom</odom_frame>
      <robot_base_frame>spacehopper/base</robot_base_frame>
      <odom_publish_frequency>20</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>

    <!-- NOTE (2026-07-16): a JointStatePublisher plugin briefly lived here
         for the zero-stiffness landing catch. REMOVED along with the catch:
         it publishes every physics step (measured 515 Hz on the wire), and
         three Python subscribers deserializing that pinned every landing
         controller at 100% CPU (imu_callback throughput collapsed from
         ~100 Hz to 2.6 Hz - the whole landing state machine froze). If
         joint-state telemetry is ever needed again, it MUST be rate-limited
         or consumed in C++, not Python. -->

    <!-- ══════════ SOLAR PANEL ARRAY (fixed to chassis top) ══════════ -->
    <link name="solar_panel">
      <pose>{0} {0} {CHASSIS_Z + CHASSIS_HALF + 0.005} 0 0 0</pose>
{box_inertia(0.15, 0.18, 0.18, 0.005)}
      <visual name="panel_surface">
        <geometry><box><size>0.18 0.18 0.005</size></box></geometry>
{mat_pbr("0.03 0.05 0.22 1", metalness=0.7, roughness=0.15)}
      </visual>
      <!-- Solar cell grid lines -->
      <visual name="cell_grid_h1">
        <pose>0 0 0.003 0 0 0</pose>
        <geometry><box><size>0.18 0.001 0.001</size></box></geometry>
{mat_pbr("0.75 0.76 0.78 1", metalness=0.9, roughness=0.2)}
      </visual>
      <visual name="cell_grid_h2">
        <pose>0 0.045 0.003 0 0 0</pose>
        <geometry><box><size>0.18 0.001 0.001</size></box></geometry>
{mat_pbr("0.75 0.76 0.78 1", metalness=0.9, roughness=0.2)}
      </visual>
      <visual name="cell_grid_h3">
        <pose>0 -0.045 0.003 0 0 0</pose>
        <geometry><box><size>0.18 0.001 0.001</size></box></geometry>
{mat_pbr("0.75 0.76 0.78 1", metalness=0.9, roughness=0.2)}
      </visual>
      <visual name="cell_grid_v1">
        <pose>0.045 0 0.003 0 0 0</pose>
        <geometry><box><size>0.001 0.18 0.001</size></box></geometry>
{mat_pbr("0.75 0.76 0.78 1", metalness=0.9, roughness=0.2)}
      </visual>
      <visual name="cell_grid_v2">
        <pose>-0.045 0 0.003 0 0 0</pose>
        <geometry><box><size>0.001 0.18 0.001</size></box></geometry>
{mat_pbr("0.75 0.76 0.78 1", metalness=0.9, roughness=0.2)}
      </visual>
    </link>
    <joint name="solar_panel_joint" type="fixed">
      <parent>base_link</parent>
      <child>solar_panel</child>
    </joint>

    <!-- ══════════ REACTION WHEELS (inside chassis) ══════════ -->
    <link name="rw_x">
      <pose>{0} {0} {CHASSIS_Z} 0 {math.pi/2} 0</pose>
{cyl_inertia(0.15, RW_RADIUS, RW_LENGTH)}
      <visual name="visual">
        <geometry><cylinder><radius>{RW_RADIUS}</radius><length>{RW_LENGTH}</length></cylinder></geometry>
{mat("0.35 0.1 0.1 1")}
      </visual>
    </link>
    <joint name="rw_x_joint" type="revolute">
      <parent>base_link</parent>
      <child>rw_x</child>
      <pose relative_to="rw_x">0 0 0 0 0 0</pose>
      <axis><xyz expressed_in="__model__">1 0 0</xyz></axis>
    </joint>

    <link name="rw_y">
      <pose>{0} {0} {CHASSIS_Z} {math.pi/2} 0 0</pose>
{cyl_inertia(0.15, RW_RADIUS, RW_LENGTH)}
      <visual name="visual">
        <geometry><cylinder><radius>{RW_RADIUS}</radius><length>{RW_LENGTH}</length></cylinder></geometry>
{mat("0.1 0.35 0.1 1")}
      </visual>
    </link>
    <joint name="rw_y_joint" type="revolute">
      <parent>base_link</parent>
      <child>rw_y</child>
      <pose relative_to="rw_y">0 0 0 0 0 0</pose>
      <axis><xyz expressed_in="__model__">0 1 0</xyz></axis>
    </joint>

    <link name="rw_z">
      <pose>{0} {0} {CHASSIS_Z} 0 0 0</pose>
{cyl_inertia(0.15, RW_RADIUS, RW_LENGTH)}
      <visual name="visual">
        <geometry><cylinder><radius>{RW_RADIUS}</radius><length>{RW_LENGTH}</length></cylinder></geometry>
{mat("0.1 0.1 0.35 1")}
      </visual>
    </link>
    <joint name="rw_z_joint" type="revolute">
      <parent>base_link</parent>
      <child>rw_z</child>
      <pose relative_to="rw_z">0 0 0 0 0 0</pose>
      <axis><xyz expressed_in="__model__">0 0 1</xyz></axis>
    </joint>

    <!-- ══════════ DRILL (prismatic, underneath chassis) ══════════ -->
    <!-- Rest pose (joint value 0) tucks the shaft's center 0.03m *above* the
         chassis bottom face, inside the drill_turret housing above - so at
         rest the shaft's bottom edge sits just past the turret's lower rim
         (mostly hidden) instead of the old pose, which hung the entire
         0.1m shaft fully exposed below the chassis even when "retracted". -->
    <link name="drill_link">
      <pose>0 0 {CHASSIS_Z - CHASSIS_HALF + 0.03} 0 0 0</pose>
{cyl_inertia(0.25, 0.015, 0.1)}
      <visual name="drill_shaft">
        <geometry><cylinder><radius>0.012</radius><length>0.1</length></cylinder></geometry>
{mat("0.6 0.6 0.6 1")}
      </visual>
      <visual name="drill_bit">
        <pose>0 0 -0.05 0 0 0</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.02</length></cylinder></geometry>
{mat("0.8 0.8 0.8 1")}
      </visual>
      <visual name="drill_collar">
        <pose>0 0 0.04 0 0 0</pose>
        <geometry><cylinder><radius>0.018</radius><length>0.015</length></cylinder></geometry>
{mat("0.3 0.3 0.35 1")}
      </visual>
    </link>
    <joint name="drill_joint" type="prismatic">
      <parent>base_link</parent>
      <child>drill_link</child>
      <pose relative_to="drill_link">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-0.1</lower><upper>0.0</upper></limit>
        <!-- Spring-to-rest at 0 (fully retracted/flush with chassis) so the drill
             doesn't passively drift/dangle under flight or landing shocks when
             uncommanded - it was previously unconstrained and had no actuator at
             all, which is why it looked detached from the body. Stiffness/damping
             raised 2026-07-15 (0.05/0.01 -> 0.2/0.05) after live observation: the
             drill visibly jiggled up/down mid-flight while the commanded position
             stayed "retracted" the whole time - the same class of issue as the
             false-landing-detection bug (RW/leg-motor reaction torque produces real
             transient body accelerations that a too-soft passive spring doesn't
             resist well). -->
        <dynamics><spring_reference>0</spring_reference><spring_stiffness>0.2</spring_stiffness><damping>0.05</damping></dynamics>
      </axis>
    </joint>
"""

# ══════════ LEGS ══════════
for i in range(NUM_LEGS):
    angle = i * (2 * math.pi / NUM_LEGS)
    cx = math.cos(angle)
    cy = math.sin(angle)

    hip_x = cx * 0.07
    hip_y = cy * 0.07
    hip_z = CHASSIS_Z - CHASSIS_HALF  # bottom face of chassis

    thigh = f"thigh_{i}"
    calf = f"calf_{i}"

    sdf += f"""
    <!-- ── LEG {i} ── -->
    <link name="{thigh}">
      <pose>{hip_x} {hip_y} {hip_z} 0 {HIP_PITCH} {angle}</pose>
{cyl_inertia(0.05, THIGH_RADIUS, THIGH_LENGTH)}
      <visual name="visual">
        <pose>0 0 {THIGH_HALF} 0 0 0</pose>
        <geometry><cylinder><radius>{THIGH_RADIUS}</radius><length>{THIGH_LENGTH}</length></cylinder></geometry>
{mat_pbr("0.12 0.12 0.14 1", metalness=0.35, roughness=0.5)}
      </visual>
{leg_cable_harness(THIGH_LENGTH, THIGH_RADIUS)}
      <!-- Hip motor housing -->
      <visual name="hip_housing_{i}">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry><cylinder><radius>0.022</radius><length>0.02</length></cylinder></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.8, roughness=0.25)}
      </visual>
      <!-- Hip actuator status LED (emissive sphere + a real point light so
           it's actually visible in Ryugu's near-total darkness, not just a
           colored-but-unlit surface) -->
      <visual name="hip_led_{i}">
        <pose>0.026 0 0.01 0 0 0</pose>
        <geometry><sphere><radius>0.006</radius></sphere></geometry>
{mat_emissive("0.0 0.8 0.0 1", "0.0 1.0 0.0 1")}
      </visual>
{led_light(f"hip_led_{i}_light", "0.026 0 0.01 0 0 0", "0.0 1.0 0.0 1")}
      <!-- Thigh collision removed 2026-07-15: foot-only ground contact
           (standard legged-sim practice). The thin thigh/calf cylinder
           collisions catching in heightmap crevices were the root cause of
           the recurring "wedge-in" jam - legs pinned so hard the 0.134 Nm
           actuators could not move them at all, silently zeroing all jump
           thrust. Stance loads go through the foot spheres; the chassis
           box still protects the body. -->
    </link>
    <joint name="hip_joint_{i}" type="revolute">
      <parent>base_link</parent>
      <child>{thigh}</child>
      <pose relative_to="{thigh}">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-3.14</lower><upper>3.14</upper></limit>
        <!-- Real joint damping added 2026-07-15: with none, the leg position
             PIDs behave as near-lossless springs at ground contact, and in
             micro-gravity the robot bounced for tens of minutes at 3-5 mm/s
             (observed live: z oscillating 4.81-4.99 m with ~2% energy loss
             per cycle, impacts too soft for spike detection, too mobile for
             rest detection).
             Raised 0.005 -> 0.15 on 2026-07-16: with the p=1.0 launch-
             authority gains, the legs became stiff springs at touchdown and
             measured contact restitution was ~0.96 (bounces from a 1.15 m
             drop did not decay; every active-control mitigation - stepped,
             ramped, or measured-angle-mirroring posture commands - ADDED
             energy, the last one because bridged joint-state feedback lags
             and pumps the rebound). Physical damping is phase-perfect by
             construction. Sizing: effective vertical leg stiffness at
             p=1.0 is ~48 N/m, so zeta = c_vert/(2*sqrt(k*m)) with
             c_vert ~= 3*c_joint/r^2 (r~0.25 m) gives zeta ~0.45 and
             restitution ~0.2 at c_joint = 0.15 N m s/rad -> bounces decay
             in 2 cycles. DO NOT raise further: p=5.0 + damping=0.4 was
             tried 2026-07-16 and froze the leg joints entirely (joints
             pinned at ~0 rad, stand-fold and crouch both produced zero
             motion, live joint_states-verified) - suspected DART explicit
             joint-damping discretization limit. p=1.0 + 0.15 is the
             verified-stable operating point. Launch cost is small because separation joint
             speed is only ~0.16 rad/s: 0.15*0.16 = 0.024 N m, ~18% of the
             0.134 N m actuator budget. -->
        <dynamics><damping>0.05</damping></dynamics>
      </axis>
    </joint>

    <link name="{calf}">
      <pose relative_to="{thigh}">0 0 {THIGH_LENGTH} 0 {KNEE_BEND} 0</pose>
{cyl_inertia(0.05, CALF_RADIUS, CALF_LENGTH)}
      <visual name="visual">
        <pose>0 0 {CALF_HALF} 0 0 0</pose>
        <geometry><cylinder><radius>{CALF_RADIUS}</radius><length>{CALF_LENGTH}</length></cylinder></geometry>
{mat_pbr("0.32 0.33 0.37 1", metalness=0.65, roughness=0.35)}
      </visual>
{leg_cable_harness(CALF_LENGTH, CALF_RADIUS)}
      <!-- Knee motor housing -->
      <visual name="knee_housing_{i}">
        <pose>0 0 0.005 0 0 0</pose>
        <geometry><cylinder><radius>0.018</radius><length>0.015</length></cylinder></geometry>
{mat_pbr("0.16 0.16 0.18 1", metalness=0.8, roughness=0.25)}
      </visual>
      <!-- Knee actuator status LED -->
      <visual name="knee_led_{i}">
        <pose>0.021 0 0.005 0 0 0</pose>
        <geometry><sphere><radius>0.005</radius></sphere></geometry>
{mat_emissive("0.0 0.8 0.0 1", "0.0 1.0 0.0 1")}
      </visual>
{led_light(f"knee_led_{i}_light", "0.021 0 0.005 0 0 0", "0.0 1.0 0.0 1", led_range=0.05)}
      <!-- Foot pad -->
      <visual name="foot_{i}">
        <pose>0 0 {CALF_LENGTH} 0 0 0</pose>
        <geometry><sphere><radius>{FOOT_RADIUS}</radius></sphere></geometry>
{mat_pbr("0.4 0.41 0.43 1", metalness=0.4, roughness=0.6)}
      </visual>
      <!-- Foot proximity/contact sensor lens -->
      <visual name="foot_sensor_{i}">
        <pose>0 0 {CALF_LENGTH - 0.03} 0 0 0</pose>
        <geometry><sphere><radius>0.005</radius></sphere></geometry>
{mat("0.05 0.05 0.15 1")}
      </visual>
      <!-- Calf cylinder collision removed 2026-07-15 (see thigh note):
           foot-only ground contact eliminates the wedge-in jam class. -->
      <collision name="foot_collision">
        <pose>0 0 {CALF_LENGTH} 0 0 0</pose>
        <geometry><sphere><radius>{FOOT_RADIUS}</radius></sphere></geometry>
      </collision>
    </link>
    <joint name="knee_joint_{i}" type="revolute">
      <parent>{thigh}</parent>
      <child>{calf}</child>
      <pose relative_to="{calf}">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-3.14</lower><upper>3.14</upper></limit>
      <!-- Damping raised 0.00001 -> 0.005 (2026-07-15), same rationale as the
           hip joint above: dissipate landing-contact energy so micro-gravity
           touchdowns settle in a couple of bounce cycles instead of ringing
           near-losslessly for tens of minutes. -->
      <dynamics><spring_reference>0</spring_reference><spring_stiffness>0.00028</spring_stiffness><damping>0.05</damping><friction>0.00001</friction></dynamics></axis>
    </joint>
"""

# ── Joint controller plugins ──
sdf += """
    <!-- ══════════ JOINT CONTROLLER PLUGINS ══════════ -->"""
for i in range(NUM_LEGS):
    for jt in ["hip", "knee"]:
        sdf += f"""
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{jt}_joint_{i}</joint_name>
      <!-- Gains raised 2026-07-15 (p 0.05 -> 1.0, d 0.01 -> 0.05) after the
           full-stroke liftoff test: at p=0.05 the legs extend at ~4 mm/s
           under body load (verified: clean ballistic arc, separation
           velocity ~3.5-4 mm/s, apex +53 mm matching v^2/2g exactly), which
           is 5x short of the 18.5 mm/s a 3 m hop needs. The launch stroke
           is rate-limited by controller authority, not by the motor: the
           0.134 Nm cmd_max (Maxon RE 13 + 1:67 gearhead spec, DO NOT raise)
           permits ~25x more torque than p=0.05 demands at typical stroke
           errors. p=1.0 makes the stroke torque-cap-limited (bang-bang
           style), which physics predicts delivers ~0.3 m/s of separation
           velocity headroom; d=0.05 keeps the unloaded joint response from
           ringing (zeta ~0.35 vs joint inertia). -->
      <p_gain>1.0</p_gain><i_gain>0</i_gain><d_gain>0.05</d_gain>
      <cmd_max>0.134</cmd_max><cmd_min>-0.134</cmd_min>
    </plugin>"""

for axis in ["x", "y", "z"]:
    # NOTE: "gz-sim-joint-velocity-controller-system" is not a real plugin in this
    # gz-sim install (load fails silently -> reaction wheels never spin). The correct
    # plugin is JointController, matching the RW torque limit (0.015 Nm) from
    # Research_Paper.md.
    sdf += f"""
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>rw_{axis}_joint</joint_name>
      <p_gain>0.05</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
      <cmd_max>0.015</cmd_max><cmd_min>-0.015</cmd_min>
    </plugin>"""

# drill_joint was previously unactuated entirely - no controller plugin at all --
# which is why it could passively drift/dangle and look detached from the body,
# especially under flight/landing shocks. cmd_max of 0.5 N is an estimate for a
# small linear feed mechanism on a ~250g corer assembly (no hardware spec given in
# Research_Paper.md for linear extension force; the paper only specifies rotary
# drilling RPM, not feed force).
sdf += """
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>drill_joint</joint_name>
      <!-- Gains raised 2026-07-15 (p=2.0/d=0.1/cmd_max=0.5 -> p=8.0/d=0.3/
           cmd_max=1.0) after live observation of visible mid-flight jiggle
           at the commanded "retracted" position - see the spring/damping
           note on drill_joint's <dynamics> above for the root-cause story. -->
      <p_gain>8.0</p_gain><i_gain>0</i_gain><d_gain>0.3</d_gain>
      <cmd_max>1.0</cmd_max><cmd_min>-1.0</cmd_min>
    </plugin>"""

sdf += """

  </model>
</sdf>
"""

with open(SDF_OUT, "w") as f:
    f.write(sdf)

print(f"Successfully wrote SDF to {SDF_OUT}")
