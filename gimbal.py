"""Pan/tilt camera gimbal.
  Base (free-standing): MG996R pan servo + 0.96" SSD1306 OLED.
  Yoke over the base:   MG90S tilt servo on the right arm, 608ZZ bearing on the left arm.
  Head (one piece):     Freenove ESP32-S3 WROOM CAM + 12 mm laser. Board slides in from the bottom,
                        its WROOM antenna tab keys into a slot at the top; a screwed floor plate clamps it.

Run:  python gimbal.py
  -> STL/ printed parts, STL/fit_test/ coupons (print first), STEP/ parts + gimbal_assembly.step
     (bought parts named BUY_*), plus self-checks:
       1. no collisions at pan 0/+-90, tilt 0/+-45 (printed + bought parts, headers/Dupont space)
       2. assembly paths are clear: board slides up into the head, laser slides into its tube,
          MG90S pushes into the yoke, head drops into the yoke.
All dimensions mm. MEASURE = check against your real part, edit, re-run.
"""
import os
import tempfile
import cadquery as cq

OUT = os.path.dirname(os.path.abspath(__file__))
CLR = 0.3  # clearance per side for sliding fits

# ---------------- bought parts ----------------
# MG996R (pan) - TowerPro datasheet + fit test 1
# measured (round 3): a = 28.6 bottom->ear underside, b = 3.0 ear, c = 8.5 ear top->case top, s = 9.5, holes 48 x 9.5
SB_L, SB_W, SB_CASE_H = 40.3, 20.0, 28.6 + 3.0 + 8.5
SB_SHAFT_FROM_END = 9.5
SB_EAR_Z, SB_EAR_T, SB_EAR_SPAN = 28.6, 3.0, 53.6
SB_EAR_PITCH, SB_EAR_ACROSS = 48.0, 9.5
SB_RIB_GAP = 2.4               # triangular rib on each ear between the holes (1 mm thick, 3 long, 2.5 tall)
SB_FIT = 0.1                   # deck cut-out clearance per side (fit test 2: 0.3 was loose both ways)
SB_HORN_FACE = 9.0             # measured e: case top to the top flat face of the fitted round horn disc
SB_HORN_D, SB_HORN_T = 20.6, 2.6   # measured round horn disc
SB_HUB_D, SB_HUB_TOP = 10.0, 1.0   # hub on the disc's outer face (1 mm measured; 3 mm hub is on the spline side)

# MG90S (tilt) - TowerPro datasheet + FreeCAD SG90 model (vendor_models/) + fit test 1
SS_L, SS_W = 22.5, 12.0
SS_SHAFT_FROM_END = 5.9
SS_EAR_PITCH, SS_EAR_SPAN = 27.8, 32.4
SS_EAR_BOT, SS_EAR_T = 15.9, 2.5
SS_EARTOP_TO_HORN_FACE = 12.5  # MEASURE: ear top face to the OUTER flat face of the fitted horn arm
SS_FIT = 0.2                   # window clearance per side (fit test 1: +-1.5 play was too loose)
SS_HORN_EXTRA_L = 2.0          # fit test 2: horn pocket needed 2 mm more length
# measured double-arm horn: 35.8 long, 7.6 wide at centre, 2.0 thick arm, hub 7.2 dia, hubs 1.5 / 2.0 high
SS_HORN_T, SS_HORN_L = 2.0, 35.8
SS_HUB_D, SS_HUB_OUT, SS_HUB_IN = 8.0, 0.6, 3.0   # measured j, k (screw side), m (servo side)
SS_HORN_W0, SS_HORN_W1 = 7.6, 3.4            # width at centre / at tips (tip from FreeCAD horn, scaled)

# Freenove ESP32-S3 WROOM CAM (FNK0085), layout from Freenove's interface diagram
PCB_W, PCB_L, PCB_T = 28.2, 49.8, 1.6     # measured: 56.8 overall incl. 7 mm antenna tab
ANT_L, MOD_W, MOD_L = 7.0, 18.0, 25.5   # WROOM module overhangs the PCB top edge by ANT_L (antenna tab)
CAM_FROM_ANT = 18.0            # measured: lens centre below the antenna end
LENS_H, CAM_BOX = 6.0, 8.0     # measured: PCB front face to lens tip; camera block 8 x 8
HDR_X, HDR_Z0 = 12.7, (49.8 - 19 * 2.54) / 2   # header rows at +-HDR_X, centred on the PCB
USB_X, USB_W = 5.4, 8.9        # two USB-C at the bottom edge
BTN_X, BTN_Z = 8.4, 12.4       # BOOT/RESET buttons
FRONT_GAP = LENS_H + 2.0       # lens clears the front wall while the board slides in
BACK_GAP = 20.0                # header 2.5 + Dupont housing 14 + bend

# 0.96" SSD1306 4-pin module, common size 27.3 x 27.8 (MEASURE: old 25.0 x 27.2 did not fit).
# Board rises between side rails, then pushes forward onto 4 thin pegs in its holes; glass flat on the wall.
OLED_W, OLED_H = 27.3, 27.8                 # MEASURE board width x height (pins on the top edge)
OLED_HX, OLED_HZ = 20.5, 23.0               # hole spacing (web: SSD1306 module footprint; old 19.5 x 22.5 jammed)
OLED_HOLE_D, OLED_PEG_D = 2.0, 1.5          # 0.25 play per side so hole-position error doesn't jam it
OLED_GW, OLED_GH = 26.7, 19.3               # MEASURE glass width x height
OLED_PCB_T, OLED_GLASS_T = 1.2, 1.6         # MEASURE board, glass thickness (sum was 2.8)
OLED_E = 3.5                                # MEASURE: board top (pin) edge to glass top edge
OLED_PINS = 8.5
OLED_WIN_W, OLED_WIN_H = 24.0, 14.0         # window inside the glass: +-1.3 / +-2.6 slack for glass position
OLED_WIN_DZ = (OLED_H / 2 - OLED_E) - OLED_GH / 2   # window centred on the glass

LASER_D = 12.1 + 0.6           # laser 12.1 x 34.6 (previous Onshape CAD) + clearance (fit test 1 was tight)
B608_OD, B608_ID, B608_W = 22.0, 8.0, 7.0
B608_FIT = 0.175                # pocket clearance per side (fit test 1: 0.1 was very tight)
AXLE_D = 7.95                  # printed D-shaped axle pin in the 608 (M3-only hardware); M3 screw locks it

# power, all inside the base (ASSUMED sizes - MEASURE each and re-run)
BAT_L, BAT_W, BAT_H = 70.0, 55.5, 25.0      # measured: 3S Li-ion pack incl. BMS (max), lies under the MG996R
BAT_GAP = 1.5                               # pack top to MG996R bottom (put a 1-2 mm foam pad here)
DCDC_L, DCDC_W, DCDC_T, DCDC_PARTS = 22.0, 17.0, 1.6, 5.0   # 5 V buck (MP1584 / Mini560 size): PCB L x W x T, height incl. parts
WAGO_L, WAGO_W, WAGO_T = 18.7, 13.1, 8.4    # 2x WAGO 221-413 (5 V bus + GND bus): length (wire entry), width, thickness
WAGO_XY = (25.0, -23.0)
ELCAP_D, ELCAP_L, ELCAP_XY = 10.0, 20.0, (-25.0, -23.0)   # 1000 uF 16 V across the 5 V bus
SW_CUT_W, SW_CUT_H, SW_DEPTH = 13.0, 19.0, 26.0   # KCD1 rocker: panel cut-out, depth behind bezel incl. spade terminals
JACK_D, JACK_DEPTH = 8.0, 20.0              # DC-022B 5.5x2.1 charge jack: M8 thread, body + solder tags behind panel

# ---------------- head (tilt-axis frame: origin on axis, -Y front, +Z up) ----------------
WALL, SIDE, FLOOR_T = 2.0, 8.0, 2.5
IN_W = PCB_W + 1.0
IN_D = FRONT_GAP + PCB_T + BACK_GAP
HW, HD = IN_W + 2 * SIDE, IN_D + 2 * WALL
HH = FLOOR_T + PCB_L + ANT_L + 0.5 + WALL
Y_FRONT, Y_BACK = -HD / 2, HD / 2
Y_PCB = Y_FRONT + WALL + FRONT_GAP
Z_FLOOR = -HH / 2 + FLOOR_T    # floor plate top = board bottom edge
PCB_TOP = Z_FLOOR + PCB_L
CAM_Z = PCB_TOP + ANT_L - CAM_FROM_ANT
RIDGE_R = LASER_D / 2 + 2.0
LASER_Z = HH / 2 + LASER_D / 2 + 1.0
RIDGE_L = 36.0
FLOOR_SCREWS = [(sx * (IN_W / 2 + SIDE / 2), sy * 10.0) for sx in (-1, 1) for sy in (-1, 1)]

# ---------------- yoke ----------------
ARM_T, ARM_BASE, ARM_TOP_R = 8.0, 20.0, 16.0
X_ARM_R_OUT = HW / 2 - (SS_HORN_T + 0.2) + SS_EARTOP_TO_HORN_FACE   # horn outer face sits on the pocket floor
ARM_T_R = ARM_T - 1.0          # right arm 1 mm thinner: MG90S horn hub (3 mm) clears it while the head is lowered in
X_ARM_R_IN = X_ARM_R_OUT - ARM_T_R
GAP_L, RING_T, HUB_T = 1.5, 1.2, 2.5
X_ARM_L_IN = -HW / 2 - GAP_L
X_ARM_L_OUT = X_ARM_L_IN - ARM_T
Z_PLATE_TOP = -38.0
DECK_GAP = 3.0                 # yoke underside to deck (round 2: yoke touched the base)
SB_POCKET = SB_HORN_FACE - DECK_GAP            # disc pocket depth from the yoke underside
PLATE_T = SB_POCKET + SB_HUB_TOP + 0.3 + 3.5
Z_PLATE_BOT = Z_PLATE_TOP - PLATE_T
SS_ZC = SS_SHAFT_FROM_END - SS_L / 2
CAP_WALL = 1.6
CAP_D = SS_EAR_T + SS_EAR_BOT + 1.0 + CAP_WALL
CAP_HW, CAP_HZ = SS_W / 2 + 0.4 + CAP_WALL, SS_EAR_SPAN / 2 + 0.4 + CAP_WALL

# ---------------- base (base frame: origin floor centre, pan axis = Z) ----------------
BASE_R, BASE_WALL, DECK_T, BOTTOM_T = 50.0, 3.0, 3.0, 2.5
DECK_TOP = BOTTOM_T + BAT_H + BAT_GAP + SB_CASE_H   # battery floor, MG996R above it, case top flush with the deck
DECK_IN = DECK_TOP - DECK_T                         # deck underside
OLED_YF, OLED_T = -(BASE_R - 4), 3.0
OLED_OZ = DECK_IN - 0.3 - OLED_H / 2                # board top edge stops under the deck
OLED_GZ = OLED_OZ + OLED_WIN_DZ                     # glass / window centre
BACK_YF, BACK_T = BASE_R - 6, 2.0                   # flat back panel, 2 mm so the rocker clips snap
SW_X, JACK_X, PANEL_Z = -11.0, 12.0, DECK_TOP - 18
HARNESS_Y = 38.5                                    # deck exit for the base->head harness, just outside the yoke sweep
COL_A = (60, 120, 240, 300)                         # bottom-plate screw columns, clear of the battery corners
DCDC_X = -(BASE_R - BASE_WALL) + 4.4                # buck pad face on the left wall
DCDC_Z0 = DECK_IN - 6 - DCDC_L                      # 6 mm under the deck for the wire pads
YOKE_Z = DECK_TOP + DECK_GAP - Z_PLATE_BOT


BRAND, BRAND_DEPTH = "ANVYXO", 0.6       # engraved, 3 layers deep
BRAND_FONT = r"C:/Windows/Fonts/ariblk.ttf"


def brand(plane, size):
    """Engraving solid for the brand text on `plane` (text reads from outside, cuts BRAND_DEPTH into the part)."""
    return cq.Workplane(plane).text(BRAND, size, -BRAND_DEPTH - 0.01, fontPath=BRAND_FONT, halign="center", valign="center")


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def boxes_x(xa, xb, *rest):
    """Same box mirrored left/right (xa, xb > 0)."""
    return box(xa, xb, *rest).union(box(-xb, -xa, *rest))


def rrect(plane, origin, center, w, h, r, depth):
    return (cq.Workplane(plane, origin=origin).center(*center)
            .sketch().rect(w, h).vertices().fillet(r).finalize().extrude(depth))


def soft(wp, fn):
    """Cosmetic edge op; keep the part if OCC can't do it."""
    try:
        r = fn(wp)
        return r if r.val().isValid() else wp
    except Exception:
        return wp


# =============================== printed parts ===============================
def head_outer():
    h = cq.Workplane("XY").box(HW, HD, HH).edges("|Y").fillet(6.0)
    h = soft(h, lambda w: w.faces("<Y").edges().chamfer(1.2))
    return soft(h, lambda w: w.faces(">Y").edges().chamfer(1.2))


def head():
    """One piece. Print camera face down. Inside roof (the back wall) bridges 29 mm."""
    h = head_outer().intersect(box(-HW, HW, -HD, HD, Z_FLOOR, HH))
    h = h.cut(box(-IN_W / 2, IN_W / 2, Y_FRONT + WALL, Y_BACK - WALL, Z_FLOOR - 1, PCB_TOP + 0.5))
    # antenna-tab slot: locates the board's top end in x, y and z
    h = h.cut(box(-MOD_W / 2 - CLR, MOD_W / 2 + CLR, Y_PCB - 1.6, Y_PCB + 0.8, PCB_TOP, PCB_TOP + ANT_L + 0.5))
    h = h.cut(box(-MOD_W / 2 - 1.0, MOD_W / 2 + 1.0, Y_PCB - 2.6, Y_PCB + 1.8, PCB_TOP, PCB_TOP + 1.5))   # lead-in

    # laser ridge: tube blended into the top with 4 mm concave fillets, covers the whole module
    zt, f, r, zl = HH / 2, 4.0, RIDGE_R, LASER_Z
    c = f * 0.7071
    ridge = (cq.Workplane("XZ", origin=(0, Y_FRONT, 0))
             .moveTo(-14, zt - WALL).lineTo(14, zt - WALL).lineTo(14, zt).lineTo(r + f, zt)
             .threePointArc((r + f - c, zt + f - c), (r, zt + f)).lineTo(r, zl)
             .threePointArc((0, zl + r), (-r, zl)).lineTo(-r, zt + f)
             .threePointArc((-r - f + c, zt + f - c), (-r - f, zt)).lineTo(-14, zt).close()
             .extrude(-RIDGE_L))
    ridge = soft(ridge, lambda w: w.faces("<Y or >Y").edges().chamfer(0.8))
    h = h.union(ridge)
    h = h.cut(cq.Workplane("XZ", origin=(0, Y_FRONT - 1, 0)).moveTo(0, zl).circle(LASER_D / 2).extrude(-(RIDGE_L + 2)))
    h = h.cut(cq.Workplane().add(cq.Solid.makeCone(LASER_D / 2 + 0.8, LASER_D / 2, 0.8,
                                                   cq.Vector(0, Y_FRONT, zl), cq.Vector(0, 1, 0))))
    h = h.cut(cq.Workplane("XY").circle(1.3).extrude(12).translate((0, Y_FRONT + RIDGE_L / 2, zl)))   # M3 grub
    h = h.cut(cq.Workplane("XY").circle(2.5).extrude(zl - PCB_TOP + 1).translate((0, Y_BACK - WALL - 4, PCB_TOP - 0.5)))  # laser wires into head

    # camera window, 45 deg chamfer outside. No bezel recess: printed face-down its ledge hung in mid-air.
    c = 1.0
    h = h.cut(cq.Workplane("XZ", origin=(0, Y_FRONT - 0.01, 0)).center(0, CAM_Z).rect(18 + 2 * c, 13 + 2 * c)
              .workplane(offset=-c - 0.01).rect(18, 13).loft())
    h = h.cut(box(-9, 9, Y_FRONT - 1, Y_FRONT + WALL + 1, CAM_Z - 6.5, CAM_Z + 6.5))
    # back vents
    h = h.cut(cq.Workplane("XZ", origin=(0, Y_BACK + 1, 0)).pushPoints([(0, 20 - 4 * i) for i in range(4)])
              .slot2D(16, 2.2).extrude(WALL + 2))
    # brand on the front, under the camera
    h = h.cut(brand(cq.Plane(origin=(0, Y_FRONT - 0.01, (Z_FLOOR + CAM_Z - 6.5 - c) / 2), xDir=(1, 0, 0), normal=(0, -1, 0)), 6.0))

    # left wall: 608ZZ pocket (teardrop roof toward +Y = up when printed face-down) + 1 mm lip
    rb = B608_OD / 2 + B608_FIT
    tear = (cq.Workplane("YZ").circle(rb).extrude(B608_W)
            .union(cq.Workplane("YZ").polyline([(rb * 0.7071, rb * 0.7071), (rb * 1.4142, 0),
                                                (rb * 0.7071, -rb * 0.7071)]).close().extrude(B608_W))
            .intersect(box(-1, B608_W + 1, -50, Y_BACK - WALL - 1.3, -50, 50)))
    h = h.cut(tear.translate((-HW / 2, 0, 0)))
    h = h.cut(cq.Workplane("YZ", origin=(-HW / 2 - 1, 0, 0)).circle(5).extrude(SIDE + 2))

    # right wall: double-arm horn pocket + 2 screw slots (self-tappers fit any horn hole pattern)
    hd = SS_HORN_T + 0.2                       # pocket depth (arm 2.0 + 0.2)
    w0, w1, L = SS_HORN_W0 + 0.4, SS_HORN_W1 + 0.4, SS_HORN_L + 0.6 + SS_HORN_EXTRA_L
    zt2 = L / 2 - w1 / 2
    wp = lambda: cq.Workplane("YZ", origin=(HW / 2 - hd, 0, 0))
    hp = (wp().polyline([(w0 / 2, 0), (w1 / 2, zt2), (-w1 / 2, zt2), (-w0 / 2, 0),
                         (-w1 / 2, -zt2), (w1 / 2, -zt2)]).close().extrude(hd + 1)
          .union(wp().circle(max(w0, SS_HUB_D + 0.8) / 2).extrude(hd + 1))
          .union(wp().pushPoints([(0, zt2), (0, -zt2)]).circle(w1 / 2).extrude(hd + 1)))
    h = h.cut(hp)
    # raised hub on the horn's outer face (measured 1.5 / 2.0: recess sized for the taller one)
    h = h.cut(cq.Workplane("YZ", origin=(HW / 2 - hd - SS_HUB_OUT - 0.4, 0, 0)).circle(SS_HUB_D / 2 + 0.4).extrude(SS_HUB_OUT + 0.41))
    h = h.cut(cq.Workplane("YZ", origin=(HW / 2 - hd - 4.5, 0, 0)).pushPoints([(0, 11), (0, -11)])
              .slot2D(12, 1.5, angle=90).extrude(4.51))

    # floor-plate screw pilots (M2x8) in the side walls
    for x, y in FLOOR_SCREWS:
        h = h.cut(cq.Workplane("XY", origin=(0, 0, Z_FLOOR - 0.01)).center(x, y).circle(1.25).extrude(10))   # M3 pilot
    return h


def floor_plate():
    """Closes the head bottom, clamps the board edge, USB-C plug opening, wire exit."""
    p = head_outer().intersect(box(-HW, HW, -HD, HD, -HH, Z_FLOOR))
    p = p.cut(rrect("XY", (0, 0, -HH), (0, Y_PCB - 1.65), 22.6, 7.1, 1.5, HH))                 # USB-C plug
    p = p.cut(rrect("XY", (0, 0, -HH), (0, (Y_PCB + 12 + Y_BACK - WALL - 1.5) / 2), 20,
                    Y_BACK - WALL - 1.5 - Y_PCB - 12, 2, HH))                                    # wire exit
    for x, y in FLOOR_SCREWS:
        p = p.cut(cq.Workplane("XY", origin=(0, 0, -HH)).center(x, y).circle(1.7).extrude(HH))
        p = p.cut(cq.Workplane().add(cq.Solid.makeCone(3.2, 1.7, 1.5, cq.Vector(x, y, -HH / 2 - 0.01), cq.Vector(0, 0, 1))))   # M3 csk
    # board edge: front stops at the outer corners (outside the header solder joints), rear posts
    p = p.union(boxes_x(PCB_W / 2 - 0.3, IN_W / 2 - 0.1, Y_PCB - 1.0, Y_PCB, Z_FLOOR - 0.01, Z_FLOOR + 1.2))
    p = p.union(boxes_x(6.0, 9.0, Y_PCB + PCB_T + 0.4, Y_PCB + PCB_T + 1.9, Z_FLOOR - 0.01, Z_FLOOR + 4.0))
    return p


def arm(x0, t=ARM_T):
    a = (cq.Workplane("YZ", origin=(x0, 0, 0)).moveTo(-ARM_BASE, Z_PLATE_TOP - 1).lineTo(ARM_BASE, Z_PLATE_TOP - 1)
         .lineTo(ARM_TOP_R, 0).threePointArc((0, ARM_TOP_R), (-ARM_TOP_R, 0)).close().extrude(t))
    return soft(a, lambda w: w.faces(">X or <X").edges().chamfer(0.8))


def yoke():
    y = box(X_ARM_L_OUT, X_ARM_R_OUT, -ARM_BASE, ARM_BASE, Z_PLATE_BOT, Z_PLATE_TOP).edges("|Z").fillet(8.0)
    y = soft(y, lambda w: w.faces("<Z").edges().chamfer(0.8))
    y = y.union(arm(X_ARM_R_IN, ARM_T_R)).union(arm(X_ARM_L_OUT))
    # zip-tie block on the back: harness + MG90S lead are tied here, then loop up into the head floor
    zm = (Z_PLATE_BOT + Z_PLATE_TOP) / 2
    y = y.union(box(-6, 6, ARM_BASE - 0.01, ARM_BASE + 4, Z_PLATE_BOT, Z_PLATE_TOP))
    y = y.cut(box(-7, 7, ARM_BASE + 1, ARM_BASE + 2.8, zm - 2.2, zm + 2.2))

    # right arm (motor side): MG90S window (+-1.5 mm vertical play) + slotted ear holes
    y = y.cut(box(X_ARM_R_IN - 1, X_ARM_R_OUT + 1, -SS_W / 2 - SS_FIT, SS_W / 2 + SS_FIT,
                  -(SS_L - SS_SHAFT_FROM_END) - SS_FIT, SS_SHAFT_FROM_END + SS_FIT))
    y = y.cut(cq.Workplane("YZ", origin=(X_ARM_R_IN - 1, 0, 0))
              .pushPoints([(0, SS_ZC - SS_EAR_PITCH / 2), (0, SS_ZC + SS_EAR_PITCH / 2)])
              .slot2D(2.6, 1.8, angle=90).extrude(ARM_T + 2))   # +-0.4 for hole spacing variation

    # left arm (bearing side): round hub outside, ring touching only the 608 inner race, axle hole + M3 lock from the top
    hub = cq.Workplane("YZ", origin=(X_ARM_L_OUT - HUB_T, 0, 0)).circle(13).extrude(HUB_T + 0.01)
    y = y.union(soft(hub, lambda w: w.faces("<X").edges().chamfer(1.0)))
    y = y.union(cq.Workplane("YZ", origin=(X_ARM_L_IN, 0, 0)).circle(5.6).extrude(RING_T))
    y = y.cut(cq.Workplane("YZ", origin=(X_ARM_L_OUT - HUB_T - 1, 0, 0)).circle(AXLE_D / 2 + 0.2).extrude(ARM_T + HUB_T + RING_T + 2))
    y = y.cut(cq.Workplane("XY", origin=(X_ARM_L_OUT + ARM_T / 2, 0, AXLE_FLAT - 0.5)).circle(1.25).extrude(ARM_TOP_R + 2))

    # MG996R round-horn pocket + 4 radial screw slots + centre-screw access
    y = y.cut(cq.Workplane("XY", origin=(0, 0, Z_PLATE_BOT - 0.01)).circle(SB_HORN_D / 2 + 0.2).extrude(SB_POCKET + 0.01))
    y = y.cut(cq.Workplane("XY", origin=(0, 0, Z_PLATE_BOT + SB_POCKET - 0.01)).circle(SB_HUB_D / 2 + 0.5).extrude(SB_HUB_TOP + 0.31))
    y = y.cut(cq.Workplane("XY", origin=(0, 0, Z_PLATE_BOT - 1)).circle(3.5).extrude(PLATE_T + 2))
    for a in (45, 135, 225, 315):
        y = y.cut(cq.Workplane("XY", origin=(0, 0, Z_PLATE_BOT - 1)).center(8.25, 0).slot2D(7.7, 2.2)
                  .extrude(PLATE_T + 2).rotate((0, 0, 0), (0, 0, 1), a))
    return y


AXLE_FLAT = AXLE_D / 2 - 0.8   # D-flat on top of the axle: lock screw presses here; printed flat-side down


def axle_pin():
    """Printed 608 axle: D-shaft through hub + arm + ring into the bearing, D-flange outside."""
    x0 = X_ARM_L_OUT - HUB_T
    clip = box(-60, 0, -10, 10, -10, AXLE_FLAT)
    shaft = cyl_x(AXLE_D / 2, x0, -HW / 2 + B608_W - 0.5)
    flange = cyl_x(6.5, x0 - 2.0, x0)
    return shaft.union(flange).intersect(clip)


def servo_cap():
    """Covers the MG90S on the outside of the right arm. Crush ribs grip the servo case."""
    c = box(0, CAP_D, -CAP_HW, CAP_HW, SS_ZC - CAP_HZ, SS_ZC + CAP_HZ).edges("|X").fillet(4.0)
    c = soft(c, lambda w: w.faces(">X").edges().fillet(2.5))
    c = c.cut(box(-1, SS_EAR_T + 0.4, -CAP_HW + CAP_WALL, CAP_HW - CAP_WALL,
                  SS_ZC - CAP_HZ + CAP_WALL, SS_ZC + CAP_HZ - CAP_WALL))
    c = c.cut(box(-1, CAP_D - CAP_WALL, -SS_W / 2 - 0.3, SS_W / 2 + 0.3, SS_ZC - SS_L / 2 - 0.3, SS_ZC + SS_L / 2 + 0.3))
    for s in (-1, 1):
        for dz in (-7, 7):
            y0, y1 = sorted((s * (SS_W / 2 + 0.3), s * (SS_W / 2 - 0.1)))
            c = c.union(box(SS_EAR_T + 0.4, CAP_D - CAP_WALL, y0, y1, SS_ZC + dz - 0.6, SS_ZC + dz + 0.6))
    c = c.cut(box(SS_EAR_T + 10, SS_EAR_T + 15.5, -2.5, 2.5, SS_ZC - CAP_HZ - 1, SS_ZC - SS_L / 2))  # MG90S cable
    return c.translate((X_ARM_R_OUT, 0, 0))


def base():
    r = BASE_R
    b = cq.Workplane("XY").circle(r).extrude(DECK_TOP)
    b = b.cut(cq.Workplane("XY").box(80, 20, DECK_TOP * 3).translate((0, OLED_YF - 10, 0)))  # screen facet
    b = b.cut(cq.Workplane("XY").box(80, 20, DECK_TOP * 3).translate((0, BACK_YF + 10, 0)))  # switch / jack facet
    b = soft(b, lambda w: w.faces(">Z").edges().chamfer(3.5))   # 45 deg: prints clean deck-down
    b = soft(b, lambda w: w.faces("<Z").edges().chamfer(1.0))
    inner = (cq.Workplane("XY").circle(r - BASE_WALL).extrude(DECK_IN + 0.01).translate((0, 0, -0.01))
             .intersect(box(-r, r, OLED_YF + OLED_T, BACK_YF - BACK_T, -1, DECK_TOP)))
    b = b.cut(inner)

    # MG996R: case top flush with deck, ears pull up against 2 lugs (M3 self-tap from below)
    cx = SB_L / 2 - SB_SHAFT_FROM_END
    ear_top = DECK_TOP - SB_CASE_H + SB_EAR_Z + SB_EAR_T
    for sx in (-1, 1):
        hx = cx + sx * SB_EAR_PITCH / 2
        for sy in (-1, 1):   # two pads per ear: centre gap clears the ear's triangular rib
            y0, y1 = sorted((sy * SB_RIB_GAP / 2, sy * 9.0))
            b = b.union(box(hx - 3.5, hx + 3.5, y0, y1, ear_top, DECK_TOP - DECK_T + 0.01))
        b = b.cut(cq.Workplane("XY", origin=(0, 0, ear_top - 1)).pushPoints([(hx, -SB_EAR_ACROSS / 2), (hx, SB_EAR_ACROSS / 2)])
                  .slot2D(6.0, 2.5).extrude(8))   # M3 self-tap, +-1.75 for hole-spacing variation
    b = b.cut(box(cx - SB_L / 2 - SB_FIT, cx + SB_L / 2 + SB_FIT, -SB_W / 2 - SB_FIT, SB_W / 2 + SB_FIT, 0, DECK_TOP + 1))

    # screen: bezel recess + window; board slides UP between side rails, then forward onto 4 pegs,
    # glass flat on the wall (dot of hot glue on each peg tip).
    b = b.cut(rrect("XZ", (0, OLED_YF + 0.8, 0), (0, OLED_GZ), OLED_WIN_W + 6, OLED_WIN_H + 7, 3, 2))
    b = b.cut(brand(cq.Plane(origin=(0, OLED_YF - 0.01, OLED_GZ - (OLED_WIN_H + 7) / 2 - 5), xDir=(1, 0, 0), normal=(0, -1, 0)), 5.5))
    b = b.cut(rrect("XZ", (0, OLED_YF + 1, 0), (0, OLED_GZ), OLED_WIN_W, OLED_WIN_H, 1, -(OLED_T + 2)))
    yi = OLED_YF + OLED_T
    peg = OLED_GLASS_T + OLED_PCB_T + 0.05
    z0 = OLED_OZ - OLED_H / 2 - 1
    b = b.union(boxes_x(OLED_W / 2 + 0.3, OLED_W / 2 + 2.0, yi - 0.01, yi + 2 * peg + 1.5, z0, DECK_IN + 0.01))   # side rails
    pts = [(sx * OLED_HX / 2, OLED_OZ + sz * OLED_HZ / 2) for sx in (-1, 1) for sz in (-1, 1)]
    b = b.union(cq.Workplane("XZ", origin=(0, yi - 0.01, 0)).pushPoints(pts).circle(OLED_PEG_D / 2).extrude(-(peg + 0.01)))

    # bottom-plate screw columns (grow from the deck: no overhangs printed deck-down)
    for a in COL_A:
        col = (cq.Workplane("XY", origin=(0, 0, BOTTOM_T)).center(r - 6.5, 0).circle(4).extrude(DECK_IN - BOTTOM_T + 0.01)
               .cut(cq.Workplane("XY", origin=(0, 0, BOTTOM_T - 1)).center(r - 6.5, 0).circle(1.25).extrude(11)))
        b = b.union(col.rotate((0, 0, 0), (0, 0, 1), a))

    # 5 V buck: flat pad on the left wall + 2 side lips; board taped to the pad, wire pads free at both ends
    b = b.union(box(-(r - BASE_WALL) - 0.5, DCDC_X, -DCDC_W / 2 - 1.8, DCDC_W / 2 + 1.8, DCDC_Z0 - 1, DECK_IN + 0.01))
    for s in (-1, 1):
        y0, y1 = sorted((s * (DCDC_W / 2 + 0.3), s * (DCDC_W / 2 + 1.8)))
        b = b.union(box(DCDC_X - 0.01, DCDC_X + DCDC_T + 1.0, y0, y1, DCDC_Z0 - 1, DECK_IN + 0.01))
    # WAGO pocket (2 side by side, wire entries facing down), push up in, dab of hot glue
    wx, wy = WAGO_XY
    hx, hy = WAGO_W / 2 + 0.2, WAGO_T + 0.2
    b = b.union(box(wx - hx - 1.6, wx + hx + 1.6, wy - hy - 1.6, wy + hy + 1.6, DECK_IN - WAGO_L + 3, DECK_IN + 0.01)
                .cut(box(wx - hx, wx + hx, wy - hy, wy + hy, DECK_IN - WAGO_L, DECK_IN)))
    # capacitor clip, leads down
    cx2, cy2 = ELCAP_XY
    b = b.union(cq.Workplane("XY", origin=(cx2, cy2, DECK_IN - 16)).circle(ELCAP_D / 2 + 1.7).extrude(16.01)
                .cut(cq.Workplane("XY", origin=(cx2, cy2, DECK_IN - 17)).circle(ELCAP_D / 2 + 0.3).extrude(17)))

    # back panel: rocker switch + charge jack
    b = b.cut(box(SW_X - SW_CUT_W / 2, SW_X + SW_CUT_W / 2, BACK_YF - BACK_T - 1, BACK_YF + 1,
                  PANEL_Z - SW_CUT_H / 2, PANEL_Z + SW_CUT_H / 2))
    b = b.cut(cq.Workplane("XZ", origin=(0, BACK_YF + 1, 0)).center(JACK_X, PANEL_Z).circle(JACK_D / 2 + 0.2).extrude(BACK_T + 2))
    # harness exit through the deck (base -> yoke tie block -> head)
    b = b.cut(cq.Workplane("XY", origin=(0, 0, DECK_IN - 1)).center(0, HARNESS_Y).slot2D(12, 5).extrude(DECK_T + 2))
    return b


def bottom_plate():
    p = cq.Workplane("XY").circle(BASE_R - BASE_WALL - 0.3).extrude(BOTTOM_T)
    p = p.cut(box(-BASE_R, BASE_R, -BASE_R, OLED_YF + OLED_T + 0.3, -1, BOTTOM_T + 1))
    p = p.cut(box(-BASE_R, BASE_R, BACK_YF - BACK_T - 0.3, BASE_R, -1, BOTTOM_T + 1))
    for a in COL_A:
        p = p.cut(cq.Workplane("XY", origin=(0, 0, -1)).center(BASE_R - 6.5, 0).circle(1.7).extrude(BOTTOM_T + 2)
                  .rotate((0, 0, 0), (0, 0, 1), a))
        p = p.cut(cq.Workplane().add(cq.Solid.makeCone(3.4, 1.7, 1.7, cq.Vector(BASE_R - 6.5, 0, -0.01), cq.Vector(0, 0, 1)))
                  .rotate((0, 0, 0), (0, 0, 1), a))
    for a in (45, 135, 225, 315):
        p = p.cut(cq.Workplane("XY", origin=(0, 0, -0.01)).center(34, 0).circle(5.2).extrude(1.0)
                  .rotate((0, 0, 0), (0, 0, 1), a))  # rubber feet recesses
    # battery fence: 4 short ribs, corners left open for the pack leads
    for s in (-1, 1):
        x0, x1 = sorted((s * (BAT_L / 2 + 0.5), s * (BAT_L / 2 + 2.0)))
        y0, y1 = sorted((s * (BAT_W / 2 + 0.5), s * (BAT_W / 2 + 2.0)))
        p = p.union(box(x0, x1, -12, 12, BOTTOM_T - 0.01, BOTTOM_T + 3))
        p = p.union(box(-15, 15, y0, y1, BOTTOM_T - 0.01, BOTTOM_T + 3))
    return p


def parts():
    return {"base": base(), "bottom_plate": bottom_plate(), "yoke": yoke(), "servo_cap": servo_cap(), "axle_pin": axle_pin(),
            "head": head(), "head_floor": floor_plate()}


# =============================== bought parts ===============================
def cyl_x(r, x0, x1):
    return cq.Workplane("YZ", origin=(x0, 0, 0)).circle(r).extrude(x1 - x0)


def esp32_board():
    """Freenove ESP32-S3 WROOM CAM in its installed position (front = -Y), incl. header + Dupont space."""
    yb, zb = Y_PCB, Z_FLOOR
    b = box(-PCB_W / 2, PCB_W / 2, yb, yb + PCB_T, zb, PCB_TOP)
    b = b.union(box(-MOD_W / 2, MOD_W / 2, yb - 3.1, yb, PCB_TOP + ANT_L - MOD_L, PCB_TOP - 0.8))   # module shield
    b = b.union(box(-MOD_W / 2, MOD_W / 2, yb - 0.8, yb, PCB_TOP - 0.8, PCB_TOP + ANT_L))           # antenna tab
    c = CAM_BOX / 2
    b = b.union(box(-c, c, yb - LENS_H + 0.8, yb - 3.1, CAM_Z - c, CAM_Z + c))                    # camera block on module
    b = b.union(cq.Workplane("XZ", origin=(0, yb - LENS_H + 0.8, 0)).moveTo(0, CAM_Z).circle(3.0).extrude(0.8))  # lens
    b = b.union(box(-11, 11, yb - 2.5, yb, PCB_TOP + ANT_L - 31, PCB_TOP + ANT_L - 25))           # FPC connector
    b = b.union(boxes_x(BTN_X - 2.5, BTN_X + 2.5, yb - 3.5, yb, zb + BTN_Z - 2.5, zb + BTN_Z + 2.5))
    b = b.union(boxes_x(USB_X - USB_W / 2, USB_X + USB_W / 2, yb - 3.2, yb, zb - 0.3, zb + 7.4))
    zh0, zh1 = max(zb + 0.05, zb + HDR_Z0 - 1.27), min(PCB_TOP - 0.05, zb + HDR_Z0 + 19 * 2.54 + 1.27)
    b = b.union(boxes_x(HDR_X - 1.0, HDR_X + 1.0, yb - 1.5, yb, zh0, zh1))                          # solder joints
    b = b.union(boxes_x(HDR_X - 1.27, HDR_X + 1.27, yb + PCB_T, yb + PCB_T + 2.5, zh0, zh1))      # header plastic
    b = b.union(boxes_x(HDR_X - 1.3, HDR_X + 1.3, yb + PCB_T + 2.5, yb + PCB_T + 16.5, zh0, zh1))  # Dupont plugs
    return b


_SG90 = None


def mg90s_parts():
    """MG90S = FreeCAD SG90 vendor STEP (vendor_models/); horn built from your measurements."""
    global _SG90
    if _SG90 is None:
        m = cq.importers.importStep(os.path.join(OUT, "vendor_models", "SG90_MG90S_servo_FreeCAD.step"))
        _SG90 = m.rotate((0, 0, 0), (0, 0, 1), 90).translate((16.6, 0, 0))   # shaft to origin, case along +x
    et = SS_EAR_BOT + SS_EAR_T
    ss = _SG90
    face = et + SS_EARTOP_TO_HORN_FACE              # outer face of horn arm
    a0 = face - SS_HORN_T
    zt2 = SS_HORN_L / 2 - SS_HORN_W1 / 2
    arm = (cq.Workplane("XY", origin=(0, 0, a0)).polyline([(0, SS_HORN_W0 / 2), (zt2, SS_HORN_W1 / 2), (zt2, -SS_HORN_W1 / 2),
            (0, -SS_HORN_W0 / 2), (-zt2, -SS_HORN_W1 / 2), (-zt2, SS_HORN_W1 / 2)]).close().extrude(SS_HORN_T)
           .union(cq.Workplane("XY", origin=(0, 0, a0)).pushPoints([(zt2, 0), (-zt2, 0)]).circle(SS_HORN_W1 / 2).extrude(SS_HORN_T)))
    horn = (arm.union(cq.Workplane("XY", origin=(0, 0, a0 - SS_HUB_IN)).circle(SS_HUB_D / 2).extrude(SS_HUB_IN + SS_HORN_T + SS_HUB_OUT))
            .cut(cq.Workplane("XY", origin=(0, 0, a0 - SS_HUB_IN - 0.1)).circle(2.3).extrude(SS_HUB_IN + 0.5)))   # spline hole

    def place(w):  # (x,y,z) -> (-z,-y,-x)
        return w.rotate((0, 0, 0), (1, 0, -1), 180).translate((X_ARM_R_OUT + et, 0, 0))
    return place(ss), place(horn)


def vitamins():
    """Envelope models of bought parts {name: (shape, group)}."""
    v = {}
    zb = DECK_TOP - SB_CASE_H
    x0, x1 = -SB_SHAFT_FROM_END, SB_L - SB_SHAFT_FROM_END
    cx = (x0 + x1) / 2
    et = SB_EAR_Z + SB_EAR_T
    mg = (box(x0, x1, -SB_W / 2, SB_W / 2, 0, SB_CASE_H)
          .union(box(cx - SB_EAR_SPAN / 2, cx + SB_EAR_SPAN / 2, -SB_W / 2 + 1, SB_W / 2 - 1, SB_EAR_Z, et))
          .union(box(x1, x1 + 3, -0.5, 0.5, et, et + 2.5)).union(box(x0 - 3, x0, -0.5, 0.5, et, et + 2.5))   # ear ribs
          .union(cq.Workplane("XY").circle(3).extrude(SB_HORN_FACE - SB_HORN_T - 3.0).translate((0, 0, SB_CASE_H))))
    v["MG996R"] = (mg.translate((0, 0, zb)), "base")
    hz = Z_PLATE_BOT + SB_POCKET                    # disc top face in yoke frame
    v["MG996R_horn"] = (cq.Workplane("XY", origin=(0, 0, hz - SB_HORN_T)).circle(SB_HORN_D / 2).extrude(SB_HORN_T)
                        .union(cq.Workplane("XY", origin=(0, 0, hz)).circle(SB_HUB_D / 2).extrude(SB_HUB_TOP))
                        .union(cq.Workplane("XY", origin=(0, 0, hz - SB_HORN_T - 3.0)).circle(SB_HUB_D / 2).extrude(3.0)), "yoke")
    v["horn_screws"] = (cq.Workplane("XY", origin=(0, 0, Z_PLATE_TOP)).pushPoints(
        [(8.25 * c, 8.25 * s) for c, s in ((0.7071, 0.7071), (-0.7071, 0.7071), (-0.7071, -0.7071), (0.7071, -0.7071))])
        .circle(2.1).extrude(1.6), "yoke")
    ss, horn = mg90s_parts()
    v["MG90S"] = (ss, "yoke")
    v["MG90S_horn"] = (horn, "head")
    v["608ZZ"] = (cyl_x(B608_OD / 2, -HW / 2, -HW / 2 + B608_W).cut(cyl_x(B608_ID / 2, -HW / 2 - 1, -HW / 2 + B608_W + 1)), "head")
    v["M3_lock_screw"] = (cq.Workplane("XY", origin=(X_ARM_L_OUT + ARM_T / 2, 0, AXLE_FLAT)).circle(1.5).extrude(16)
                          .union(cq.Workplane("XY", origin=(X_ARM_L_OUT + ARM_T / 2, 0, AXLE_FLAT + 16)).circle(2.75).extrude(3)), "yoke")
    v["laser_12mm"] = (cq.Workplane("XZ", origin=(0, Y_FRONT + 0.5, 0)).moveTo(0, LASER_Z).circle(6).extrude(-35), "head")
    v["ESP32S3_CAM"] = (esp32_board(), "head")
    ys = OLED_YF + OLED_T + OLED_GLASS_T + 0.05
    gz = OLED_GZ
    oled = (box(-OLED_GW / 2, OLED_GW / 2, ys - OLED_GLASS_T, ys, gz - OLED_GH / 2, gz + OLED_GH / 2)
            .union(box(-OLED_W / 2, OLED_W / 2, ys, ys + OLED_PCB_T, OLED_OZ - OLED_H / 2, OLED_OZ + OLED_H / 2))
            .cut(cq.Workplane("XZ", origin=(0, ys + OLED_PCB_T + 1, 0)).pushPoints(
                [(sx * OLED_HX / 2, OLED_OZ + sz * OLED_HZ / 2) for sx in (-1, 1) for sz in (-1, 1)])
                .circle(OLED_HOLE_D / 2).extrude(OLED_PCB_T + 2)))
    zt = OLED_OZ + OLED_H / 2
    v["OLED_096"] = (oled, "base")
    v["OLED_header_dupont"] = (box(-5.05, 5.05, ys + OLED_PCB_T, ys + OLED_PCB_T + 2.5 + 14 + 5, zt - 2.6, zt - 0.1), "base")

    v["battery_3S_BMS"] = (box(-BAT_L / 2, BAT_L / 2, -BAT_W / 2, BAT_W / 2, BOTTOM_T, BOTTOM_T + BAT_H), "base")
    v["buck_5V"] = (box(DCDC_X + 0.05, DCDC_X + DCDC_T, -DCDC_W / 2, DCDC_W / 2, DCDC_Z0, DCDC_Z0 + DCDC_L)
                    .union(box(DCDC_X + DCDC_T, DCDC_X + DCDC_PARTS, -DCDC_W / 2 + 1.5, DCDC_W / 2 - 1.5, DCDC_Z0 + 2, DCDC_Z0 + DCDC_L - 2)), "base")
    wx, wy = WAGO_XY
    v["WAGO_x2"] = (box(wx - WAGO_W / 2, wx + WAGO_W / 2, wy - WAGO_T, wy + WAGO_T, DECK_IN - 0.1 - WAGO_L, DECK_IN - 0.1), "base")
    v["cap_1000uF"] = (cq.Workplane("XY", origin=(*ELCAP_XY, DECK_IN - 0.1 - ELCAP_L)).circle(ELCAP_D / 2).extrude(ELCAP_L), "base")
    sw, sh = SW_CUT_W / 2 - 0.2, SW_CUT_H / 2 - 0.2
    v["rocker_switch"] = (box(SW_X - sw, SW_X + sw, BACK_YF - SW_DEPTH, BACK_YF, PANEL_Z - sh, PANEL_Z + sh)
                          .union(box(SW_X - sw - 1.2, SW_X + sw + 1.2, BACK_YF, BACK_YF + 2.5, PANEL_Z - sh - 1.2, PANEL_Z + sh + 1.2)), "base")
    yi = BACK_YF - BACK_T
    v["dc_jack"] = (cq.Workplane("XZ", origin=(0, yi, 0)).center(JACK_X, PANEL_Z).circle(5.5).extrude(JACK_DEPTH)
                    .union(cq.Workplane("XZ", origin=(0, BACK_YF + 4, 0)).center(JACK_X, PANEL_Z).circle(JACK_D / 2 - 0.1).extrude(BACK_T + 4)), "base")
    v["dc_jack_nut"] = (cq.Workplane("XZ", origin=(0, BACK_YF + 2, 0)).center(JACK_X, PANEL_Z).polygon(6, 12.7)
                        .extrude(2).cut(cq.Workplane("XZ", origin=(0, BACK_YF + 3, 0)).center(JACK_X, PANEL_Z).circle(JACK_D / 2).extrude(4)), "base")
    return v


GROUPS = {"base": "base", "bottom_plate": "base", "yoke": "yoke", "servo_cap": "yoke", "axle_pin": "yoke",
          "head": "head", "head_floor": "head"}
# intended overlaps: grub screw threads into the arm; cap crush ribs grip the servo
ALLOWED = {frozenset(("yoke", "M3_lock_screw")), frozenset(("servo_cap", "MG90S")), frozenset(("MG90S", "MG90S_horn"))}


def posed(shape, group, pan, tilt):
    if group == "base":
        return shape
    if group == "head":
        shape = shape.rotate((0, 0, 0), (1, 0, 0), tilt)
    return shape.translate((0, 0, YOKE_Z)).rotate((0, 0, 0), (0, 0, 1), pan)


def vol(a, b):
    A, B = a.val().BoundingBox(), b.val().BoundingBox()
    if (A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin
            or A.zmax < B.zmin or B.zmax < A.zmin):
        return 0.0
    return a.intersect(b).val().Volume()


def check(p):
    """1. No part overlaps another at pan 0/+-90, tilt 0/+-45."""
    allp = {k: (s, GROUPS[k]) for k, s in p.items()} | vitamins()
    for k, (s, _) in allp.items():
        assert s.val().isValid() and s.val().Volume() > 1, f"{k} is not a valid solid"
    names = list(allp)
    for pan in (0, 90, -90):
        for tilt in (0, 45, -45):
            sh = {k: posed(s, g, pan, tilt) for k, (s, g) in allp.items()}
            for i, a in enumerate(names):
                for b in names[i + 1:]:
                    if frozenset((a, b)) in ALLOWED or (allp[a][1] == allp[b][1] and (pan, tilt) != (0, 0)):
                        continue
                    v = vol(sh[a], sh[b])
                    assert v < 0.01, f"{a} hits {b} at pan {pan} tilt {tilt}: {v:.2f} mm^3"
    print("check 1 OK: no collisions (printed + bought parts, headers/Dupont, pan 0/+-90, tilt 0/+-45)")

    """2. Assembly paths: each part moves into place without hitting what is already installed."""
    h, y = p["head"], p["yoke"]
    board = esp32_board()
    for dz in range(-64, 1, 2):            # board slides up from below (floor plate not fitted yet)
        assert vol(board.translate((0, 0, dz)), h) < 0.01, f"board jams sliding in at {dz} mm"
    laser = vitamins()["laser_12mm"][0]
    for dy in range(-40, 1, 4):            # laser slides in from the front
        assert vol(laser.translate((0, dy, 0)), h) < 0.01, f"laser jams at {dy} mm"
    ss, horn = mg90s_parts()
    for dx in range(30, -1, -2):           # MG90S pushed in from outside onto the horn (head already in)
        s = ss.translate((dx, 0, 0))
        assert vol(s, y) < 0.01 and vol(s, h) < 0.01, f"MG90S jams at {dx} mm"   # spline entering the horn is intended
    brg = vitamins()["608ZZ"][0]
    for dz in range(80, -1, -4):           # head (with horn + 608) lowered between the arms
        for s in (h, horn, brg):
            assert vol(s.translate((0, 0, dz)), y) < 0.01, f"head jams lowering into yoke at {dz} mm"
    v = vitamins()
    b, mg = p["base"], v["MG996R"][0]
    for dz in range(-50, 1, 2):            # MG996R pushed up through the deck from below
        assert vol(mg.translate((0, 0, dz)), b) < 0.01, f"MG996R jams at {dz} mm"
    up = [(0, 0, -dz) for dz in range(50, -1, -2)]
    oled = [(0, 3.5, -dz) for dz in range(50, -1, -2)] + [(0, dy / 2, 0) for dy in range(7, -1, -1)]   # up behind the pegs, then forward
    moves = {"OLED_096": oled, "OLED_header_dupont": oled, "buck_5V": up, "WAGO_x2": up, "cap_1000uF": up,
             "battery_3S_BMS": up,                                    # last, then the bottom plate
             "rocker_switch": [(0, dy, 0) for dy in range(30, -1, -2)]}   # snapped in from outside
    for k, path in moves.items():          # everything in the base goes in from below with the bottom plate off
        for d in path:
            s = v[k][0].translate(d)
            assert vol(s, b) < 0.01 and vol(s, mg) < 0.01, f"{k} jams at {d}"
    for dy in range(-16, 1, 2):            # charge jack from inside out (fitted before the MG996R), nut from outside
        assert vol(v["dc_jack"][0].translate((0, dy, 0)), b) < 0.01, f"dc_jack jams at {dy}"
    ax = p["axle_pin"]
    for dx in range(-26, 1, 2):            # axle pin pushed in from the left through the hub into the 608
        s = ax.translate((dx, 0, 0))
        assert vol(s, y) < 0.01 and vol(s, h) < 0.01 and vol(s, v["608ZZ"][0]) < 0.01, f"axle pin jams at {dx} mm"
    for dz in range(40, -1, -2):           # yoke lowered onto the MG996R horn
        s = posed(y, "yoke", 0, 0).translate((0, 0, dz))
        for o in (b, mg, posed(v["MG996R_horn"][0], "yoke", 0, 0)):
            assert vol(s, o) < 0.01, f"yoke jams at {dz} mm"
    print("check 2 OK: MG996R, OLED, buck, WAGOs, cap, battery, switch, jack, yoke, board, laser, head and MG90S "
          "all go in along their assembly paths")

    """3. Every screw can be reached with a screwdriver at the step where it is fitted."""
    def tool(pt, d, r_tip=1.6, tip=45.0, r_handle=11.0, handle=60.0):
        pt, d = cq.Vector(*pt), cq.Vector(*d)
        t = cq.Solid.makeCylinder(r_tip, tip, pt, d).fuse(cq.Solid.makeCylinder(r_handle, handle, pt + d * tip, d))
        return cq.Workplane().add(t)

    def reach(name, tools, obstacles):
        for t in tools:
            for o in obstacles:
                assert vol(t, o) < 0.01, f"no screwdriver access: {name}"

    cx = SB_L / 2 - SB_SHAFT_FROM_END
    ear_bot = DECK_TOP - SB_CASE_H + SB_EAR_Z
    reach("MG996R ear screws (from below)",
          [tool((cx + sx * SB_EAR_PITCH / 2, sy * SB_EAR_ACROSS / 2, ear_bot - 2.0), (0, 0, -1)) for sx in (-1, 1) for sy in (-1, 1)],
          [b, mg])
    yk = posed(y, "yoke", 0, 0)
    pts = [(8.25 * c, 8.25 * s, Z_PLATE_TOP + 1.6) for c, s in ((0.7071, 0.7071), (-0.7071, 0.7071), (-0.7071, -0.7071), (0.7071, -0.7071))]
    pts.append((0, 0, Z_PLATE_BOT + SB_POCKET))
    reach("yoke-to-horn screws + horn centre screw (from above)",
          [posed(tool(q, (0, 0, 1)), "yoke", 0, 0) for q in pts], [yk, b, mg])
    reach("head floor screws (from below)", [tool((x, yy, -HH / 2), (0, 0, -1)) for x, yy in FLOOR_SCREWS], [h, p["head_floor"]])
    reach("M3 axle lock screw (from the top of the left arm)",
          [tool((X_ARM_L_OUT + ARM_T / 2, 0, AXLE_FLAT + 19), (0, 0, 1))], [y, h, p["axle_pin"]])
    reach("MG90S ear screws (from the right)",
          [tool((X_ARM_R_OUT + SS_EAR_T, 0, SS_ZC + dz), (1, 0, 0)) for dz in (-SS_EAR_PITCH / 2, SS_EAR_PITCH / 2)],
          [y, h, ss, horn])
    reach("laser grub screw (from the top)", [tool((0, Y_FRONT + RIDGE_L / 2, LASER_Z + RIDGE_R), (0, 0, 1), r_tip=0.8, r_handle=0.8)], [h])
    print("check 3 OK: screwdriver / hex-key access to every screw at its assembly step")


def fit_tests(p):
    """Small slices of the real parts at every bought-part interface. Print these first."""
    h, y, b, f = p["head"], p["yoke"], p["base"], p["head_floor"]
    return {
        "fit_tilt_servo_arm": y.intersect(box(X_ARM_R_IN - 0.5, X_ARM_R_OUT + 0.5, -25, 25, SS_ZC - 19, 20)),
        "fit_horn_pocket": h.intersect(box(HW / 2 - 4, HW / 2 + 1, -10, 10, -21, 21)),
        "fit_608_pocket": h.intersect(box(-HW / 2 - 1, -HW / 2 + SIDE, -14, 14, -14, 14)),
        "fit_pan_horn": y.intersect(cq.Workplane("XY", origin=(0, 0, Z_PLATE_BOT - 1)).circle(18).extrude(PLATE_T + 2)),
        "fit_laser_bore": h.intersect(box(-12, 12, Y_FRONT - 1, Y_FRONT + 10, HH / 2 - WALL, HH)),
        "fit_board_top_antenna_camera": h.intersect(box(-IN_W / 2 - 2, IN_W / 2 + 2, Y_FRONT - 1, Y_PCB + 3, CAM_Z - 8, HH / 2)),
        "fit_head_floor_usb": f,
        "fit_screen": b.intersect(box(-19, 19, -BASE_R - 1, OLED_YF + 9, OLED_OZ - OLED_H / 2 - 3, DECK_TOP)),
        "fit_back_panel": b.intersect(box(-24, 24, BACK_YF - BACK_T - 1, BACK_YF + 1, PANEL_Z - 14, PANEL_Z + 14)),
        "fit_pan_servo_deck": b.intersect(box(-22, 42, -14, 14, DECK_TOP - 10, DECK_TOP + 1)),
        "fit_axle_arm": y.intersect(box(X_ARM_L_OUT - HUB_T - 1, X_ARM_L_IN + RING_T + 1, -20, 20, -12, ARM_TOP_R + 1)),
        "axle_pin": p["axle_pin"],
    }


# print orientation (Bambu Lab, no supports) and plates
ROT = {
    "flat": lambda s: s,
    "face_down": lambda s: s.rotate((0, 0, 0), (1, 0, 0), 90),      # head: camera face on the bed
    "flip": lambda s: s.rotate((0, 0, 0), (1, 0, 0), 180),          # base: deck on the bed
    "x_down": lambda s: s.rotate((0, 0, 0), (0, 1, 0), 90),         # +X face on the bed
    "x_up": lambda s: s.rotate((0, 0, 0), (0, 1, 0), -90),          # +X face up
    "back_down": lambda s: s.rotate((0, 0, 0), (1, 0, 0), -90),     # +Y face on the bed
}
ORIENT = {"head": "face_down", "head_floor": "flat", "yoke": "flat", "servo_cap": "x_down", "base": "flip",
          "bottom_plate": "flat", "fit_tilt_servo_arm": "x_down", "fit_horn_pocket": "x_up", "fit_608_pocket": "x_down",
          "fit_pan_horn": "flip", "fit_laser_bore": "face_down", "fit_board_top_antenna_camera": "face_down",
          "fit_head_floor_usb": "flat", "fit_screen": "face_down", "fit_pan_servo_deck": "flip",
          "fit_axle_arm": "flat", "axle_pin": "flip", "fit_back_panel": "back_down"}
PLATES = {
    "plate1_fit_tests_any_colour": ["fit_board_top_antenna_camera", "fit_horn_pocket", "fit_608_pocket",
                                    "fit_tilt_servo_arm", "fit_pan_horn", "fit_pan_servo_deck", "fit_laser_bore",
                                    "fit_screen", "fit_back_panel", "fit_axle_arm", "axle_pin"],
    "plate2_light_head_yoke": ["head", "yoke"],
    "plate3_dark_base_servo_cap": ["base", "servo_cap"],
    "plate4_dark_bottom_plate_head_floor": ["bottom_plate", "head_floor", "axle_pin"],
}


def export_print(shapes):
    """Oriented STLs + one 3MF per plate. Objects packed in 170 x 170 mm (fits A1 mini and up)."""
    import trimesh
    d = os.path.join(OUT, "PRINT_BambuLab")
    os.makedirs(os.path.join(d, "stl_oriented"), exist_ok=True)
    tmp = tempfile.mkdtemp()
    for plate, names in PLATES.items():
        meshes = []
        for n in names:
            s = ROT[ORIENT[n]](shapes[n])
            bb = s.val().BoundingBox()
            s = s.translate((-bb.xmin, -bb.ymin, -bb.zmin))
            f = os.path.join(d, "stl_oriented", f"{n}.stl")
            cq.exporters.export(s, f, tolerance=0.02, angularTolerance=0.1)
            meshes.append((n, trimesh.load(f), bb.xlen, bb.ylen))
        x = y = row = 0.0
        scene = trimesh.Scene()
        for n, m, w, dep in sorted(meshes, key=lambda t: -t[3]):   # shelf packing
            if x + w > 170:
                x, y, row = 0.0, y + row + 6, 0.0
            assert y + dep <= 170, f"{plate} does not fit 170 x 170"
            m.apply_translation((5 + x, 5 + y, 0))
            scene.add_geometry(m, node_name=n, geom_name=n)
            x, row = x + w + 6, max(row, dep)
        os.makedirs(os.path.join(d, "other_printers_geometry_only"), exist_ok=True)
        scene.export(os.path.join(d, "other_printers_geometry_only", f"{plate}.3mf"))
    return d


EXPLODE = {  # offsets (mm) along each part's assembly path, in its own frame
    "bottom_plate": (0, 0, -110), "BUY_battery_3S_BMS": (0, 0, -75), "BUY_buck_5V": (0, 0, -45), "BUY_WAGO_x2": (0, 0, -45),
    "BUY_cap_1000uF": (0, 0, -45), "BUY_rocker_switch": (0, 30, 0), "BUY_dc_jack_nut": (0, 25, 0), "BUY_MG996R": (0, 0, -40), "BUY_OLED_096": (0, 15, -55), "BUY_OLED_header_dupont": (0, 15, -55),
    "BUY_MG996R_horn": (0, 0, 25), "BUY_horn_screws": (0, 0, 55), "yoke": (0, 0, 40), "BUY_MG90S": (55, 0, 40),
    "servo_cap": (95, 0, 40), "axle_pin": (-45, 0, 40), "BUY_M3_lock_screw": (0, 0, 70), "head": (0, 0, 150), "BUY_608ZZ": (-25, 0, 150),
    "BUY_MG90S_horn": (20, 0, 150), "BUY_laser_12mm": (0, -60, 150), "BUY_ESP32S3_CAM": (0, 0, 95),
    "head_floor": (0, 0, 70),
}


def sw(shape):
    """Assembly only: turn the camera toward SolidWorks' Front view (SW's STEP import maps Z-up to Y-up)."""
    return shape.rotate((0, 0, 0), (0, 0, 1), 90)


if __name__ == "__main__":
    p = parts()
    check(p)
    for d in ("STL", "STEP", os.path.join("STL", "fit_test")):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    for k, s in p.items():
        cq.exporters.export(s, os.path.join(OUT, "STL", f"{k}.stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(s, os.path.join(OUT, "STEP", f"{k}.step"))
    for k, s in fit_tests(p).items():
        cq.exporters.export(s, os.path.join(OUT, "STL", "fit_test", f"{k}.stl"), tolerance=0.02, angularTolerance=0.1)
    asm = cq.Assembly(name="gimbal")
    cols = {"base": (0.13, 0.15, 0.18), "bottom_plate": (0.13, 0.15, 0.18), "yoke": (0.85, 0.86, 0.88),
            "servo_cap": (0.13, 0.15, 0.18), "head": (0.85, 0.86, 0.88), "head_floor": (0.13, 0.15, 0.18),
            "axle_pin": (0.13, 0.15, 0.18)}
    for k, s in p.items():
        asm.add(sw(posed(s, GROUPS[k], 0, 0)), name=k, color=cq.Color(*cols[k]))
    vcol = {"ESP32S3_CAM": (0.1, 0.45, 0.2), "OLED_096": (0.05, 0.05, 0.3), "laser_12mm": (0.8, 0.1, 0.1),
            "608ZZ": (0.7, 0.7, 0.75), "M3_lock_screw": (0.7, 0.7, 0.75), "MG996R_horn": (0.95, 0.95, 0.95),
            "MG90S_horn": (0.95, 0.95, 0.95), "OLED_header_dupont": (0.9, 0.9, 0.2), "horn_screws": (0.7, 0.7, 0.75),
            "MG90S": (0.1, 0.2, 0.6), "MG996R": (0.1, 0.1, 0.1), "battery_3S_BMS": (0.1, 0.35, 0.75),
            "buck_5V": (0.1, 0.3, 0.6), "WAGO_x2": (0.85, 0.5, 0.1), "cap_1000uF": (0.15, 0.15, 0.5)}
    for k, (s, g) in vitamins().items():
        asm.add(sw(posed(s, g, 0, 0)), name="BUY_" + k, color=cq.Color(*vcol.get(k, (0.1, 0.1, 0.1))))
    asm.export(os.path.join(OUT, "STEP", "gimbal_assembly.step"))
    ex = cq.Assembly(name="gimbal_exploded")
    for k, s in p.items():
        ex.add(sw(posed(s, GROUPS[k], 0, 0).translate(EXPLODE.get(k, (0, 0, 0)))), name=k, color=cq.Color(*cols[k]))
    for k, (s, g) in vitamins().items():
        ex.add(sw(posed(s, g, 0, 0).translate(EXPLODE.get("BUY_" + k, (0, 0, 0)))), name="BUY_" + k,
               color=cq.Color(*vcol.get(k, (0.1, 0.1, 0.1))))
    ex.export(os.path.join(OUT, "STEP", "gimbal_exploded.step"))
    print("print files:", export_print(p | fit_tests(p)))
    print(f"head {HW:.1f} x {HD:.1f} x {HH:.1f}, overall width {X_ARM_R_OUT + CAP_D - X_ARM_L_OUT + HUB_T:.1f}, "
          f"tilt axis {YOKE_Z:.1f} above table, total height {YOKE_Z + LASER_Z + RIDGE_R:.1f}")
