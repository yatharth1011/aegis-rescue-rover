# Author: Yatharth Shrivastava () and team
# Project: A.E.G.I.S. rescue rover, Mk2 assembly
# Course: TL101 Tinkering Lab | IIT Ropar
#
# Builds the complete rover natively in Autodesk Fusion:
#   - covered intake bay: the target rolls in and a servo-driven side door
#     closes behind it (the rover "takes the object inside")
#   - 4WD on N20 gearmotors in printed hangers, chassis raised to clear the bay
#   - two laser-cut acrylic decks on M3 brass pillars
#   - electronics, sensors and camera mast placed where the firmware expects
#   - one component per part, repeated parts as instances, named parameters,
#     wheel and door joints with limits, materials and colours
#
# Run it from Utilities > Scripts and Add-Ins, or through the AEGIS bridge
# add-in, which calls build(out_dir) and also exports renders, STEP, the
# laser-cut DXFs and the 3D-print STLs.
#
# Coordinates in mm (the API works in cm; helpers convert).
# +X forward, +Y left, +Z up, ground at Z = 0.

import adsk.core, adsk.fusion, traceback, math, os, json

MM = 0.1

# ---------------------------------------------------------------- dimensions
P = {
    # running gear
    'wheel_D': 44.0, 'wheel_w': 18.0, 'axle_z': 22.0,
    'axle_front_x': 20.0, 'axle_rear_x': -80.0,
    'motor_y0': 18.0, 'motor_len': 32.0,          # N20 body spans y 18..50
    # decks
    'deck_x0': -115.0, 'deck_x1': 140.0, 'deck_hw': 60.0, 'deck_r': 12.0,
    'deck_t': 3.0, 'lower_z0': 50.0, 'pillar_h': 30.0,
    'upper_x0': -115.0, 'upper_x1': 95.0,
    # intake bay
    'bay_x0': 40.0, 'bay_x1': 128.0, 'bay_hw': 32.0, 'wall_t': 3.0, 'bay_floor_gap': 4.0,
    'door_x': 129.5, 'door_hinge_y': 45.0, 'door_len': 80.0, 'door_t': 3.0, 'door_top': 42.0,
    # servo MG946R (inverted through the lower deck, spline on the door hinge)
    'servo_L': 40.7, 'servo_W': 19.7, 'servo_H': 36.1, 'servo_ear_h': 26.9, 'servo_spline_off': 10.35,
    # sensors
    'mic_spacing': 100.0, 'cam_tilt_deg': 30.0,
    # electronics layout: ESP32-S3 on its perfboard carrier, main power switch through the upper deck
    'esp_x': -63.0, 'switch_x': -85.0, 'switch_y': 40.0,
    # target used in renders and in the demo
    'token_d': 40.0, 'token_h': 40.0,
}

COLORS = {
    'acrylic': (35, 38, 45), 'pla': (234, 108, 28), 'pla_dark': (55, 60, 70), 'brass': (196, 160, 70),
    'rubber': (28, 28, 30), 'rim': (225, 225, 228), 'metal': (165, 170, 178), 'pcb_green': (25, 110, 60),
    'pcb_black': (20, 20, 22), 'pcb_blue': (30, 70, 150), 'pcb_green_unused': (0, 0, 0), 'servo': (22, 22, 24), 'battery': (40, 90, 170),
    'chip': (70, 70, 75), 'token': (230, 40, 40), 'lens': (10, 10, 12),
    'white': (235, 235, 235), 'header': (18, 18, 20), 'pcb_purple': (95, 40, 130), 'led': (40, 220, 60),
    'pot_blue': (40, 90, 200), 'cap': (25, 25, 60), 'inductor': (200, 170, 60), 'heatsink': (190, 195, 200),
    'cell': (30, 120, 70), 'pcb_green': (20, 110, 55), 'terminal': (40, 110, 210), 'nickel': (200, 200, 205), 'gear': (196, 160, 70),
    'w_red': (210, 30, 30), 'w_black': (15, 15, 15), 'w_yellow': (240, 200, 20), 'w_green': (30, 170, 60),
    'w_blue': (40, 90, 220), 'w_orange': (240, 120, 20), 'w_white': (240, 240, 240), 'w_purple': (140, 60, 200),
}
FR4 = ['FR4', 'Epoxy', 'ABS Plastic', 'ABS']


# ---------------------------------------------------------------- helpers
def pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def vec(x, y, z):
    return adsk.core.Vector3D.create(x * MM, y * MM, z * MM)


def mat(tx=0.0, ty=0.0, tz=0.0, rot=None):
    """rot: list of (angle_deg, axis 'x'|'y'|'z') applied in order, then translation."""
    m = adsk.core.Matrix3D.create()
    for ang, ax in (rot or []):
        r = adsk.core.Matrix3D.create()
        axis = {'x': adsk.core.Vector3D.create(1, 0, 0), 'y': adsk.core.Vector3D.create(0, 1, 0),
                'z': adsk.core.Vector3D.create(0, 0, 1)}[ax]
        r.setToRotation(math.radians(ang), axis, adsk.core.Point3D.create(0, 0, 0))
        m.transformBy(r)
    m.translation = vec(tx, ty, tz)
    return m


def place(occ, m):
    try:
        occ.transform2 = m
    except AttributeError:
        occ.transform = m


def new_comp(root, name, m=None):
    """Create the occurrence at its final position. In a parametric design a transform
    given at creation is recorded in the timeline; moving it afterwards is not, and the
    part snaps back to the origin when joints recompute."""
    occ = root.occurrences.addNewComponent(m or adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def xy_plane(comp, z):
    if abs(z) < 1e-9:
        return comp.xYConstructionPlane
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z * MM))
    return comp.constructionPlanes.add(pin)


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def largest_profile(sk):
    best, area = None, -1.0
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        a = p.areaProperties().area
        if a > area:
            best, area = p, a
    return best


def extrude(comp, prof, h, op=NEW):
    return comp.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal(h * MM), op)


def box(comp, sx, sy, sz, cx=0.0, cy=0.0, z0=0.0, op=NEW):
    sk = comp.sketches.add(xy_plane(comp, z0))
    sk.sketchCurves.sketchLines.addCenterPointRectangle(pt(cx, cy), pt(cx + sx / 2, cy + sy / 2))
    return extrude(comp, sk.profiles.item(0), sz, op)


def cyl(comp, d, h, cx=0.0, cy=0.0, z0=0.0, op=NEW):
    sk = comp.sketches.add(xy_plane(comp, z0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy), d / 2 * MM)
    return extrude(comp, sk.profiles.item(0), h, op)


def rounded_rect_sketch(sk, x0, y0, x1, y1, r):
    arcs = sk.sketchCurves.sketchArcs
    lines = sk.sketchCurves.sketchLines
    q = math.pi / 2
    a = [arcs.addByCenterStartSweep(pt(x1 - r, y0 + r), pt(x1 - r, y0), q),
         arcs.addByCenterStartSweep(pt(x1 - r, y1 - r), pt(x1, y1 - r), q),
         arcs.addByCenterStartSweep(pt(x0 + r, y1 - r), pt(x0 + r, y1), q),
         arcs.addByCenterStartSweep(pt(x0 + r, y0 + r), pt(x0, y0 + r), q)]
    for i in range(4):
        lines.addByTwoPoints(a[i].endSketchPoint, a[(i + 1) % 4].startSketchPoint)


def rounded_box(comp, x0, y0, x1, y1, r, z0, h, op=NEW):
    sk = comp.sketches.add(xy_plane(comp, z0))
    rounded_rect_sketch(sk, x0, y0, x1, y1, r)
    return extrude(comp, largest_profile(sk), h, op)


def poly_prism(comp, pts2d, z0, h, op=NEW):
    sk = comp.sketches.add(xy_plane(comp, z0))
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pt(*pts2d[0]), pt(*pts2d[1]))
    prev = first
    for p in pts2d[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, pt(*p))
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
    return extrude(comp, largest_profile(sk), h, op)


def holes(comp, top_z, depth, spec):
    """spec: list of (diameter, x, y); through-cut downward from top_z."""
    sk = comp.sketches.add(xy_plane(comp, top_z))
    for d, x, y in spec:
        sk.sketchCurves.sketchCircles.addByCenterRadius(pt(x, y), d / 2 * MM)
    col = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        col.add(sk.profiles.item(i))
    comp.features.extrudeFeatures.addSimple(col, adsk.core.ValueInput.createByReal(-depth * MM), CUT)


def slot_cut(comp, top_z, depth, x0, y0, x1, y1, r):
    sk = comp.sketches.add(xy_plane(comp, top_z))
    rounded_rect_sketch(sk, x0, y0, x1, y1, r)
    comp.features.extrudeFeatures.addSimple(largest_profile(sk), adsk.core.ValueInput.createByReal(-depth * MM), CUT)


def prism_xz(comp, pts_xz, width, op=NEW):
    """Closed (x, z) outline in the component XZ plane, extruded symmetrically along Y to the full width."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    lines = sk.sketchCurves.sketchLines
    sp = [sk.modelToSketchSpace(pt(x, 0, z)) for x, z in pts_xz]
    first = lines.addByTwoPoints(sp[0], sp[1])
    prev = first
    for p in sp[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, p)
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
    ei = comp.features.extrudeFeatures.createInput(largest_profile(sk), op)
    ei.setSymmetricExtent(adsk.core.ValueInput.createByReal(width * MM), True)
    return comp.features.extrudeFeatures.add(ei)


def hex_prism(comp, af, h, cx, cy, z0):
    r = af / math.sqrt(3.0)
    return poly_prism(comp, [(cx + r * math.cos(math.radians(60 * i)), cy + r * math.sin(math.radians(60 * i)))
                             for i in range(6)], z0, h)


def revolve_z(comp, outline, op=NEW):
    """outline: closed list of (radius, z) points; revolved 360 deg about the component Z axis."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    lines = sk.sketchCurves.sketchLines
    sp = [sk.modelToSketchSpace(pt(r, 0, z)) for r, z in outline]
    first = lines.addByTwoPoints(sp[0], sp[1])
    prev = first
    for p in sp[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, p)
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
    rev = comp.features.revolveFeatures
    inp = rev.createInput(largest_profile(sk), comp.zConstructionAxis, op)
    inp.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    return rev.add(inp)


def origin_point(comp):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'joint_origin'
    return sk.sketchPoints.add(pt(0, 0, 0))


# ---------------------------------------------------------------- look
class Look:
    def __init__(self, app, design):
        self.app, self.design, self.base, self.cache, self.mats = app, design, None, {}, {}
        for lib in app.materialLibraries:
            try:
                apps = lib.appearances
            except Exception:
                continue
            for i in range(apps.count):
                a = apps.item(i)
                if a.name.startswith('Plastic - Matte') or a.name.startswith('Plastic - Glossy'):
                    self.base = a
                    break
            if self.base:
                break

    def appearance(self, key):
        if key in self.cache:
            return self.cache[key]
        a = None
        if self.base:
            name = 'AEGIS ' + key
            a = self.design.appearances.itemByName(name) or self.design.appearances.addByCopy(self.base, name)
            r, g, b = COLORS[key]
            for i in range(a.appearanceProperties.count):
                cp = adsk.core.ColorProperty.cast(a.appearanceProperties.item(i))
                if cp and cp.name.lower() in ('color', 'colour'):
                    cp.value = adsk.core.Color.create(r, g, b, 0)
                    break
        self.cache[key] = a
        return a

    def material(self, names):
        key = names[0]
        if key not in self.mats:
            self.mats[key] = None
            for lib in self.app.materialLibraries:
                for n in names:
                    try:
                        m = lib.materials.itemByName(n)
                    except Exception:
                        m = None
                    if m:
                        self.mats[key] = m
                        break
                if self.mats[key]:
                    break
        return self.mats[key]

    def paint(self, comp, color, material=None):
        m = self.material(material) if material else None
        a = self.appearance(color)
        for b in comp.bRepBodies:
            if m:
                try:
                    b.material = m
                except Exception:
                    pass
            if a:
                b.appearance = a


ACRYLIC = ['Acrylic', 'Plastic - Acrylic', 'PMMA']
PLA = ['PLA', 'Plastic - PLA', 'Polylactic Acid']
BRASS = ['Brass', 'Brass C36000']
STEEL = ['Steel', 'Steel, Alloy', 'Stainless Steel']
RUBBER = ['Rubber', 'Rubber, Silicone', 'Silicone']
ABS = ['ABS Plastic', 'ABS', 'Plastic - ABS']


# ---------------------------------------------------------------- detailed parts
# Each builder draws one Robu module to its published dimensions, origin at the
# centre of the PCB underside (same footprint as the simple version), and paints
# every body in its own material/colour.
def tint(look, feat, key, mats=None):
    m = look.material(mats) if mats else None
    a = look.appearance(key)
    for b in feat.bodies:
        if m:
            try:
                b.material = m
            except Exception:
                pass
        if a:
            b.appearance = a
    return feat


def pins_down(c, look, xs, y, z_top, length=6.0):
    """Row of 0.64 mm square header pins going down from z_top."""
    sk = c.sketches.add(xy_plane(c, z_top))
    for x in xs:
        sk.sketchCurves.sketchLines.addCenterPointRectangle(pt(x, y), pt(x + 0.32, y + 0.32))
    col = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        col.add(sk.profiles.item(i))
    f = c.features.extrudeFeatures.addSimple(col, adsk.core.ValueInput.createByReal(-length * MM), NEW)
    return tint(look, f, 'gear', BRASS)


def row(n, pitch=2.54):
    return [(i - (n - 1) / 2.0) * pitch for i in range(n)]


def d_esp32s3(c, L):                       # Espressif ESP32-S3-DevKitC-1 on two female header strips
    tint(L, box(c, 63.0, 26.0, 1.6), 'pcb_black', FR4)
    tint(L, box(c, 25.5, 18.0, 0.8, 18.75, 0, 1.6), 'pcb_blue', FR4)            # WROOM-1 module
    tint(L, box(c, 17.6, 15.8, 2.4, 15.5, 0, 2.4), 'metal', STEEL)              # RF shield
    for sy in (1, -1):
        tint(L, box(c, 7.5, 9.0, 3.2, -28.0, sy * 6.5, 1.6), 'metal', STEEL)      # USB-C x2
        tint(L, box(c, 4.0, 3.0, 1.5, -21.0, sy * 9.5, 1.6), 'header')            # BOOT / RST
        tint(L, cyl(c, 1.6, 0.6, -21.0, sy * 9.5, 3.1), 'white')
        tint(L, box(c, 55.9, 2.5, 2.5, 0, sy * 11.43, -2.5), 'header')            # male header plastic
        tint(L, box(c, 55.9, 2.6, 8.5, 0, sy * 11.43, -11.0), 'header')           # female socket strip
    tint(L, box(c, 1.6, 0.8, 0.6, -12.0, 0, 1.6), 'led')
    tint(L, box(c, 5.0, 5.0, 1.0, -6.0, 3.0, 1.6), 'chip')                       # USB-UART bridge


def d_mpu6050(c, L):                       # GY-521
    tint(L, box(c, 21.0, 16.0, 1.6), 'pcb_blue', FR4)
    tint(L, box(c, 4.0, 4.0, 0.9, 1.0, 1.5, 1.6), 'chip')
    tint(L, box(c, 3.0, 3.5, 1.2, -6.0, 3.0, 1.6), 'chip')                       # 3V3 regulator
    tint(L, box(c, 20.3, 2.5, 2.5, 0, -6.7, 1.6), 'header')
    tint(L, box(c, 1.6, 0.8, 0.6, 7.0, 4.5, 1.6), 'led')


def d_inmp441(c, L):
    tint(L, rounded_box(c, -7.0, -7.0, 7.0, 7.0, 3.0, 0, 1.6), 'pcb_blue', FR4)
    tint(L, box(c, 4.0, 3.0, 1.0, 0, 2.0, 1.6), 'metal', STEEL)                  # MEMS mic can
    tint(L, box(c, 13.0, 2.5, 2.5, 0, -5.2, 1.6), 'header')


def d_tb6612(c, L):                        # SparkFun-style breakout
    tint(L, box(c, 20.0, 20.0, 1.6), 'pcb_blue', FR4)
    tint(L, box(c, 6.6, 7.8, 1.2, 0, 0, 1.6), 'chip')
    for sy in (1, -1):
        tint(L, box(c, 20.3, 2.5, 2.5, 0, sy * 8.9, 1.6), 'header')
    for x in (-5.5, 5.5):
        tint(L, box(c, 2.0, 1.25, 1.0, x, 4.0, 1.6), 'cap')


def d_xl4015(c, L):                        # XL4015 5 A buck module
    tint(L, box(c, 51.0, 26.0, 1.6), 'pcb_blue', FR4)
    ind = cyl(c, 20.0, 10.0, -7.0, 0, 1.6)                                       # toroidal inductor
    holes(c, 11.6, 10.0, [(9.0, -7.0, 0)])
    tint(L, ind, 'inductor')
    tint(L, box(c, 14.0, 10.0, 2.0, 10.0, -7.0, 1.6), 'heatsink', ['Aluminum', 'Aluminum 6061'])
    for i in range(5):                                                           # heatsink fins
        tint(L, box(c, 1.2, 10.0, 9.0, 4.0 + i * 3.0, -7.0, 3.6), 'heatsink', ['Aluminum', 'Aluminum 6061'])
    for x in (8.0, 16.0):
        tint(L, cyl(c, 8.0, 12.0, x, 9.0, 1.6), 'cap')                           # electrolytics
    for x in (-21.0, 21.0):
        tint(L, box(c, 7.5, 10.0, 10.0, x, 0, 1.6), 'terminal')                  # screw terminals
    tint(L, box(c, 4.5, 7.0, 9.0, -21.0, 9.0, 1.6), 'pot_blue')                 # trimpot
    tint(L, cyl(c, 2.5, 1.0, -21.0, 9.0, 10.6), 'gear')


def d_pcf8574(c, L):
    tint(L, box(c, 40.0, 16.0, 1.6), 'pcb_blue', FR4)
    tint(L, box(c, 10.0, 6.0, 1.6, 0, 0, 1.6), 'chip')
    for i, x in enumerate(row(3, 3.0)):
        tint(L, box(c, 2.5, 4.0, 2.5, x, -6.0, 1.6), 'header')                   # A0-A2 jumpers
    for sx in (1, -1):
        tint(L, box(c, 2.5, 10.2, 2.5, sx * 18.0, 0, 1.6), 'header')


def d_vl53(c, L):                          # GY-530 (25 x 12 carrier)
    tint(L, box(c, 25.0, 12.0, 1.2), 'pcb_purple', FR4)
    tint(L, box(c, 4.4, 2.4, 1.0, 4.0, 0, 1.2), 'chip')                          # VL53L0X
    tint(L, box(c, 2.0, 1.2, 0.4, 4.0, 0, 2.2), 'lens')                        # aperture cover
    tint(L, box(c, 3.0, 3.0, 1.1, -3.0, 2.5, 1.2), 'chip')                       # regulator
    tint(L, box(c, 2.5, 10.2, 2.5, -10.5, 0, 1.2), 'header')


def d_tcrt(c, L):                          # TCRT5000 module, 32 x 14
    tint(L, box(c, 32.0, 14.0, 1.6), 'pcb_black', FR4)
    tint(L, box(c, 10.2, 5.8, 7.0, 10.0, 0, 1.6), 'header')                      # sensor body
    for x in (7.6, 12.4):
        tint(L, cyl(c, 3.0, 0.6, x, 0, 8.6), 'lens' if x < 10 else 'led')
    tint(L, box(c, 5.0, 4.0, 1.5, -2.0, 3.0, 1.6), 'chip')                       # LM393
    tint(L, box(c, 6.5, 6.5, 4.5, -2.0, -3.0, 1.6), 'pot_blue')
    tint(L, box(c, 2.5, 7.6, 2.5, -13.5, 0, 1.6), 'header')


def d_ir_capture(c, L):                    # IR obstacle module, LEDs face the bay
    tint(L, box(c, 14.0, 22.0, 1.6), 'pcb_black', FR4)
    for x, k in ((-3.0, 'white'), (3.0, 'lens')):
        tint(L, cyl(c, 5.0, 6.0, x, 6.0, 1.6), k)
    tint(L, box(c, 6.5, 6.5, 4.5, 0, -2.0, 1.6), 'pot_blue')
    tint(L, box(c, 7.6, 2.5, 2.5, 0, -9.5, 1.6), 'header')


DETAIL = {
    'ESP32_S3_DevKitC': (d_esp32s3, 11.0), 'MPU6050_IMU': (d_mpu6050, 0.0), 'INMP441_Mic_L': (d_inmp441, 0.0),
    'INMP441_Mic_R': (d_inmp441, 0.0), 'TB6612FNG_A': (d_tb6612, 0.0), 'TB6612FNG_B': (d_tb6612, 0.0),
    'XL4015_Logic_5V': (d_xl4015, 0.0), 'XL4015_Servo_5V': (d_xl4015, 0.0), 'PCF8574_IO_Expander': (d_pcf8574, 0.0),
    'VL53L0X_Front': (d_vl53, 0.0), 'VL53L0X_Left': (d_vl53, 0.0), 'VL53L0X_Right': (d_vl53, 0.0),
    'IR_Cliff_FL': (d_tcrt, 0.0), 'IR_Cliff_FR': (d_tcrt, 0.0), 'IR_Cliff_Rear': (d_tcrt, 0.0),
    'IR_Cliff_RL': (d_tcrt, 0.0), 'IR_Cliff_RR': (d_tcrt, 0.0),
    'IR_Capture_Sensor': (d_ir_capture, 0.0),
}


# Placement marks: footprint outlines (0.4 mm wide, 0.3 mm deep) and labels engraved into the decks.
# (label, x, y, w, h) in deck coordinates; sizes are the modules' published board outlines.
# MEASURE THE BOARDS YOU RECEIVE and edit these if a clone differs.
FOOTPRINTS_LOWER_TOP = [
    ('BATTERY 3S 18650', -75.0, 0.0, 65.0, 56.0), ('TB6612 A', -25.0, 32.0, 20.3, 20.3), ('TB6612 B', -25.0, -32.0, 20.3, 20.3),
    ('XL4015 LOGIC 5V0', 32.0, 32.0, 51.0, 26.0), ('XL4015 SERVO 5V0', 32.0, -32.0, 51.0, 26.0),
    ('PCF8574', 80.0, -30.0, 40.0, 16.0), ('TOF FRONT', 135.0, 0.0, 6.0, 25.0), ('FUSE', -58.0, 44.0, 30.0, 14.0),
]
FOOTPRINTS_LOWER_BOTTOM = [
    ('CLIFF FL', 66.0, 52.0, 32.0, 14.0), ('CLIFF FR', 130.0, -50.0, 32.0, 14.0), ('CLIFF R', -108.0, 0.0, 32.0, 14.0),
    ('CLIFF RL', -108.0, 45.0, 14.0, 32.0), ('CLIFF RR', -108.0, -45.0, 14.0, 32.0),
]
FOOTPRINTS_UPPER_TOP = [
    ('ESP32-S3 CARRIER  USB<', -63.0, 0.0, 66.0, 34.0), ('MPU6050', 10.0, 0.0, 21.0, 16.0), ('MIC L', 0.0, 50.0, 14.0, 14.0),
    ('MIC R', 0.0, -50.0, 14.0, 14.0), ('CAM MAST', 60.0, 0.0, 22.0, 34.0), ('TOF L', 40.0, 55.5, 25.0, 6.0),
    ('TOF R', 40.0, -55.5, 25.0, 6.0),
]


def engrave(comp, z_face, items, downward, dxf_path=None, line=0.4, depth=0.3, text_h=3.0):
    """Engrave footprint outlines and labels on a deck face. downward=True for a top face."""
    report = []
    sk = comp.sketches.add(xy_plane(comp, z_face))
    sk.name = 'engrave_marks'
    lines = sk.sketchCurves.sketchLines
    for label, x, y, w, h in items:
        lines.addCenterPointRectangle(pt(x, y), pt(x + w / 2, y + h / 2))
        lines.addCenterPointRectangle(pt(x, y), pt(x + w / 2 - line, y + h / 2 - line))
    rings = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        prof = sk.profiles.item(i)
        if prof.profileLoops.count == 2:                     # the thin ring between the two rectangles
            rings.add(prof)
    if rings.count:
        comp.features.extrudeFeatures.addSimple(rings, adsk.core.ValueInput.createByReal((-depth if downward else depth) * MM), CUT)
    # labels (best effort: text APIs differ between Fusion versions)
    tsk = comp.sketches.add(xy_plane(comp, z_face))
    tsk.name = 'engrave_labels'
    for label, x, y, w, h in items:
        try:
            th = min(text_h, h * 0.35)
            if len(label) * th * 0.7 > w - 2 * line - 1.0:
                th = max(1.5, (w - 2 * line - 1.0) / (len(label) * 0.7))
            ti = tsk.sketchTexts.createInput2(label, th * MM)
            ti.setAsMultiLine(pt(x - w / 2 + line + 0.6, y - h / 2 + line + 0.6), pt(x + w / 2 - line - 0.6, y + h / 2 - line - 0.6),
                              adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                              adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
            if not downward:
                ti.isHorizontalFlip = True                   # reads correctly when seen from below
            txt = tsk.sketchTexts.add(ti)
            ei = comp.features.extrudeFeatures.createInput(txt, CUT)
            ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal((-depth if downward else depth) * MM))
            comp.features.extrudeFeatures.add(ei)
        except Exception as e:
            report.append('label %s: %s' % (label, e))
    if dxf_path:
        try:
            sk.saveAsDXF(dxf_path)
            tsk.saveAsDXF(dxf_path.replace('.dxf', '_labels.dxf'))
        except Exception as e:
            report.append('dxf: %s' % e)
    return report


def wire(c, look, pts, d, color):
    """A cable: pipe of diameter d along a fitted 3D spline through pts (mm, world frame)."""
    sk = c.sketches.add(c.xYConstructionPlane)
    col = adsk.core.ObjectCollection.create()
    for p in pts:
        col.add(pt(*p))
    spl = sk.sketchCurves.sketchFittedSplines.add(col)
    path = c.features.createPath(spl, False)
    pin = c.features.pipeFeatures.createInput(path, NEW)
    pin.sectionSize = adsk.core.ValueInput.createByReal(d * MM)
    f = c.features.pipeFeatures.add(pin)
    return tint(look, f, color, ['Rubber', 'Rubber, Silicone', 'Silicone'])


# ---------------------------------------------------------------- the build
def build(out_dir=None, detailed=False):
    app = adsk.core.Application.get()
    log = []

    for i in range(app.documents.count - 1, -1, -1):        # close earlier bridge builds only
        d = app.documents.item(i)
        try:
            prod = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
            if prod and prod.attributes.itemByName('AEGIS', 'bridge_build'):
                d.close(False)
        except Exception:
            pass

    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    design.attributes.add('AEGIS', 'bridge_build', '1')
    root = design.rootComponent
    look = Look(app, design)
    for k, v in P.items():
        if k != 'cam_tilt_deg':
            design.userParameters.add(k, adsk.core.ValueInput.createByString('%g mm' % v), 'mm', '')

    # derived heights
    lz0, lz1 = P['lower_z0'], P['lower_z0'] + P['deck_t']
    uz0 = lz1 + P['pillar_h']
    uz1 = uz0 + P['deck_t']
    hw, bhw, wt = P['deck_hw'], P['bay_hw'], P['wall_t']
    dx, hy = P['door_x'], P['door_hinge_y']
    servo_cx = dx - P['servo_spline_off']
    servo_below = P['servo_H'] - P['servo_ear_h']
    axles = [P['axle_front_x'], P['axle_rear_x']]
    pillars = [(-100.0, 52.0), (-100.0, -52.0), (85.0, 52.0), (85.0, -52.0)]
    hanger_bolts = [(ax + sx, sy * 34.0) for ax in axles for sx in (-6.5, 6.5) for sy in (1, -1)]
    shroud_bolts = [(x, sy * 38.5) for x in (55.0, 85.0) for sy in (1, -1)]
    mast_bolts = [(64.0, sy * 11.0) for sy in (1, -1)]               # M3 x 10 up through the upper deck, nut on the mast base
    rigid = []

    # --- lower deck -----------------------------------------------------------------
    o = new_comp(root, 'Lower_Deck_Acrylic')
    c = o.component
    rounded_box(c, P['deck_x0'], -hw, P['deck_x1'], hw, P['deck_r'], lz0, P['deck_t'])
    holes(c, lz1, P['deck_t'], [(3.2, x, y) for x, y in pillars + hanger_bolts + shroud_bolts] +
          [(3.2, servo_cx + sx * 24.0, hy) for sx in (-1, 1)])
    slot_cut(c, lz1, P['deck_t'], servo_cx - 20.65, hy - 10.15, servo_cx + 20.65, hy + 10.15, 1.0)   # servo pocket
    slot_cut(c, lz1, P['deck_t'], -30.0, -6.0, 10.0, 6.0, 6.0)                                      # harness slot
    slot_cut(c, lz1, P['deck_t'], 60.0, -20.0, 100.0, 20.0, 8.0)                                     # bay inspection window
    look.paint(c, 'acrylic', ACRYLIC)
    o.isGrounded = True
    lower = o
    rigid.append(o)

    # --- upper deck -----------------------------------------------------------------
    o = new_comp(root, 'Upper_Deck_Acrylic')
    c = o.component
    rounded_box(c, P['upper_x0'], -hw, P['upper_x1'], hw, P['deck_r'], uz0, P['deck_t'])
    holes(c, uz1, P['deck_t'], [(3.2, x, y) for x, y in pillars + mast_bolts])
    slot_cut(c, uz1, P['deck_t'], -30.0, -6.0, 10.0, 6.0, 6.0)
    sw_x, sw_y = P['switch_x'], P['switch_y']                       # rocker switch cutout (19.4 x 13.2)
    slot_cut(c, uz1, P['deck_t'], sw_x - 9.7, sw_y - 6.6, sw_x + 9.7, sw_y + 6.6, 0.5)
    look.paint(c, 'acrylic', ACRYLIC)
    rigid.append(o)

    # --- pillars: 20 mm F-F + 10 mm M-F stacked, modelled as one 30 mm hex --------
    o = new_comp(root, 'M3_Pillar_30mm', mat(pillars[0][0], pillars[0][1], lz1))
    c = o.component
    hex_prism(c, 5.5, P['pillar_h'], 0, 0, 0)
    holes(c, P['pillar_h'], P['pillar_h'], [(2.5, 0, 0)])
    look.paint(c, 'brass', BRASS)
    rigid.append(o)
    for x, y in pillars[1:]:
        rigid.append(root.occurrences.addExistingComponent(c, mat(x, y, lz1)))

    # --- N20 gearmotor, built along +Z (shaft tip direction) -------------------------
    corners = [(ax, side) for ax in axles for side in (1, -1)]

    def corner_mats(ax, side):
        r = [(-90.0 * side, 'x')]          # local +Z -> world +Y on the left, -Y on the right
        return (mat(ax, side * (P['motor_y0'] + P['motor_len']), P['axle_z'], r),
                mat(ax, side * (P['motor_y0'] + P['motor_len'] + 2.0 + P['wheel_w'] / 2), P['axle_z'], r),
                mat(ax, side * 34.0, 0.0))
    m0_mot, m0_whl, m0_hng = corner_mats(*corners[0])
    o = new_comp(root, 'N20_Gearmotor_150to1', m0_mot)
    c = o.component
    if detailed:
        tint(look, cyl(c, 3.0, 10.0, 0, 0, 0), 'metal', STEEL)                        # D-shaft
        tint(look, cyl(c, 6.0, 1.5, 0, 0, -1.5), 'gear', BRASS)                       # bushing
        for z0 in (-2.5, -10.5):
            tint(look, box(c, 12.0, 10.0, 1.0, 0, 0, z0), 'metal', STEEL)             # gearbox plates
        for sx in (-1, 1):
            for sy in (-1, 1):
                tint(look, cyl(c, 1.4, 7.0, sx * 4.8, sy * 3.8, -9.5), 'metal', STEEL)  # plate posts
        tint(look, cyl(c, 8.0, 7.0, -1.0, 0, -9.5), 'gear', BRASS)                   # gear train
        tint(look, rounded_box(c, -6.0, -5.0, 6.0, 5.0, 4.0, -25.5, 15.0), 'metal', STEEL)  # can
        tint(look, cyl(c, 10.0, 1.6, 0, 0, -27.1), 'pcb_green', FR4)                  # encoder PCB
        tint(look, cyl(c, 6.0, 1.5, 0, 0, -28.6), 'header')                           # magnet disc
        tint(look, box(c, 6.0, 3.0, 3.0, 0, 3.5, -31.6), 'white')                     # 6-pin JST
    else:
        cyl(c, 3.0, 10.0, 0, 0, 0)                                     # D-shaft z 0..10
        cyl(c, 6.0, 1.5, 0, 0, -1.5)                                   # bearing boss
        box(c, 12.0, 10.0, 9.0, 0, 0, -10.5)                           # gearbox
        rounded_box(c, -6.0, -5.0, 6.0, 5.0, 4.0, -25.5, 15.0)         # motor can
        cyl(c, 10.0, 1.6, 0, 0, -27.1)                                 # encoder PCB (fits under the hanger bridge)
        look.paint(c, 'metal', STEEL)
    motor_comp = c
    motor_occ = o

    # --- motor hanger (printed), straddles the motor, bolts under the lower deck -----
    o = new_comp(root, 'Motor_Hanger_PLA', m0_hng)
    c = o.component
    box(c, 3.0, 26.0, 35.0, -7.7, 0, 15.0)                         # cheeks
    box(c, 3.0, 26.0, 35.0, 7.7, 0, 15.0)
    box(c, 18.4, 26.0, 22.8, 0, 0, 27.2, JOIN)                     # bridge to the deck
    holes(c, 50.0, 22.8, [(2.5, -6.5, 0), (2.5, 6.5, 0)])
    for sy in (1, -1):                                             # 2.5 mm cable-tie path that holds the motor up in the slot:
        box(c, 18.4, 3.5, 2.0, 0, sy * 8.0, 28.5, CUT)             # tunnel through the bridge just above the gearbox / can
        box(c, 18.4, 3.5, 2.2, 0, sy * 8.0, 15.0, CUT)             # notch in both cheek bottoms so the tie bears on the motor
    look.paint(c, 'pla_dark', PLA)
    hanger_comp = c
    hanger_occ = o

    # --- wheel: revolved tyre and rim, built about +Z ------------------------------
    o = new_comp(root, 'Wheel_44mm', m0_whl)
    c = o.component
    R, hwid = P['wheel_D'] / 2, P['wheel_w'] / 2
    if detailed:
        # tyre: rubber ring with three circumferential grooves (1 mm deep) for grip on a table top
        g = []
        for z0 in (-4.5, 0.0, 4.5):
            g += [(R, z0 - 0.6), (R - 1.0, z0 - 0.6), (R - 1.0, z0 + 0.6), (R, z0 + 0.6)]
        outline = [(R - 4.0, -hwid), (R - 1.2, -hwid), (R, -hwid + 1.2)] + g + [(R, hwid - 1.2), (R - 1.2, hwid), (R - 4.0, hwid)]
        tint(look, revolve_z(c, outline), 'rubber', RUBBER)
        tint(look, revolve_z(c, [(1.5, -hwid), (7.0, -hwid), (7.0, -hwid + 1.5), (R - 4.0, -hwid + 1.5), (R - 4.0, hwid - 1.5),
                                 (7.0, hwid - 1.5), (7.0, hwid), (1.5, hwid)]), 'white', ABS)   # hub
    else:
        revolve_z(c, [(1.5, -hwid), (7.0, -hwid), (7.0, -hwid + 1), (R - 3.5, -hwid + 1), (R - 3.5, -hwid),
                      (R - 1.5, -hwid), (R, -hwid + 1.5), (R, hwid - 1.5), (R - 1.5, hwid), (R - 3.5, hwid),
                      (R - 3.5, hwid - 1), (7.0, hwid - 1), (7.0, hwid), (1.5, hwid)])
        look.paint(c, 'rubber', RUBBER)
    wheel_comp = c
    wheel_occ = o
    wheel_pt = origin_point(c)

    motors, wheels, hangers = [motor_occ], [wheel_occ], [hanger_occ]
    for ax, side in corners[1:]:
        m_mot, m_whl, m_hng = corner_mats(ax, side)
        motors.append(root.occurrences.addExistingComponent(motor_comp, m_mot))
        wheels.append(root.occurrences.addExistingComponent(wheel_comp, m_whl))
        hangers.append(root.occurrences.addExistingComponent(hanger_comp, m_hng))
    rigid += motors + hangers

    # --- intake shroud (printed): side walls, back wall, mounting flanges, right fin --
    o = new_comp(root, 'Intake_Shroud_PLA')
    c = o.component
    fz = P['bay_floor_gap']
    wall_h = lz0 - fz
    box(c, P['bay_x1'] - P['bay_x0'], wt, wall_h, (P['bay_x0'] + P['bay_x1']) / 2, bhw + wt / 2, fz)
    box(c, P['bay_x1'] - P['bay_x0'], wt, wall_h, (P['bay_x0'] + P['bay_x1']) / 2, -bhw - wt / 2, fz, JOIN)
    box(c, wt, 2 * (bhw + wt), wall_h, P['bay_x0'] - wt / 2, 0, fz, JOIN)
    for sy in (1, -1):                                   # flanges stop short of the servo (x < 96)
        box(c, 56.0, 9.0, 3.0, 68.0, sy * (bhw + wt + 4.5), lz0 - 3.0, JOIN)
    holes(c, lz0, 3.0, [(3.2, x, y) for x, y in shroud_bolts])
    box(c, 3.0, wt, lz0 - 3.0 - (P['door_top'] + 1.0), P['bay_x1'] + 1.5, -bhw - wt / 2, P['door_top'] + 1.0, JOIN)  # fin bridge above the door
    poly_prism(c, [(P['bay_x1'] + 3.0, -bhw), (P['bay_x1'] + 3.0, -bhw - wt), (P['bay_x1'] + 20.0, -bhw - 13.0),
                   (P['bay_x1'] + 20.0, -bhw - 10.0)], fz, wall_h, JOIN)                    # right funnel fin (full height: prints flange-down)
    box(c, 3.0, 12.0, 16.0, P['bay_x0'] + 1.5, 0, 18.0, JOIN)                                # capture-sensor boss
    box(c, 32.6, 14.6, 2.0, 130.0, -50.0, lz0 - 2.0, CUT)                                    # clearance for the FR cliff sensor
    look.paint(c, 'pla', PLA)
    rigid.append(o)

    # --- door (printed): vertical leaf hinged on a vertical axis at the left -------
    o = new_comp(root, 'Intake_Door_PLA', mat(dx, hy, fz))
    c = o.component
    door_h = P['door_top'] - fz
    box(c, P['door_t'], P['door_len'], door_h, 0, -P['door_len'] / 2, 0)                     # leaf
    cyl(c, 12.0, 10.0, 0, 0, door_h - 8.5, JOIN)                                             # spline hub (top 1.5 above leaf)
    cyl(c, 8.0, 10.0, 0, 0, 0, JOIN)                                                          # lower knuckle
    for i in range(4):                                                                        # outside stiffening ribs
        box(c, 2.0, 2.0, door_h, 2.5, -14.0 - i * 18.0, 0, JOIN)
    holes(c, door_h + 1.5, 4.0, [(5.8, 0, 0)])                                                # spline socket
    revolve_z(c, [(0.0, door_h - 13.0), (1.5, door_h - 13.0), (6.0, door_h - 8.5), (0.0, door_h - 8.5)], JOIN)  # 45 deg cone under the hub: prints without support
    look.paint(c, 'pla', PLA)
    door_occ = o
    door_pt = origin_point(c)

    # --- servo MG946R, inverted through the lower deck -----------------------------
    o = new_comp(root, 'Servo_MG946R', mat(servo_cx, hy, lz1))
    c = o.component
    if detailed:
        tint(look, box(c, P['servo_L'], P['servo_W'], servo_below - 1.0, 0, 0, -servo_below + 1.0), 'servo', ABS)  # top cap
        tint(look, cyl(c, 13.0, 1.0, P['servo_spline_off'], 0, -servo_below), 'servo', ABS)                # spline boss
        tint(look, cyl(c, 5.8, 4.0, P['servo_spline_off'], 0, -servo_below - 4.0), 'gear', BRASS)          # 25T spline
        tint(look, box(c, P['servo_L'] + 14.0, P['servo_W'], 2.5, 0, 0, 0), 'servo', ABS)                # ears
        holes(c, 2.5, 2.5, [(4.0, sx * 24.0, 0) for sx in (-1, 1)])
        tint(look, box(c, P['servo_L'], P['servo_W'], P['servo_ear_h'] - 2.5, 0, 0, 2.5), 'servo', ABS)    # case
        tint(look, box(c, P['servo_L'] - 6.0, 0.4, 14.0, 0, P['servo_W'] / 2 + 0.2, 8.0), 'white')         # label
        tint(look, box(c, 3.0, 6.0, 3.0, -P['servo_L'] / 2 - 1.5, 0, 18.0), 'header')                    # cable exit
    else:
        box(c, P['servo_L'], P['servo_W'], P['servo_H'], 0, 0, -servo_below)
        box(c, P['servo_L'] + 14.0, P['servo_W'], 2.5, 0, 0, 0, JOIN)                             # ears
        cyl(c, 5.8, 4.0, P['servo_spline_off'], 0, -servo_below - 4.0, JOIN)                      # output spline
        holes(c, 2.5, 2.5, [(4.0, sx * 24.0, 0) for sx in (-1, 1)])
        look.paint(c, 'servo', ABS)
    rigid.append(o)

    # --- battery: 3S1P 18650 pack (three cells under shrink wrap) -------------------
    if detailed:
        # three 18650 cells lying along X (built along local Z, rotated), BMS board at one end
        o = new_comp(root, 'Battery_3S1P_18650', mat(-107.5, 0, lz1, [(90.0, 'y')]))
        c = o.component
        for k in (-1, 0, 1):
            tint(look, cyl(c, 18.2, 64.0, -9.5, k * 18.6, 0.5), 'cell', ['Aluminum', 'Aluminum 6061'])
            tint(look, cyl(c, 7.0, 0.5, -9.5, k * 18.6, 64.5), 'nickel', STEEL)
            tint(look, cyl(c, 7.0, 0.5, -9.5, k * 18.6, 0.0), 'nickel', STEEL)
        tint(look, box(c, 17.0, 52.0, 1.4, -9.5, 0, 65.0), 'pcb_green', FR4)                     # BMS
        tint(look, box(c, 6.0, 2.0, 1.0, -9.5, 8.0, 66.4), 'chip')
        tint(look, box(c, 5.0, 10.0, 6.0, -9.5, -15.0, 66.4), 'white')                         # JST-XH balance plug
    else:
        o = new_comp(root, 'Battery_3S1P_18650', mat(-75.0, 0, lz1))
        c = o.component
        rounded_box(c, -32.5, -28.0, 32.5, 28.0, 8.0, 0, 19.0)
        look.paint(c, 'battery', ['Aluminum', 'Aluminum 6061'])         # Li-ion cells ~2.5 g/cm3
    rigid.append(o)

    # --- electronics -----------------------------------------------------------------
    def board(name, sx, sy, t, color, parts=(), at=(0, 0, 0), rot=None):
        if detailed and name in DETAIL:
            fn, lift = DETAIL[name]
            oo = new_comp(root, name, mat(at[0], at[1], at[2] + lift, rot))
            fn(oo.component, look)
            rigid.append(oo)
            return oo
        oo = new_comp(root, name, mat(at[0], at[1], at[2], rot))
        cc = oo.component
        box(cc, sx, sy, t, 0, 0, 0)
        look.paint(cc, color, ['FR4', 'Epoxy', 'ABS Plastic', 'ABS'])
        for (px, py, psx, psy, psz, pcol) in parts:
            b = box(cc, psx, psy, psz, px, py, t)
            if look.appearance(pcol):
                for bb in b.bodies:
                    bb.appearance = look.appearance(pcol)
        rigid.append(oo)
        return oo

    board('ESP32_S3_DevKitC', 63.0, 26.0, 1.6, 'pcb_black',
          [(14.0, 0, 18.0, 19.5, 3.2, 'metal'), (-29.0, 0, 7.5, 9.0, 3.0, 'metal'),
           (0, 11.5, 50.0, 2.5, 8.0, 'pcb_black'), (0, -11.5, 50.0, 2.5, 8.0, 'pcb_black')], (P['esp_x'], 0, uz1 + 1.6))
    board('ESP32_Carrier_Perfboard', 66.0, 34.0, 1.6, 'pcb_green', [], (P['esp_x'], 0, uz1))   # sockets, pull-ups, divider
    board('MPU6050_IMU', 21.0, 16.0, 1.6, 'pcb_blue', [(0, 0, 4.0, 4.0, 1.0, 'chip')], (10.0, 0, uz1))
    for sy, nm in ((1, 'INMP441_Mic_L'), (-1, 'INMP441_Mic_R')):
        board(nm, 14.0, 14.0, 1.6, 'pcb_blue', [(0, 0, 3.5, 4.5, 1.0, 'metal')],
              (0.0, sy * P['mic_spacing'] / 2, uz1))
    board('TB6612FNG_A', 20.0, 20.0, 1.6, 'pcb_blue', [(0, 0, 5.0, 4.0, 1.2, 'chip')], (-25.0, 32.0, lz1))
    board('TB6612FNG_B', 20.0, 20.0, 1.6, 'pcb_blue', [(0, 0, 5.0, 4.0, 1.2, 'chip')], (-25.0, -32.0, lz1))
    for sy, nm in ((1, 'XL4015_Logic_5V'), (-1, 'XL4015_Servo_5V')):
        b = board(nm, 51.0, 26.0, 1.6, 'pcb_blue',
                  [(-8.0, 0, 12.0, 12.0, 8.0, 'servo'), (12.0, 0, 10.0, 10.0, 4.0, 'metal')],
                  (32.0, sy * 32.0, lz1))
        if not detailed:
            cyl(b.component, 8.0, 12.0, -20.0, 6.0, 1.6)                                      # capacitor
    board('PCF8574_IO_Expander', 40.0, 16.0, 1.6, 'pcb_blue', [(0, 0, 9.0, 6.0, 2.0, 'chip')], (80.0, -30.0, lz1))
    board('Fuse_Holder_Board', 30.0, 14.0, 1.6, 'pcb_green', [(0, 0, 22.0, 7.0, 9.0, 'pla_dark')], (-58.0, 44.0, lz1))

    # main power switch (oval rocker, KCD1 style): bezel on the upper deck, body and tabs between the decks
    o = new_comp(root, 'Power_Switch_Rocker', mat(P['switch_x'], P['switch_y'], uz1))
    c = o.component
    box(c, 21.0, 15.0, 2.0, 0, 0, 0)                                   # bezel
    box(c, 16.0, 10.0, 4.0, 0, 0, 2.0, JOIN)                           # rocker
    box(c, 19.0, 12.8, 18.0, 0, 0, -18.0, JOIN)                        # body through the deck
    for dx in (-4.0, 4.0):
        box(c, 0.8, 4.8, 7.0, dx, 0, -25.0, JOIN)                      # solder tabs
    look.paint(c, 'pla_dark', ['ABS Plastic', 'ABS'])
    rigid.append(o)

    # ToF sensors on vertical carriers (board normal = viewing direction)
    def tof(name, x, y, yaw):
        nonlocal z
        oo = board(name, 25.0, 12.0, 1.2, 'pcb_black', [(0, 0, 4.4, 2.4, 1.0, 'chip')],
                   (x, y, z), [(90.0, 'y'), (yaw, 'z')])
        return oo
    z = lz1 + 13.5
    tof('VL53L0X_Front', 135.0, 0.0, 0.0)                    # front lip of the lower deck, below the camera's view
    z = uz1 + 13.5
    tof('VL53L0X_Left', 40.0, 57.0, 90.0)
    tof('VL53L0X_Right', 40.0, -57.0, -90.0)

    # IR modules: three table-edge (cliff) sensors under the lower deck, one capture sensor in the bay
    for nm, x, y in (('IR_Cliff_FL', 66.0, 52.0), ('IR_Cliff_FR', 130.0, -50.0), ('IR_Cliff_Rear', -108.0, 0.0)):
        board(nm, 32.0, 14.0, 1.6, 'pcb_black', [(10.0, 0, 10.0, 6.0, 5.0, 'lens')], (x, y, lz0), [(180.0, 'x')])
    # rear-corner cliff sensors (board along Y, lens outboard near each rear wheel)
    for nm, sy in (('IR_Cliff_RL', 1), ('IR_Cliff_RR', -1)):
        board(nm, 32.0, 14.0, 1.6, 'pcb_black', [(10.0, 0, 10.0, 6.0, 5.0, 'lens')], (-108.0, sy * 45.0, lz0),
              [(180.0, 'x'), (-90.0 * sy, 'z')])
    board('IR_Capture_Sensor', 14.0, 22.0, 1.6, 'pcb_black', [(0, 6.0, 6.0, 9.0, 5.0, 'lens')],
          (P['bay_x0'] + 3.0, 0.0, 26.0), [(90.0, 'y')])

    # camera mast (printed) and ESP32-CAM tilted down
    o = new_comp(root, 'Camera_Mast_PLA', mat(60.0, 0, uz1))
    c = o.component
    box(c, 22.0, 34.0, 3.0, 0, 0, 0)
    box(c, 6.0, 34.0, 56.0, -6.0, 0, 3.0, JOIN)
    # camera pad: its sloped face is parallel to the ESP32-CAM back, 2 mm behind it (foam tape)
    prism_xz(c, [(-3.5, 38.4), (8.89, 59.0), (-3.5, 59.0)], 34.0, JOIN)
    holes(c, 3.0, 3.0, [(3.2, x - 60.0, y) for x, y in mast_bolts])
    look.paint(c, 'pla', PLA)
    rigid.append(o)
    cam = new_comp(root, 'ESP32_CAM_OV2640', mat(66.0, 0, uz1 + 50.0, [(90.0 + P['cam_tilt_deg'], 'y')]))
    c = cam.component
    if detailed:
        tint(look, box(c, 27.0, 40.0, 1.6, 0, 0, 0), 'pcb_black', FR4)
        tint(look, box(c, 17.0, 13.0, 2.6, 0, -9.0, 1.6), 'metal', STEEL)            # ESP32-S shield
        tint(look, box(c, 10.0, 4.0, 1.2, 0, 2.0, 1.6), 'header')                    # FPC connector
        tint(look, box(c, 8.5, 8.5, 4.0, 0, 9.0, 1.6), 'header')                     # OV2640 module
        tint(look, cyl(c, 7.5, 4.0, 0, 9.0, 5.6), 'lens')                            # lens barrel
        tint(look, cyl(c, 3.5, 0.4, 0, 9.0, 9.6), 'pcb_blue')                        # lens glass
        tint(look, box(c, 14.0, 15.0, 1.8, 0, -4.0, -1.8), 'metal', STEEL)           # microSD (back)
        tint(look, box(c, 1.6, 0.8, 0.6, 9.0, 14.0, 1.6), 'white')                   # flash LED
    else:
        box(c, 27.0, 40.0, 1.6, 0, 0, 0)                                    # PCB, lens on +Z side
        box(c, 8.5, 8.5, 5.0, 0, 6.0, 1.6, JOIN)                            # camera module
        cyl(c, 7.0, 4.0, 0, 6.0, 6.6, JOIN)                                 # lens barrel
        look.paint(c, 'pcb_black', ['FR4', 'Epoxy', 'ABS Plastic', 'ABS'])
    rigid.append(cam)

    # --- target token used in demos and renders --------------------------------------
    o = new_comp(root, 'Victim_Token_40mm', mat(250.0, -10.0, 0.0))
    c = o.component
    cyl(c, P['token_d'], P['token_h'], 0, 0, 0)
    look.paint(c, 'token', PLA)
    token = o

    # --- laser-cut DXFs: written BEFORE engraving so they hold only the outline and through-holes
    cut_dxf_notes = {}
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        for occ_name, fn in (('Lower_Deck_Acrylic', 'lasercut_lower_deck.dxf'), ('Upper_Deck_Acrylic', 'lasercut_upper_deck.dxf')):
            try:
                cc = [o2 for o2 in root.occurrences if o2.component.name == occ_name][0].component
                body = cc.bRepBodies.item(0)
                top = max((f for f in body.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType),
                          key=lambda f: f.area)
                sk = cc.sketches.add(top)
                sk.name = 'laser_cut_outline'
                sk.project(top)
                sk.saveAsDXF(os.path.join(out_dir, fn))
            except Exception as e:
                cut_dxf_notes['dxf_error_' + fn] = str(e)

    # --- placement marks engraved into both decks --------------------------------------------
    eng_dir = out_dir if out_dir else None
    lower_c = [o2 for o2 in root.occurrences if o2.component.name == 'Lower_Deck_Acrylic'][0].component
    upper_c = [o2 for o2 in root.occurrences if o2.component.name == 'Upper_Deck_Acrylic'][0].component
    eng_report = []
    if eng_dir:
        os.makedirs(eng_dir, exist_ok=True)
    eng_report += engrave(lower_c, lz1, FOOTPRINTS_LOWER_TOP, True,
                          os.path.join(eng_dir, 'engrave_lower_deck_top.dxf') if eng_dir else None)
    eng_report += engrave(lower_c, lz0, FOOTPRINTS_LOWER_BOTTOM, False,
                          os.path.join(eng_dir, 'engrave_lower_deck_underside.dxf') if eng_dir else None)
    eng_report += engrave(upper_c, uz1, FOOTPRINTS_UPPER_TOP, True,
                          os.path.join(eng_dir, 'engrave_upper_deck_top.dxf') if eng_dir else None)

    # --- wiring harness (detailed model) -------------------------------------------------
    wiring = None
    if detailed:
        wiring = new_comp(root, 'Wiring_Harness')
        wc = wiring.component
        bt = lz1 + 10.0
        # battery -> both bucks (input terminals) and both motor drivers (VM): red / black
        for sy in (1, -1):
            wire(wc, look, [(-42.0, 3.0, bt), (-15.0, sy * 12.0, bt + 8), (11.0, sy * 34.0, lz1 + 14.0)], 2.2, 'w_red')
            wire(wc, look, [(-42.0, -3.0, bt), (-15.0, sy * 8.0, bt + 5), (11.0, sy * 30.0, lz1 + 14.0)], 2.2, 'w_black')
            wire(wc, look, [(-42.0, 0.0, bt - 4), (-30.0, sy * 20.0, lz1 + 8), (-25.0, sy * 41.0, lz1 + 4.2)], 1.6, 'w_red')
        # logic buck -> ESP32 5 V through the upper-deck slot
        wire(wc, look, [(53.0, 30.0, lz1 + 14.0), (20.0, 10.0, lz1 + 22.0), (-10.0, 0.0, uz0 - 4.0), (-10.0, 0.0, uz1 + 3.0),
                        (-53.0, 11.4, uz1 + 9.0)], 1.6, 'w_red')
        # battery + -> power switch -> fuse board, before the distribution above
        wire(wc, look, [(-39.0, 10.0, bt), (-39.0, 32.0, lz1 + 12.0), (-80.0, 34.0, lz1 + 10.0),
                        (P['switch_x'] - 4.0, P['switch_y'], lz1 + 6.5)], 2.2, 'w_red')
        wire(wc, look, [(P['switch_x'] + 4.0, P['switch_y'], lz1 + 6.5), (-76.0, 48.0, lz1 + 7.0), (-71.0, 44.0, lz1 + 5.0)], 2.2, 'w_red')
        # servo buck -> servo
        wire(wc, look, [(53.0, -30.0, lz1 + 14.0), (80.0, 0.0, lz1 + 20.0), (servo_cx - 22.0, hy, lz1 + 18.0)], 1.6, 'w_orange')
        # motor leads: drivers -> through the harness slot -> each motor's encoder end
        for (ax, side), col in zip(corners, ('w_red', 'w_black', 'w_red', 'w_black')):
            wire(wc, look, [(-25.0, side * 25.0, lz1 + 4.0), (-12.0, side * 3.0, lz1 + 2.0), (-12.0, side * 3.0, lz0 - 4.0),
                            (ax, side * 20.0, 30.0), (ax, side * 16.0, 24.0)], 1.4, col)
        # I2C bus (yellow SDA, green SCL): ESP32 -> IMU -> side ToFs -> through slot -> PCF8574 and front ToF
        for off, col in ((0.0, 'w_yellow'), (2.0, 'w_green')):
            wire(wc, look, [(-48.0, -11.4 + off, uz1 + 9.0), (10.0, -6.0 + off, uz1 + 4.0), (40.0, -45.0 + off, uz1 + 6.0),
                            (40.0, -55.0, uz1 + 8.0)], 1.2, col)
            wire(wc, look, [(10.0, 6.0 + off, uz1 + 4.0), (40.0, 45.0 + off, uz1 + 6.0), (40.0, 55.0, uz1 + 8.0)], 1.2, col)
            wire(wc, look, [(-43.0, -11.4 + off, uz1 + 9.0), (-15.0, -2.0 + off, uz1 + 5.0), (-15.0, -2.0 + off, uz0 - 2.0), (40.0, -20.0 + off, lz1 + 6.0),
                            (80.0, -24.0 + off, lz1 + 4.5), (133.0, 2.0 + off, lz1 + 8.0)], 1.2, col)
        # I2S microphones (blue) and camera UART (white)
        for sy in (1, -1):
            wire(wc, look, [(-58.0, sy * 11.4, uz1 + 9.0), (-10.0, sy * 30.0, uz1 + 4.0), (0.0, sy * 44.0, uz1 + 3.0)], 1.2, 'w_blue')
        wire(wc, look, [(-68.0, -11.4, uz1 + 9.0), (20.0, -8.0, uz1 + 12.0), (55.0, -10.0, uz1 + 35.0), (62.0, -12.0, uz1 + 48.0)],
             1.2, 'w_white')

    # --- assembly relationships --------------------------------------------------------
    col = adsk.core.ObjectCollection.create()
    for oo in rigid:
        col.add(oo)
    root.rigidGroups.add(col, True)

    joints = root.asBuiltJoints
    for w in wheels:
        g = adsk.fusion.JointGeometry.createByPoint(wheel_pt.createForAssemblyContext(w))
        ji = joints.createInput(w, lower, g)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,
                                    wheel_comp.zConstructionAxis.createForAssemblyContext(w))
        joints.add(ji)
    g = adsk.fusion.JointGeometry.createByPoint(door_pt.createForAssemblyContext(door_occ))
    ji = joints.createInput(door_occ, lower, g)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,
                                door_occ.component.zConstructionAxis.createForAssemblyContext(door_occ))
    door_joint = joints.add(ji)
    lim = door_joint.jointMotion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = 0.0
    lim.isMaximumValueEnabled = True
    lim.maximumValue = math.radians(120.0)          # 110 deg open: the leaf becomes the left funnel wall

    # --- checks ------------------------------------------------------------------------
    info = {'occurrences': root.allOccurrences.count, 'parameters': design.userParameters.count}
    try:
        props = root.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        tok = token.component.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass
        info['mass_g_rover'] = round((props.mass - tok) * 1000.0, 1)
    except Exception as e:
        info['mass_error'] = str(e)
    bb = root.boundingBox
    info['bbox_mm'] = [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1), round((bb.maxPoint.y - bb.minPoint.y) * 10, 1),
                       round((bb.maxPoint.z - bb.minPoint.z) * 10, 1)]
    expected = {frozenset(('Servo_MG946R', 'Intake_Door_PLA')), frozenset(('N20_Gearmotor_150to1', 'Wheel_44mm'))}
    clashes = []
    try:
        bodies = adsk.core.ObjectCollection.create()
        for occ in root.allOccurrences:
            if occ.component.name in ('Victim_Token_40mm', 'Wiring_Harness'):
                continue
            for b in occ.bRepBodies:
                bodies.add(b)
        inp = design.createInterferenceInput(bodies)
        inp.areCoincidentFacesIncluded = False
        res = design.analyzeInterference(inp)
        for r in res:
            def owner(e):
                try:
                    if e.assemblyContext:
                        return e.assemblyContext.component.name
                except Exception:
                    pass
                try:
                    return e.parentComponent.name
                except Exception:
                    return '?'
            pair = (owner(r.entityOne), owner(r.entityTwo))
            vol = r.interferenceBody.volume * 1000.0 if r.interferenceBody else 0.0
            if frozenset(pair) in expected:
                continue
            clashes.append('%s <-> %s (%.1f mm3)' % (pair[0], pair[1], vol))
    except Exception as e:
        clashes.append('interference check failed: %s' % e)
    info['unexpected_clashes'] = clashes
    if eng_report:
        info['engraving_notes'] = eng_report

    # --- outputs -------------------------------------------------------------------------
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        vp = app.activeViewport
        try:
            vp.visualStyle = adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
        except Exception:
            pass

        try:
            root.isJointsFolderLightBulbOn = False          # no joint glyphs in the images
            root.isOriginFolderLightBulbOn = False
        except Exception:
            pass

        def shot(name, orient, door_deg=0.0):
            door_joint.jointMotion.rotationValue = math.radians(door_deg)
            camv = vp.camera
            camv.viewOrientation = orient
            camv.isFitView = True
            vp.camera = camv
            vp.refresh()
            adsk.doEvents()
            vp.saveAsImageFile(os.path.join(out_dir, name), 1800, 1200)

        V = adsk.core.ViewOrientations
        shot('render_iso_door_open.png', V.IsoTopRightViewOrientation, 60.0)
        shot('render_iso_door_closed.png', V.IsoTopRightViewOrientation, 0.0)
        shot('render_front.png', V.FrontViewOrientation, 60.0)
        shot('render_side.png', V.RightViewOrientation, 0.0)
        shot('render_top.png', V.TopViewOrientation, 60.0)
        shot('render_iso_rear.png', V.IsoTopLeftViewOrientation, 0.0)
        if detailed:
            upper = [o2 for o2 in root.occurrences if o2.component.name in
                     ('Upper_Deck_Acrylic', 'Camera_Mast_PLA', 'ESP32_CAM_OV2640', 'ESP32_S3_DevKitC', 'MPU6050_IMU',
                      'ESP32_Carrier_Perfboard', 'Power_Switch_Rocker',
                      'INMP441_Mic_L', 'INMP441_Mic_R', 'VL53L0X_Left', 'VL53L0X_Right')]
            for o2 in upper:
                o2.isLightBulbOn = False
            shot('render_wiring_lower_deck.png', V.IsoTopRightViewOrientation, 0.0)
            shot('render_wiring_top.png', V.TopViewOrientation, 0.0)
            for o2 in upper:
                o2.isLightBulbOn = True
        door_joint.jointMotion.rotationValue = math.radians(60.0)

        em = design.exportManager
        base = 'AEGIS_Rover_Detailed' if detailed else 'AEGIS_Rover'
        for fn, opt in ((base + '.f3d', em.createFusionArchiveExportOptions(os.path.join(out_dir, base + '.f3d'))),
                        (base + '.step', em.createSTEPExportOptions(os.path.join(out_dir, base + '.step')))):
            try:
                em.execute(opt)
            except Exception as e:
                info['export_error_' + fn] = str(e)

        info.update(cut_dxf_notes)          # cut DXFs were written before engraving (cut lines only)

        for occ_name in ('Intake_Shroud_PLA', 'Intake_Door_PLA', 'Motor_Hanger_PLA', 'Camera_Mast_PLA'):
            try:
                oo = [o2 for o2 in root.allOccurrences if o2.component.name == occ_name][0]
                opt = em.createSTLExportOptions(oo, os.path.join(out_dir, 'print_%s.stl' % occ_name))
                opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                em.execute(opt)
            except Exception as e:
                info['stl_error_' + occ_name] = str(e)

        with open(os.path.join(out_dir, 'build_report.json'), 'w') as f:
            json.dump(info, f, indent=2)

    vpc = app.activeViewport.camera
    vpc.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
    vpc.isFitView = True
    app.activeViewport.camera = vpc
    return info


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output')
        info = build(out)
        ui.messageBox('A.E.G.I.S. Mk2 built.\n\n' + json.dumps(info, indent=2) +
                      '\n\nRenders, STEP, DXF and STL files are in:\n' + out, 'TL101 A.E.G.I.S.')
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
