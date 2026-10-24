"""
A.E.G.I.S. design calculations: mass, drive torque, electrical load per rail,
battery C-rate / runtime, and gripper strength (jaw arm + printed gear).

Run:  python3 aegis_design_calcs.py
Writes AEGIS_Design_Calculations.md next to this file.

Every input carries its source:
  [L] = Robu.in product listing (checked 2026-10-23)
  [D] = component datasheet value
  [G] = geometry from Build_AEGIS_Rover.py
  [A] = engineering assumption, stated so it can be challenged or measured
"""
import math
import os

G = 9.81

# -----------------------------------------------------------------------------
# 1. Mass budget
# -----------------------------------------------------------------------------
RHO_ACRYLIC = 1.19e-3  # g/mm^3 [D]
RHO_PLA = 1.24e-3      # g/mm^3 [D], printed solid (conservative)

plate_vol = 170 * 110 * 3 - 4 * math.pi * 1.6**2 * 3 - 30 * 12 * 3          # [G]
grip_base_vol = 81 * 60 * 5 - 41.3 * 20.3 * 5                               # [G]
jaw_vol = math.pi * 15**2 * 10 + 50 * 8 * 8 + 6 * 4 * 8 - math.pi * 1.6**2 * 10  # [G]

MASS = [  # (item, qty, grams each, source)
    ('Acrylic chassis plate 170x110x3', 2, plate_vol * RHO_ACRYLIC, 'G'),
    ('PLA gripper base', 1, grip_base_vol * RHO_PLA, 'G'),
    ('PLA jaw (hub + arm)', 2, jaw_vol * RHO_PLA, 'G'),
    ('N20 gearmotor with encoder', 4, 11.0, 'L'),
    ('44 mm wheel', 4, 9.0, 'A'),
    ('MG946R servo', 1, 65.0, 'L'),
    ('3S1P 18650 pack incl. BMS, wrap, leads', 1, 3 * 46.0 + 25.0, 'A'),
    ('ESP32-S3 DevKitC', 1, 10.0, 'A'),
    ('ESP32-CAM + OV2640', 1, 10.0, 'A'),
    ('TB6612FNG module', 2, 3.0, 'A'),
    ('XL4015 module with heatsink', 2, 22.0, 'A'),
    ('MPU-6050 / INMP441 x2 / VL53L0X x3', 1, 3 + 2 * 2 + 3 * 2, 'A'),
    ('M3 pillars (20 F-F + 10 M-F) per corner', 4, 4.5, 'A'),
    ('M3 screws, nuts', 1, 20.0, 'A'),
    ('Wiring harness, perfboard, connectors', 1, 60.0, 'A'),
]
CONTINGENCY = 0.15       # [A] unmodelled parts, glue, cable ties
PAYLOAD_G = 200.0        # [A] design payload held in the gripper

m_dry = sum(q * g for _, q, g, _ in MASS)
m_robot = m_dry * (1 + CONTINGENCY)
m_total = (m_robot + PAYLOAD_G) / 1000.0  # kg, used for all load cases

# -----------------------------------------------------------------------------
# 2. Drive: geometry, motor model, load cases
# -----------------------------------------------------------------------------
R_WHEEL = 0.044 / 2        # m [G]
WHEELBASE = 0.110          # m [G] front-rear axle distance L
TRACK = 0.132              # m [G] left-right wheel centre distance B
N_WHEELS = 4

# N20 12 V 200 rpm 150:1 [L]
V_RATED = 12.0
RPM_NL = 200.0
RATED_TORQUE_KGCM = 1.0      # continuous rating, the design limit
RATED_RPM = 160.0
STALL_TORQUE_KGCM_LISTED = 7.0
I_STALL = 1.0
I_NOLOAD_LISTED = 0.010
# [A] The listed 10 mA no-load current is below what N20 motors draw in
# practice; 60 mA is used so currents are not under-estimated. The listed
# 7 kg-cm stall torque is also optimistic for a 150:1 N20, so the current
# estimate is repeated with a stall torque derated to 40 % (2.8 kg-cm), which
# raises current per unit torque. Both bounds are reported.
I_NOLOAD = 0.060
STALL_DERATE = 0.40

KGCM = G / 100.0  # N*m per kg-cm

C_RR = 0.05        # [A] rolling resistance, rubber tyre on rough indoor floor / debris
MU_TRACTION = 0.8  # [A] rubber on concrete, peak longitudinal
MU_LATERAL = 0.6   # [A] scrubbing friction during skid-steer pivot
ACCEL = 0.5        # m/s^2 [A] design acceleration
INCLINE_DEG = 15.0 # [A] design ramp


def wheel_torque_straight(m, a, theta_deg):
    """Per-wheel output torque for straight-line driving (N*m).

    tau = r / N * (C_rr*m*g*cos(theta) + m*a + m*g*sin(theta))

    Notes on the formula in the hand-off: it used the friction coefficient mu
    as the resistive term. mu is the traction LIMIT, not a resistance; the
    resistance is rolling resistance C_rr. It also divided by a gearbox
    efficiency, but the listed torques are already at the gearbox output and
    the wheel is direct-drive, so no extra efficiency term applies.
    """
    th = math.radians(theta_deg)
    f = C_RR * m * G * math.cos(th) + m * a + m * G * math.sin(th)
    return f * R_WHEEL / N_WHEELS, f


def wheel_torque_pivot(m):
    """Per-wheel torque to pivot a 4-wheel skid-steer rover in place (N*m).

    Each wheel carries m*g/4 and scrubs sideways at mu_lat with lever arm L/2
    about the centre, so the resisting yaw moment is mu_lat*m*g*L/2. The four
    longitudinal wheel forces F act at B/2, giving 2*F*B. Hence
    F = mu_lat*m*g*L / (4*B).
    """
    f = MU_LATERAL * m * G * WHEELBASE / (4 * TRACK)
    return f * R_WHEEL, f


def motor_current(tau_nm, stall_kgcm):
    tau_s = stall_kgcm * KGCM
    return I_NOLOAD + (I_STALL - I_NOLOAD) * tau_nm / tau_s


def motor_speed_rpm(tau_nm, stall_kgcm):
    return RPM_NL * (1 - tau_nm / (stall_kgcm * KGCM))


tau_straight, f_straight = wheel_torque_straight(m_total, ACCEL, INCLINE_DEG)
tau_flat_cruise, _ = wheel_torque_straight(m_total, 0.0, 0.0)
tau_pivot, f_pivot = wheel_torque_pivot(m_total)
tau_rated = RATED_TORQUE_KGCM * KGCM
traction_per_wheel = MU_TRACTION * m_total * G / N_WHEELS
v_rated = RATED_RPM / 60 * 2 * math.pi * R_WHEEL
max_incline_traction = math.degrees(math.atan(MU_TRACTION))

# -----------------------------------------------------------------------------
# 3. Electrical budget
# -----------------------------------------------------------------------------
V_BATT_NOM = 11.1
V_BATT_MIN = 10.5   # [A] ~3.5 V/cell under load, before BMS cut-off
CAP_AH = 2.2        # [L]
USABLE = 0.80       # [A] usable fraction of capacity
PACK_C = 10.0       # [L] ISR 10C cells
ETA_BUCK = 0.85     # [A] XL4015 at these loads

# 5.0 V logic rail: (name, average A, peak A, source)
LOGIC = [
    ('ESP32-S3 (Wi-Fi active, 240 MHz dual core)', 0.120, 0.340, 'D: ESP32-S3 TX peak ~340 mA'),
    ('ESP32-CAM streaming OV2640', 0.180, 0.310, 'A: ESP32 TX peak + camera'),
    ('VL53L0X x3 (continuous ranging)', 3 * 0.019, 3 * 0.019, 'D: 19 mA active'),
    ('MPU-6050', 0.004, 0.004, 'D: 3.9 mA'),
    ('INMP441 x2', 2 * 0.0014, 2 * 0.0014, 'D: 1.4 mA'),
    ('N20 Hall encoders x4', 4 * 0.010, 4 * 0.010, 'A'),
    ('TB6612FNG logic x2', 2 * 0.002, 2 * 0.002, 'D'),
    ('Module power LEDs (~6)', 0.030, 0.030, 'A'),
]
logic_avg = sum(a for _, a, _, _ in LOGIC)
logic_peak = sum(p for _, _, p, _ in LOGIC)

# 5.0 V servo rail
SERVO_V = 5.0
SERVO_STALL_LISTED = 1.2   # A [L] MG946R listing
SERVO_STALL_DESIGN = 2.5   # A [A] design value: MG99x-class servos are commonly
                           # measured at 2-2.5 A stall, well above the listing.
SERVO_MOVE = 0.5           # A [A] moving jaws, no load
SERVO_HOLD = 0.15          # A [A] holding at the commanded contact angle


def batt_side(i_rail, v_rail, v_batt):
    return i_rail * v_rail / (v_batt * ETA_BUCK)


# Worst case: all four motors stalled at full duty, servo stalled, logic peak,
# battery at its loaded minimum voltage (highest current through the bucks).
i_motor_stall_at_vmax = I_STALL * 12.6 / V_RATED
worst = {
    'Motors 4 x stall': N_WHEELS * I_STALL,
    'Servo stall (via buck)': batt_side(SERVO_STALL_DESIGN, SERVO_V, V_BATT_MIN),
    'Logic peak (via buck)': batt_side(logic_peak, 5.0, V_BATT_MIN),
}
i_worst = sum(worst.values())

# Handoff figure for comparison
i_handoff = 4 * 0.45 + 1.2 + 0.25 + 0.1

# Mission profile, time fractions [A]
PROFILE = [
    # phase, fraction, per-motor torque N*m, motor duty, servo A
    ('Cruise on flat at 0.3 m/s', 0.45, tau_flat_cruise, 0.8, SERVO_HOLD),
    ('Pivot turns (acoustic homing)', 0.25, tau_pivot, 0.6, SERVO_HOLD),
    ('Ramp / obstacle climb 15 deg', 0.05, tau_straight, 1.0, SERVO_HOLD),
    ('Stopped: listening / scanning', 0.20, 0.0, 0.0, SERVO_HOLD),
    ('Gripping', 0.05, 0.0, 0.0, SERVO_MOVE),
]


def profile_current(stall_kgcm):
    total = 0.0
    rows = []
    for name, frac, tau, duty, servo_a in PROFILE:
        i_m = N_WHEELS * duty * (motor_current(tau, stall_kgcm) if duty > 0 else 0.0)
        i = i_m + batt_side(servo_a, SERVO_V, V_BATT_NOM) + batt_side(logic_avg, 5.0, V_BATT_NOM)
        rows.append((name, frac, i))
        total += frac * i
    return total, rows


i_avg_listed, rows_listed = profile_current(STALL_TORQUE_KGCM_LISTED)
i_avg_derated, rows_derated = profile_current(STALL_TORQUE_KGCM_LISTED * STALL_DERATE)
runtime_h_best = CAP_AH * USABLE / i_avg_listed
runtime_h_worst = CAP_AH * USABLE / i_avg_derated
runtime_min_stall = CAP_AH * USABLE / i_worst * 60

# -----------------------------------------------------------------------------
# 4. Gripper strength
# -----------------------------------------------------------------------------
SIGMA_Y_PLA = 40.0       # MPa [A] FDM PLA+, in-plane (XY) tensile; derated
                         # from typical 45-50 MPa filament datasheet values
SF_REQ = 2.5

# MG946R stall torque [L]: 10.5 kg-cm @ 4.8 V, 13 kg-cm @ 6 V -> linear at 5 V
servo_torque_kgcm = 10.5 + (13.0 - 10.5) * (SERVO_V - 4.8) / (6.0 - 4.8)
servo_torque = servo_torque_kgcm * KGCM

JAW_L = 50.0e-3          # pivot to fingertip [G]
HUB_R = 15.0e-3          # gear pitch radius = hub radius [G]
JAW_B = 8.0              # mm, section depth in pinch (bending) direction [G]
JAW_H = 8.0              # mm, section height [G]
GEAR_M = 2.0             # module, mm [G]
GEAR_Z = 15              # teeth [G]
GEAR_FACE = 10.0         # mm [G]
LEWIS_Y = 0.290          # Lewis form factor, 15 teeth, 20 deg [D, standard table]
PIN_D = 3.0              # mm, M3 steel pivot pin [G]
F_SPEC = 10.0            # N fingertip pinch, project requirement

assert abs(GEAR_M * GEAR_Z / 2 - HUB_R * 1000) < 1e-9, 'pitch radius must equal hub radius'


def jaw_root_stress(f_tip):
    # Bending at the hub edge; arm length from hub edge to tip
    m = f_tip * (JAW_L - HUB_R) * 1000.0            # N*mm
    z = JAW_H * JAW_B**2 / 6.0                       # mm^3
    return m / z


f_tip_stall = servo_torque / JAW_L
sigma_arm_spec = jaw_root_stress(F_SPEC)
sigma_arm_stall = jaw_root_stress(f_tip_stall)

f_gear = servo_torque / HUB_R                         # tangential tooth load
sigma_gear = f_gear / (GEAR_FACE * GEAR_M * LEWIS_Y)  # Lewis bending, MPa
f_pin = math.hypot(f_gear, f_tip_stall)               # worst-case pin reaction
bearing = f_pin / (PIN_D * GEAR_FACE)                 # MPa on PLA bore
torque_needed_spec = F_SPEC * JAW_L


# Same gear at the old design point (MG996R at 6 V, 11 kg-cm, r = 12 mm,
# module 2, 12 teeth, Y = 0.245, 10 mm face) to show why it was changed
old_gear_sigma = (11.0 * KGCM / 0.012) / (10.0 * 2.0 * 0.245)


# -----------------------------------------------------------------------------
# Report
# -----------------------------------------------------------------------------
def kgcm(nm):
    return nm / KGCM


def ok(flag):
    return 'PASS' if flag else '**FAIL**'


L = []
w = L.append
w('# A.E.G.I.S. Design Calculations')
w('')
w('Generated by `aegis_design_calcs.py`. Do not edit by hand. Change the inputs in the script and re-run it.')
w('')
w('Source tags: **[L]** Robu.in listing (23-Oct-2026), **[D]** datasheet, **[G]** Fusion model geometry, **[A]** stated assumption.')
w('')
w('## 1. Mass budget')
w('')
w('| Item | Qty | g each | g total | Src |')
w('|---|---:|---:|---:|---|')
for name, q, g, src in MASS:
    w('| {} | {} | {:.1f} | {:.1f} | {} |'.format(name, q, g, q * g, src))
w('| **Dry sum** | | | **{:.0f}** | |'.format(m_dry))
w('| Contingency {:.0f} % | | | {:.0f} | A |'.format(CONTINGENCY * 100, m_dry * CONTINGENCY))
w('| Design payload | | | {:.0f} | A |'.format(PAYLOAD_G))
w('| **Design mass m** | | | **{:.0f}** | |'.format(m_total * 1000))
w('')
w('## 2. Drive torque and traction')
w('')
w('Per-wheel output torque, straight line:')
w('')
w('$$\\tau_{wheel} = \\frac{r}{4}\\left(C_{rr}\\,m g\\cos\\theta + m a + m g\\sin\\theta\\right)$$')
w('')
w('Two corrections to the formula in the hand-off note:')
w('')
w('- It used the traction coefficient μ as a resistive force. μ is the grip limit, not a drag. The drag term is rolling resistance $C_{rr}$. μ is used below only to check for wheel slip.')
w('- It divided by a gearbox efficiency η. The listed motor torques are already measured at the gearbox output and the wheel is mounted directly on the shaft, so no extra η applies.')
w('')
w('Skid-steer pivot turn: each wheel scrubs sideways at $\\mu_{lat}\\,mg/4$ with lever arm L/2. The four drive forces act at B/2:')
w('')
w('$$F_{wheel} = \\frac{\\mu_{lat}\\, m g\\, L}{4B}, \\qquad \\tau_{wheel} = F_{wheel}\\, r$$')
w('')
w('| Input | Value | Src |')
w('|---|---|---|')
w('| Wheel radius r | {:.1f} mm | G |'.format(R_WHEEL * 1000))
w('| Wheelbase L / track B | {:.0f} / {:.0f} mm | G |'.format(WHEELBASE * 1000, TRACK * 1000))
w('| C_rr / μ traction / μ lateral | {} / {} / {} | A |'.format(C_RR, MU_TRACTION, MU_LATERAL))
w('| Design accel / incline | {} m/s² / {:.0f}° | A |'.format(ACCEL, INCLINE_DEG))
w('| N20 rated torque (continuous limit) | {:.1f} kg·cm = {:.4f} N·m @ {:.0f} rpm | L |'.format(RATED_TORQUE_KGCM, tau_rated, RATED_RPM))
w('| N20 stall: listed / derated | {:.1f} / {:.1f} kg·cm, {:.1f} A | L / A |'.format(STALL_TORQUE_KGCM_LISTED, STALL_TORQUE_KGCM_LISTED * STALL_DERATE, I_STALL))
w('')
w('| Load case | Total force | τ per wheel | % of rated | Check |')
w('|---|---:|---:|---:|---|')
w('| Cruise, flat, constant speed | {:.2f} N | {:.3f} kg·cm | {:.0f} % | {} |'.format(
    tau_flat_cruise * 4 / R_WHEEL, kgcm(tau_flat_cruise), 100 * tau_flat_cruise / tau_rated, ok(tau_flat_cruise < tau_rated)))
w('| Accelerate 0.5 m/s² up 15° ramp | {:.2f} N | {:.3f} kg·cm | {:.0f} % | {} |'.format(
    f_straight, kgcm(tau_straight), 100 * tau_straight / tau_rated, ok(tau_straight < tau_rated)))
w('| Pivot turn in place | {:.2f} N per wheel | {:.3f} kg·cm | {:.0f} % | {} |'.format(
    f_pivot, kgcm(tau_pivot), 100 * tau_pivot / tau_rated, ok(tau_pivot < tau_rated)))
w('')
w('- Traction available per wheel: μ·mg/4 = {:.2f} N. The worst drive force is {:.2f} N per wheel, so the wheels grip and do not slip ({}).'.format(
    traction_per_wheel, max(f_straight / 4, f_pivot), ok(max(f_straight / 4, f_pivot) < traction_per_wheel)))
w('- The steepest climbable slope is set by traction, not torque: atan(μ) = {:.0f}°.'.format(max_incline_traction))
w('- Speed at rated load: {:.2f} m/s. Pivot turning is the hardest drive case, at {:.1f}× the torque of the ramp case.'.format(
    v_rated, tau_pivot / tau_straight))
w('')
w('## 3. Electrical budget')
w('')
w('Power architecture: the 3S pack (10.5–12.6 V) feeds both TB6612FNG drivers directly (VM max 15 V [D]). Two XL4015 bucks make separate 5.0 V rails: **Rail A** for logic and **Rail B** for the servo. The servo gets its own rail so its stall-current dips cannot trip the ESP32 brown-out reset.')
w('')
w('### 3.1 Rail A, 5.0 V logic')
w('')
w('| Load | Avg A | Peak A | Src |')
w('|---|---:|---:|---|')
for name, a, p, src in LOGIC:
    w('| {} | {:.3f} | {:.3f} | {} |'.format(name, a, p, src))
w('| **Total** | **{:.3f}** | **{:.3f}** | |'.format(logic_avg, logic_peak))
w('')
w('XL4015 is rated 5 A. Rail A peaks at {:.0f} % of that.'.format(100 * logic_peak / 5.0))
w('')
w('### 3.2 Rail B, 5.0 V servo')
w('')
w('MG946R stall is listed at {:.1f} A [L]. The rail is sized for {:.1f} A [A], because MG99x-class servos commonly measure 2–2.5 A at stall. Measure the real value on a bench supply before trusting either number. The servo is run at 5.0 V, not 6 V, so its stall torque ({:.1f} kg·cm) keeps the printed gear above SF 2.5 (section 4).'.format(
    SERVO_STALL_LISTED, SERVO_STALL_DESIGN, servo_torque_kgcm))
w('')
w('### 3.3 Battery: worst case')
w('')
w('Everything stalled at once, battery at {:.1f} V loaded:'.format(V_BATT_MIN))
w('')
w('| Load | Battery-side A |')
w('|---|---:|')
for k, v in worst.items():
    w('| {} | {:.2f} |'.format(k, v))
w('| **Total** | **{:.2f}** |'.format(i_worst))
w('')
w('- C-rate: {:.2f} A / {:.1f} Ah = **{:.2f} C**, against the pack\'s {:.0f} C rating ({}).'.format(i_worst, CAP_AH, i_worst / CAP_AH, PACK_C, ok(i_worst / CAP_AH < PACK_C)))
w('- A 2C pack (the 999-rupee 2200 mAh INR option) has a {:.1f} A limit and would **fail** this case. That is why the BOM uses the 10C ISR pack.'.format(2 * CAP_AH))
w('- The pack\'s BMS continuous rating is not on the listing. It must be ≥ {:.0f} A. Check it on arrival.'.format(math.ceil(i_worst)))
w('- TB6612FNG channel: motor stall is {:.2f} A at 12.6 V, against 1.2 A continuous per channel ({}). This works only with **one motor per channel**, which is why the BOM has two drivers.'.format(
    i_motor_stall_at_vmax, ok(i_motor_stall_at_vmax <= 1.2)))
w('')
w('The hand-off note\'s figure was 4(0.45) + 1.2 + 0.25 + 0.1 = {:.2f} A ("1.52 C, far below 20 C"). It was wrong in four ways:'.format(i_handoff))
w('')
w('1. It assumed 0.45 A motor stall. The listing says 1 A.')
w('2. It assumed 1.2 A servo stall, the optimistic listing figure.')
w('3. It added 5 V and 6 V rail currents directly to battery current, without converting through the buck converters.')
w('4. It assumed the pack was rated 20 C. The pack it named is 2C.')
w('')
w('### 3.4 Runtime, mission profile')
w('')
w('Usable capacity: {:.1f} Ah × {:.0f} % = {:.2f} Ah.'.format(CAP_AH, USABLE * 100, CAP_AH * USABLE))
w('')
w('| Phase | Time share | Battery A (listed stall) | Battery A (derated stall) |')
w('|---|---:|---:|---:|')
for (name, frac, a1), (_, _, a2) in zip(rows_listed, rows_derated):
    w('| {} | {:.0f} % | {:.2f} | {:.2f} |'.format(name, frac * 100, a1, a2))
w('| **Time-weighted average** | | **{:.2f}** | **{:.2f}** |'.format(i_avg_listed, i_avg_derated))
w('')
w('- Runtime: **{:.1f} h** with the listed motor stall torque, **{:.1f} h** with it derated to 40 %.'.format(runtime_h_best, runtime_h_worst))
w('- Runtime with everything continuously stalled: {:.0f} min.'.format(runtime_min_stall))
w('- Electronics dominate the average draw. The motors are lightly loaded except when pivot-turning.')
w('')
w('## 4. Gripper strength (hand calculation, to be confirmed in Fusion Simulation)')
w('')
w('Mechanism: the servo spline drives the +Y jaw directly. The −Y jaw pivots on an M3 steel pin. The two jaw hubs mesh as a 1:1 spur pair (module {:.0f}, {} teeth, pitch radius {:.0f} mm, face {:.0f} mm), so each jaw sees the full servo torque.'.format(
    GEAR_M, GEAR_Z, HUB_R * 1000, GEAR_FACE))
w('')
w('Material: PLA+ printed flat (XY), allowable yield σ_y = {:.0f} MPa [A]. Required SF ≥ {}.'.format(SIGMA_Y_PLA, SF_REQ))
w('')
w('Servo stall torque at {:.1f} V: {:.2f} kg·cm = {:.3f} N·m (interpolated from listed 10.5 kg·cm @ 4.8 V and 13 kg·cm @ 6 V).'.format(
    SERVO_V, servo_torque_kgcm, servo_torque))
w('')
w('| Check | Load | Stress | SF | Result |')
w('|---|---:|---:|---:|---|')
w('| Jaw arm bending at hub, spec pinch | {:.1f} N tip | {:.1f} MPa | {:.1f} | {} |'.format(
    F_SPEC, sigma_arm_spec, SIGMA_Y_PLA / sigma_arm_spec, ok(SIGMA_Y_PLA / sigma_arm_spec >= SF_REQ)))
w('| Jaw arm bending at hub, servo stall | {:.1f} N tip | {:.1f} MPa | {:.1f} | {} |'.format(
    f_tip_stall, sigma_arm_stall, SIGMA_Y_PLA / sigma_arm_stall, ok(SIGMA_Y_PLA / sigma_arm_stall >= SF_REQ)))
w('| Gear tooth (Lewis), servo stall | {:.1f} N tangential | {:.1f} MPa | {:.2f} | {} |'.format(
    f_gear, sigma_gear, SIGMA_Y_PLA / sigma_gear, ok(SIGMA_Y_PLA / sigma_gear >= SF_REQ)))
w('| Pivot bore bearing on PLA | {:.1f} N | {:.1f} MPa | {:.1f} | {} |'.format(
    f_pin, bearing, SIGMA_Y_PLA / bearing, ok(SIGMA_Y_PLA / bearing >= SF_REQ)))
w('')
w('- Arm bending: σ = M/Z, with M = F·(L − r_hub) and Z = h·b²/6 (b = {:.0f} mm in the bending direction, h = {:.0f} mm).'.format(JAW_B, JAW_H))
w('- The **gear teeth are the critical feature**, not the jaw arm. A first layout used a 12-tooth, 12 mm-radius gear with the servo at 6 V. It gave σ = {:.1f} MPa, SF = {:.2f}, a **fail**. The fix was to grow the pitch radius to 15 mm (15 teeth) and run the servo at 5.0 V.'.format(
    old_gear_sigma, SIGMA_Y_PLA / old_gear_sigma))
w('- 15 teeth at 20° pressure angle is just under the 17-tooth undercut limit. Generate the teeth with Fusion\'s SpurGear add-in at 25° pressure angle, or with +0.1 profile shift.')
w('- The torque needed for the 10 N spec pinch is {:.2f} N·m, {:.0f} % of stall. **Firmware must stop commanding the servo at the contact angle.** Commanding past contact stalls the servo continuously and heats it.'.format(
    torque_needed_spec, 100 * torque_needed_spec / servo_torque))
w('')
w('## 5. Fusion Simulation setup for the jaw (FEA)')
w('')
w('Run this after `Build_AEGIS_Rover.py` and after cutting the real gear teeth.')
w('')
w('1. **Simulation workspace → New Study → Static Stress.** Simplify: keep only the `Gripper_Jaw_PLA` body; remove or suppress everything else.')
w('2. **Material:** Study Materials → PLA. If the library has no PLA, create a custom material: E = 3.5 GPa, ν = 0.36, ρ = 1.24 g/cm³, **yield = {:.0f} MPa**, ultimate = 45 MPa.'.format(SIGMA_Y_PLA))
w('3. **Constraints:**')
w('   - Pin constraint on the 3.2 mm pivot bore (radial + axial fixed, tangential free).')
w('   - Fixed constraint on the flank of one gear tooth at the pitch line. This reacts the drive torque the way the meshing tooth does.')
w('4. **Loads:**')
w('   - Case A: {:.0f} N on the fingertip face, normal to it, pinching inward.'.format(F_SPEC))
w('   - Case B: {:.1f} N, the servo stall case.'.format(f_tip_stall))
w('5. **Mesh:** global 5 % of model size. Local mesh control 0.5 mm on the hub-to-arm junction and on the loaded tooth root. Enable adaptive refinement (3 passes).')
w('6. **Pass criteria:**')
w('   - Minimum safety factor (von Mises vs yield) ≥ {} in both cases.'.format(SF_REQ))
w('   - Peak stress in case B should be within about 30 % of the hand calculation: arm {:.1f} MPa, tooth root {:.1f} MPa.'.format(sigma_arm_stall, sigma_gear))
w('   - If FEA is much higher at the hub-arm corner, add a 2 mm fillet there.')
w('7. **Record for the report:** the safety-factor plot, the von Mises plot, the displacement at the fingertip, and the mesh convergence plot.')
w('')

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'AEGIS_Design_Calculations.md')
with open(out, 'w') as f:
    f.write('\n'.join(L))

print('design mass {:.0f} g'.format(m_total * 1000))
print('tau: cruise {:.3f}, ramp {:.3f}, pivot {:.3f} kg-cm (rated {:.1f})'.format(
    kgcm(tau_flat_cruise), kgcm(tau_straight), kgcm(tau_pivot), RATED_TORQUE_KGCM))
print('worst battery {:.2f} A = {:.2f} C'.format(i_worst, i_worst / CAP_AH))
print('avg {:.2f}-{:.2f} A, runtime {:.1f}-{:.1f} h'.format(i_avg_listed, i_avg_derated, runtime_h_worst, runtime_h_best))
print('arm SF spec {:.1f} stall {:.1f}, gear SF {:.2f}, bearing SF {:.1f}'.format(
    SIGMA_Y_PLA / sigma_arm_spec, SIGMA_Y_PLA / sigma_arm_stall, SIGMA_Y_PLA / sigma_gear, SIGMA_Y_PLA / bearing))
print('wrote', out)
