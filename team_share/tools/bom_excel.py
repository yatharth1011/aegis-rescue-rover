import os

import openpyxl
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter

wb = openpyxl.Workbook()
ws = wb.active
ws.title = "Item List"

# Ensure grid lines are visible
ws.views.sheetView[0].showGridLines = True

# Colors
HEADER_FILL = PatternFill(start_color="1E3A8A", end_color="1E3A8A", fill_type="solid") # Dark Blue
HEADER_FONT = Font(name="Calibri", size=11, bold=True, color="FFFFFF")
TOTAL_FILL = PatternFill(start_color="DBEAFE", end_color="DBEAFE", fill_type="solid") # Light Blue
TOTAL_FONT = Font(name="Calibri", size=11, bold=True, color="1E3A8A")
REGULAR_FONT = Font(name="Calibri", size=10)
BOLD_FONT = Font(name="Calibri", size=10, bold=True)
LINK_FONT = Font(name="Calibri", size=10, color="0044CC", underline="single")

thin_border = Border(
    left=Side(style='thin', color='D1D5DB'),
    right=Side(style='thin', color='D1D5DB'),
    top=Side(style='thin', color='D1D5DB'),
    bottom=Side(style='thin', color='D1D5DB')
)

double_bottom_border = Border(
    left=Side(style='thin', color='D1D5DB'),
    right=Side(style='thin', color='D1D5DB'),
    top=Side(style='thin', color='1E3A8A'),
    bottom=Side(style='double', color='1E3A8A')
)

headers = [
    "S.\nNo.",
    "Item Name",
    "Item Quantity\nRequired",
    "Per Item Cost as mentioned\non Robu.in or other indian\nsites (only if item is not\navailable on Robu)",
    "Total Cost of\nItem",
    "Item Weblink on Robu.in (if not available on Robu,\nonly then put link of item available on any other\nIndian Site)",
    "Remarks"
]

ws.row_dimensions[1].height = 48
for col_num, h in enumerate(headers, 1):
    cell = ws.cell(row=1, column=col_num, value=h)
    cell.fill = HEADER_FILL
    cell.font = HEADER_FONT
    cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
    cell.border = thin_border

# Every link below was opened on robu.in on 2026-10-23; price is the listed
# price incl. GST on that date. "OUT OF STOCK" items must be re-checked before
# ordering (the remark names the in-stock fallback where one exists).
items_data = [
    (1, "WeAct Studio ESP32-S3-DevKitC-1 Development Board", 1, 1099,
     "https://robu.in/product/weact-studio-esp32-s3-devkitc-1-esp32-s3-wifi-bluetooth-compatible-ble-5-0-mesh-development-board-esp32-wireless-module-micropython/",
     "Main controller (dual-core LX7, 240 MHz): FreeRTOS audio DSP on core 1, navigation on core 0. In stock. Confirm the module variant (N8R8 / N16R8) on the shield can before soldering."),
    (2, "ESP32-CAM Wi-Fi + Bluetooth Module with OV2640 2MP Camera", 1, 779,
     "https://robu.in/product/esp32-cam-wifi-module-bluetooth-iot-module/",
     "Vision node: target detection and docking trigger, reports to the S3 over UART. OUT OF STOCK on 23-Oct-2026."),
    (3, "ESP32-CAM-MB Micro USB Download Module", 1, 119,
     "https://robu.in/product/esp32-cam-mb-micro-usb-shield-module-for-esp32-cam-development-board/",
     "USB programmer for the ESP32-CAM (it has no USB port of its own; missing from the previous BOM). In stock."),
    (4, "INMP441 MEMS High Precision Omnidirectional Microphone Module (I2S)", 2, 149,
     "https://robu.in/product/inmp441-mems-high-precision-omnidirectional-microphone-module-i2s/",
     "Two-mic TDoA array, 100 mm baseline, one shared I2S bus (L/R select pins). OUT OF STOCK on 23-Oct-2026."),
    (5, "N20 12V 200RPM Micro Metal Gear Motor With Encoder (GA12-N20, 150:1)", 4, 559,
     "https://robu.in/product/n20-12v-200rpm-micro-metal-gear-motor-with-encoder/",
     "Drive motors. Listing: rated 1 kg-cm @160 rpm, stall 1 A. 200 rpm chosen over 300 rpm (out of stock) for more torque; 0.37 m/s at rated load with 44 mm wheels. In stock."),
    (6, "TB6612FNG Dual Motor Driver Module (1.2 A cont. / 3.2 A peak per channel)", 2, 149,
     "https://robu.in/product/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/",
     "Two drivers = one channel per motor, so each wheel has its own PID loop. Motor stall 1 A < 1.2 A channel rating. One driver for four motors (previous BOM) would put 2 A on a 1.2 A channel. In stock."),
    (7, "GY-530 VL53L0X Time-of-Flight Laser Ranging Sensor (unsoldered)", 3, 209,
     "https://robu.in/product/unsoldered-purple-gy-530-vl53l0x-time-of-flight-tof-laser-ranging-sensor-module/",
     "Front / left / right ranging for the policy state and the 10 cm safety shield. Three needed (previous BOM had two). XSHUT pins used to give each a unique I2C address. In stock."),
    (8, "SmartElex MPU-6050 3-Axis Accelerometer and Gyro Sensor Module", 1, 444,
     "https://robu.in/product/smartelex-mpu-6050-3-axis-accelerometer-and-gyro-sensor-module/",
     "Yaw rate for odometry fusion and turn control. In stock."),
    (9, "TowerPro MG946R Digital High Torque Metal Gear Servo Motor", 1, 459,
     "https://robu.in/product/towerpro-mg946r-digital-high-torque-metal-gear-servo-motor/",
     "Intake-door actuator: the spline drives the door hinge directly (inverted through the lower deck). Metal gears hold the door shut against a captured object. Run at 5.0 V. In stock."),
    (10, "Pro-Range ISR 18650 11.1V 2200mAh 10C 3S1P Li-Ion Battery Pack with DC Jack", 1, 1729,
     "https://robu.in/product/pro-range-18650-li-ion-2200mah-3s-11-1v-10c-3s1p/",
     "10C (22 A) cells. Worst-case draw is 5.8 A (2.6C), which exceeds the 4.4 A limit of the 2C packs. Check the BMS continuous rating on arrival. In stock."),
    (11, "Pro-Range Battery Charger 3S Li-Ion 12.6V with DC 5.5 x 2.5 mm Plug", 1, 549,
     "https://robu.in/product/pro-range-battery-charger-3s-li-ion-12-6v-2a-with-dc-5-5mm-2-5mm-male-plug/",
     "CC/CV charger matching the pack's DC jack. In stock."),
    (12, "XL4015 5A 75W Adjustable DC-DC Step-Down Module", 2, 149,
     "https://robu.in/product/dc-dc-xl4015-5a-75w-adjustable-step-down-module/",
     "Rail A 5.0 V logic (0.8 A peak). Rail B 5.0 V servo (2.5 A design stall). Separate rails keep servo stall dips off the ESP32 brown-out detector. LM2596 modules were out of stock and rated too low for servo stall. In stock."),
    (13, "3PI miniQ 44mm N20 DC Gear Motor Wheel", 4, 39,
     "https://robu.in/product/3pi-miniq-car-wheel-tyre-42mm-n20-dc-gear-motor-wheel/",
     "Fits the N20 3 mm D-shaft. OUT OF STOCK on 23-Oct-2026. In-stock fallback: 34 mm N20 wheel, Rs 59 (https://robu.in/product/34mm-mini-car-n20-motor-wheel-rubber-small-wheel/). Set wheel_D = 34 in the Fusion parameters if used."),
    (14, "Numakers PLA+ Filament, Pitch Black, 1.75 mm, 1 kg", 1, 780,
     "https://robu.in/product/numakers-pla-filament-pitch-black-1-75-mm-1-kg/",
     "Intake shroud and door, motor hangers, camera mast (about 150 g of filament in total). In stock."),
    (15, "Pro-Range 120PCS M3 Hexagon Copper Pillar Screw Kit", 1, 499,
     "https://robu.in/product/pro-range-120pcs-m3-hexagon-copper-pillar-screw-kit/",
     "30 mm deck spacing = 20 mm F-F pillar + 10+6 mm M-F pillar (both in kit), plus M3 screws and nuts. In stock."),
    (16, "Cast Acrylic Sheet, 3 mm, 300 x 300 mm", 1, 400,
     "NOT STOCKED ON ROBU.IN - team to add an Indian vendor link before submission",
     "Lower deck 255 x 120 mm and upper deck 210 x 120 mm nest on one 300 x 300 sheet. Price is an ESTIMATE until a vendor link is added. Use Tinkering Lab stock if available."),
    (17, "PCF8574T I2C I/O Extension Board", 1, 79,
     "https://robu.in/product/pcf8574t-i2c-i-o-extension-board/",
     "Reads the four IR sensors over the existing I2C bus (the ESP32-S3 has no free GPIOs left). In stock."),
    (18, "TCRT5000 IR Tracking Sensor Module", 5, 49,
     "https://robu.in/product/tcrt5000-ir-reflex-tracking-sensor-module/",
     "Table-edge (cliff) sensors pointing down: front-left, front-right, rear, and both rear corners (added after the 3D simulation showed rear-corner falls). The rover stops and moves away before a wheel leaves the table. In stock."),
    (19, "Infrared Obstacle Avoidance Sensor Module", 1, 89,
     "https://robu.in/product/infrared-obstacle-avoidance-sensor-module/",
     "On the back wall of the intake bay: confirms the victim token is inside before the door closes and again before returning. In stock."),
]
start_row = 2
for idx, item in enumerate(items_data):
    r = start_row + idx
    ws.row_dimensions[r].height = 32
    
    s_no, name, qty, unit_price, link, remarks = item
    
    c1 = ws.cell(row=r, column=1, value=s_no)
    c1.alignment = Alignment(horizontal="center", vertical="center")
    c1.font = REGULAR_FONT
    c1.border = thin_border
    
    c2 = ws.cell(row=r, column=2, value=name)
    c2.alignment = Alignment(horizontal="left", vertical="center", wrap_text=True)
    c2.font = REGULAR_FONT
    c2.border = thin_border
    
    c3 = ws.cell(row=r, column=3, value=qty)
    c3.alignment = Alignment(horizontal="center", vertical="center")
    c3.font = REGULAR_FONT
    c3.border = thin_border
    
    c4 = ws.cell(row=r, column=4, value=unit_price)
    c4.alignment = Alignment(horizontal="right", vertical="center")
    c4.number_format = '₹ #,##0'
    c4.font = REGULAR_FONT
    c4.border = thin_border
    
    # Formula for Total: Quantity * Unit Price
    c5 = ws.cell(row=r, column=5, value=f"=C{r}*D{r}")
    c5.alignment = Alignment(horizontal="right", vertical="center")
    c5.number_format = '₹ #,##0'
    c5.font = BOLD_FONT
    c5.border = thin_border
    
    c6 = ws.cell(row=r, column=6, value=link)
    c6.alignment = Alignment(horizontal="left", vertical="center")
    c6.font = LINK_FONT
    if link.startswith("http"):
        c6.hyperlink = link
    else:
        c6.font = Font(name="Calibri", size=10, bold=True, color="B91C1C")
    c6.border = thin_border
    
    c7 = ws.cell(row=r, column=7, value=remarks)
    c7.alignment = Alignment(horizontal="left", vertical="center", wrap_text=True)
    c7.font = REGULAR_FONT
    c7.border = thin_border

# Total Row
total_row = start_row + len(items_data)
ws.row_dimensions[total_row].height = 28

c_tot_label = ws.cell(row=total_row, column=1, value="Total")
ws.merge_cells(start_row=total_row, start_column=1, end_row=total_row, end_column=4)
c_tot_label.alignment = Alignment(horizontal="right", vertical="center")
c_tot_label.font = TOTAL_FONT
c_tot_label.fill = TOTAL_FILL
c_tot_label.border = double_bottom_border

for col_idx in range(2, 5):
    cell = ws.cell(row=total_row, column=col_idx)
    cell.border = double_bottom_border
    cell.fill = TOTAL_FILL

c_tot_val = ws.cell(row=total_row, column=5, value=f"=SUM(E{start_row}:E{total_row-1})")
c_tot_val.alignment = Alignment(horizontal="right", vertical="center")
c_tot_val.number_format = '₹ #,##0'
c_tot_val.font = TOTAL_FONT
c_tot_val.fill = TOTAL_FILL
c_tot_val.border = double_bottom_border

c_tot_empty1 = ws.cell(row=total_row, column=6, value="")
c_tot_empty1.border = double_bottom_border
c_tot_empty1.fill = TOTAL_FILL

c_tot_empty2 = ws.cell(row=total_row, column=7, value=f"Budget Ceiling: ₹15,000 | Balance Reserve: ₹{15000 - sum(q * u for _, _, q, u, _, _ in items_data):,}")
c_tot_empty2.alignment = Alignment(horizontal="left", vertical="center")
c_tot_empty2.font = TOTAL_FONT
c_tot_empty2.border = double_bottom_border
c_tot_empty2.fill = TOTAL_FILL

# Column widths
ws.column_dimensions['A'].width = 8
ws.column_dimensions['B'].width = 38
ws.column_dimensions['C'].width = 14
ws.column_dimensions['D'].width = 24
ws.column_dimensions['E'].width = 18
ws.column_dimensions['F'].width = 46
ws.column_dimensions['G'].width = 42

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "to_submit", "ItemList_AEGIS_Thursday_1.xlsx")
wb.save(OUT)
print("Saved", os.path.abspath(OUT))
