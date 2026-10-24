import os

import openpyxl
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side

# Layout follows the course template (ItemList_GroupName_LabSessionDay_LabSessionSequence):
# yellow header, plain black grid, Total row at the bottom.
wb = openpyxl.Workbook()
ws = wb.active
ws.title = "Item List"

HEADER_FILL = PatternFill(start_color="FFFF00", end_color="FFFF00", fill_type="solid")
HEADER_FONT = Font(name="Calibri", size=11, bold=True)
FONT = Font(name="Calibri", size=11)
LINK_FONT = Font(name="Calibri", size=11, color="0563C1", underline="single")
side = Side(style="thin", color="000000")
grid = Border(left=side, right=side, top=side, bottom=side)
center = Alignment(horizontal="center", vertical="center", wrap_text=True)
left = Alignment(horizontal="left", vertical="center", wrap_text=True)

headers = ["S. No.", "Item Name", "Item Quantity Required",
           "Per Item Cost as mentioned on Robu.in or other indian sites (only if item is not available on Robu)",
           "Total Cost of Item",
           "Item Weblink on Robu.in (if not available on Robu, only then put link of item available on any other Indian Site)",
           "Remarks"]
for col, h in enumerate(headers, 1):
    c = ws.cell(row=1, column=col, value=h)
    c.fill, c.font, c.alignment, c.border = HEADER_FILL, HEADER_FONT, center, grid
ws.row_dimensions[1].height = 75

# Prices incl. GST as listed on 23/24-Oct-2026.
items_data = [
    (1, 'WeAct Studio ESP32-S3-DevKitC-1 Development Board', 1, 1099, 'https://robu.in/product/weact-studio-esp32-s3-devkitc-1-esp32-s3-wifi-bluetooth-compatible-ble-5-0-mesh-development-board-esp32-wireless-module-micropython/', 'Main controller: motors, sensors, safety, Wi-Fi link to laptop'),
    (2, 'ESP32-CAM Wi-Fi + Bluetooth Module with OV2640 2MP Camera', 1, 779, 'https://robu.in/product/esp32-cam-wifi-module-bluetooth-iot-module/', 'Camera for victim detection. Out of stock on Robu.in (24-Oct-2026); re-check before ordering.'),
    (3, 'ESP32-CAM-MB Micro USB Download Module', 1, 119, 'https://robu.in/product/esp32-cam-mb-micro-usb-shield-module-for-esp32-cam-development-board/', 'USB programmer for the ESP32-CAM'),
    (4, 'INMP441 MEMS High Precision Omnidirectional Microphone Module (I2S)', 2, 149, 'https://robu.in/product/inmp441-mems-high-precision-omnidirectional-microphone-module-i2s/', 'Two-microphone array for scream direction. Out of stock on Robu.in (24-Oct-2026); re-check before ordering.'),
    (5, 'N20 12V 200RPM Micro Metal Gear Motor With Encoder (GA12-N20, 150:1)', 4, 559, 'https://robu.in/product/n20-12v-200rpm-micro-metal-gear-motor-with-encoder/', 'Drive motors with encoders (4WD)'),
    (6, 'TB6612FNG Dual Motor Driver Module (1.2 A cont. / 3.2 A peak per channel)', 2, 149, 'https://robu.in/product/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/', 'Motor drivers, one channel per motor'),
    (7, 'GY-530 VL53L0X Time-of-Flight Laser Ranging Sensor (unsoldered)', 3, 209, 'https://robu.in/product/unsoldered-purple-gy-530-vl53l0x-time-of-flight-tof-laser-ranging-sensor-module/', 'Front, left and right obstacle distance'),
    (8, 'SmartElex MPU-6050 3-Axis Accelerometer and Gyro Sensor Module', 1, 444, 'https://robu.in/product/smartelex-mpu-6050-3-axis-accelerometer-and-gyro-sensor-module/', 'Gyro for heading and turn control'),
    (9, 'TowerPro MG946R Digital High Torque Metal Gear Servo Motor', 1, 459, 'https://robu.in/product/towerpro-mg946r-digital-high-torque-metal-gear-servo-motor/', 'Opens and closes the intake-bay door'),
    (10, 'Pro-Range ISR 18650 11.1V 2200mAh 10C 3S1P Li-Ion Battery Pack with DC Jack', 1, 1729, 'https://robu.in/product/pro-range-18650-li-ion-2200mah-3s-11-1v-10c-3s1p/', 'Main battery (3S, 10C)'),
    (11, 'Pro-Range Battery Charger 3S Li-Ion 12.6V with DC 5.5 x 2.5 mm Plug', 1, 549, 'https://robu.in/product/pro-range-battery-charger-3s-li-ion-12-6v-2a-with-dc-5-5mm-2-5mm-male-plug/', 'Charger for the battery pack'),
    (12, 'XL4015 5A 75W Adjustable DC-DC Step-Down Module', 2, 149, 'https://robu.in/product/dc-dc-xl4015-5a-75w-adjustable-step-down-module/', 'Two 5 V rails: logic and servo'),
    (13, '3PI miniQ 44mm N20 DC Gear Motor Wheel', 4, 39, 'https://robu.in/product/3pi-miniq-car-wheel-tyre-42mm-n20-dc-gear-motor-wheel/', 'Wheels for the N20 motors. Out of stock on Robu.in (24-Oct-2026); re-check before ordering.'),
    (14, 'Numakers PLA+ Filament, Pitch Black, 1.75 mm, 1 kg', 1, 780, 'https://robu.in/product/numakers-pla-filament-pitch-black-1-75-mm-1-kg/', '3D-printed intake bay, door, motor mounts, camera mast'),
    (15, 'Pro-Range 120PCS M3 Hexagon Copper Pillar Screw Kit', 1, 499, 'https://robu.in/product/pro-range-120pcs-m3-hexagon-copper-pillar-screw-kit/', 'Spacers, screws and nuts for the two decks'),
    (16, 'NEILL Clear Acrylic Sheet, 12 x 12 inch, 3 mm', 1, 235, 'https://www.amazon.in/dp/B0CP65DR3L', 'Laser-cut upper and lower decks (both fit on one sheet). Not sold on Robu.in'),
    (17, 'PCF8574T I2C I/O Extension Board', 1, 79, 'https://robu.in/product/pcf8574t-i2c-i-o-extension-board/', 'I/O expander for the IR sensors'),
    (18, 'TCRT5000 IR Tracking Sensor Module', 5, 49, 'https://robu.in/product/tcrt5000-ir-reflex-tracking-sensor-module/', 'Table-edge sensors (front-left, front-right, rear, two rear corners)'),
    (19, 'Infrared Obstacle Avoidance Sensor Module', 1, 89, 'https://robu.in/product/infrared-obstacle-avoidance-sensor-module/', 'Detects the victim inside the intake bay'),
]

for r, (n, name, qty, price, link, remark) in enumerate(items_data, start=2):
    vals = (n, name, qty, price, "=C%d*D%d" % (r, r), link, remark)
    for col, v in enumerate(vals, 1):
        c = ws.cell(row=r, column=col, value=v)
        c.font, c.border = FONT, grid
        c.alignment = left if col in (2, 6, 7) else center
    ws.cell(row=r, column=6).hyperlink = link
    ws.cell(row=r, column=6).font = LINK_FONT
    ws.row_dimensions[r].height = 36

total = len(items_data) + 2
for col in range(1, 8):
    ws.cell(row=total, column=col).border = grid
ws.cell(row=total, column=2, value="Total").font = HEADER_FONT
ws.cell(row=total, column=2).alignment = center
ws.cell(row=total, column=5, value="=SUM(E2:E%d)" % (total - 1)).font = HEADER_FONT
ws.cell(row=total, column=5).alignment = center

for col, w in zip("ABCDEFG", (6, 38, 11, 20, 11, 70, 40)):
    ws.column_dimensions[col].width = w
ws.page_setup.paperSize = ws.PAPERSIZE_A4
ws.page_setup.orientation = "landscape"
ws.page_setup.fitToWidth, ws.page_setup.fitToHeight = 1, 0
ws.sheet_properties.pageSetUpPr.fitToPage = True

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "to_submit", "ItemList_AEGIS_Thursday_1.xlsx")
wb.save(OUT)
print("Saved", os.path.abspath(OUT), "total Rs", sum(q * u for _, _, q, u, _, _ in items_data))
