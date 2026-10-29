---
name: aegis-firmware
description: Build, test, flash or change the A.E.G.I.S. ESP32-S3 rover firmware (team_share/firmware/aegis_s3) or the ESP32-CAM firmware (team_share/firmware/aegis_cam). Use for any edit to .c/.h files, aegis_params.h, aegis_config.h, sdkconfig.defaults, Kconfig, pins, reflexes, the Wi-Fi protocol or the camera stream.
---

# A.E.G.I.S. firmware

## Layout
- `aegis_s3/components/aegis_core/`: the **portable C core**. It has no ESP-IDF calls: control, CBF speed limit, reflexes (`aegis_reflex.c`), mission logic, DSP (GCC-PHAT). The same code runs in the simulator (`simulation3d/core_shim.c`) and in the host unit tests. Keep it portable: no `esp_*` includes there.
- `aegis_s3/components/aegis_core/include/aegis_params.h`: every tunable number. It has door servo µs, CBF distance, reflex distances, the 300 ms link watchdog, the PCF8574 address 0x20, `CLIFF_ACTIVE_HIGH`, `CAPTURE_ACTIVE_LOW` and the battery divider (100k/20k). **Changing a value here is a design change: ask first.**
- `aegis_s3/main/aegis_config.h`: pin map. It must match `tools/wiring_steps.py` and `tools/wiring_guide.py`. If a pin changes, all three change, and the wiring PDF is regenerated. Ask first.
- `aegis_s3/main/`: FreeRTOS tasks.
  - `task_nav` runs on core 1 at 100 Hz.
  - `task_audio` handles the 48 kHz I2S mics and the 16 kHz listen clips.
  - `task_net` handles Wi-Fi/UDP. Port 4210 is laptop→rover, 4211 is rover→laptop.
  - `task_vision` and `app_main.c`.
  - Drivers live in `main/drivers/`.
- `aegis_cam/main/cam_main.c`: MJPEG `/stream` and `/capture`, plus the UDP beacon `{"type":"cam","url"}`. `CAM_FLIP` sets the image orientation.

## Wire protocol (do not change without updating `edge/aegis_link.py` and `edge/fake_rover.py` together)
- Laptop→rover messages:
  - `{"type":"cmd","v","w","door"}`, sent at 20 Hz by `KeepAlive`. If commands stop for 300 ms, the rover stops.
  - `{"type":"listen","dur"}`.
- Rover→laptop: telemetry JSON at 20 Hz, broadcast at 2 Hz until a laptop connects.
- Audio packets: `"AUD1"` followed by id, idx, total and n (uint16 LE), then int16 samples, then an `audio_meta` message.

## Commands
```bash
# host unit tests of the portable core (no ESP-IDF needed), currently 41/41
make -C team_share/firmware/aegis_s3/test/host

# full builds (ESP-IDF v5.3.2)
. ~/esp/idf_env.sh
cd team_share/firmware/aegis_s3 && idf.py build      # ESP32-S3, single-app-large partition, 4 MB flash
cd team_share/firmware/aegis_cam && idf.py build     # AI-Thinker ESP32-CAM
# flash: idf.py -p /dev/cu.usbserial-XXXX flash monitor   (ESP32-CAM: IO0 to GND while flashing)
```
Do not run `idf.py fullclean`, `menuconfig` or `set-target` unless asked. `sdkconfig` is generated from `sdkconfig.defaults`; edit the defaults.

## Rules for changes
1. After any change in `aegis_core`, run the host tests. Add a test in `test/host/test_core.c` for new behaviour, in the same style as the existing ones.
2. After any change, run `idf.py build` for the affected firmware. Report warnings: the build uses `-Werror` on the host. Check that the app partition still fits; the build prints the percentage used.
3. Safety reflexes always win over laptop commands. These are the edge stop, the link watchdog, the tilt cut-off and low-battery cut-off, and the CBF speed limit. Never weaken one to make a test pass. Ask instead.
4. If the reflex or control code changes, re-run the simulator comparison (skill `aegis-brain-sim`), because the simulator uses this same C core.
