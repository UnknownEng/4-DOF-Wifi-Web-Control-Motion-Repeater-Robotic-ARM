# RobotArm – 4-DOF Wi-Fi Web Control Motion Repeater (ESP32)

An ESP32 creates its own Wi-Fi hotspot and serves a phone-friendly web page that controls a
4-servo robot arm (Base, Shoulder, Elbow, Gripper). You can **record** a sequence of movements and
**play** it back in a loop with the original timing.

![Robot Arm](Robot%20Arm.png)

## Hardware

| Joint    | ESP32 GPIO | Default limits (deg) |
|----------|-----------:|----------------------|
| Base     | 27 | 0 – 180  |
| Shoulder | 26 | 15 – 165 |
| Elbow    | 25 | 15 – 165 |
| Gripper  | 33 | 30 – 150 |

* Power the servos from a **separate 5–6 V supply** (≥ 2 A for 4 SG90/MG90, more for MG996R).
  Connect the supply's GND to the ESP32's GND. Don't power servos from the ESP32's 5V/3V3 pin,
  because the voltage drops and the board resets (brown-out).
* **Calibrate the limits** in the `joints[]` table at the top of `Robot_Arm.ino` so no joint can
  hit its mechanical end stops or crush the gripper.
* At power-on the servos jump to the home angle (a hobby servo's position can't be read back).
  Leave the arm near its home pose before switching off.

## Software

1. Install **ESP32 board support** (Espressif `esp32` core 3.x).
2. Install libraries from the Library Manager: **ESP Async WebServer** and **Async TCP**
   (both by ESP32Async), and **ESP32Servo** (Kevin Harrington).
3. **Change the settings** at the top of the sketch: `AP_PASSWORD` (8–63 characters) and
   `CONTROL_PIN`.
4. Upload with board "ESP32 Dev Module".

The Arduino tools need the sketch in a folder with the same name (`Robot_Arm/Robot_Arm.ino`).
The Arduino IDE offers to create that folder for you. With the command line:
```bash
mkdir Robot_Arm && cp Robot_Arm.ino Robot_Arm/
arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir build Robot_Arm
```

## Usage

1. Join the Wi-Fi network `RobotArm-XXXX` (the last 4 characters of the board's MAC address).
2. Open `http://192.168.4.1` and enter the PIN.
3. Move the sliders. **Record ON** → move the arm → **Record OFF**. The recording is saved to flash
   and survives a reboot.
4. **Play ON**: the arm returns slowly to the pose it had when recording started, waits 2 s and
   repeats the recording until you press **Play OFF**, which stops it immediately.
5. **Home** moves all joints to their home angles.

All connected phones stay in sync. The same commands also work on the Serial monitor (115200 baud):

```
Status            Move,Base,90      Record,1 / Record,0
Play,1 / Play,0   Home,1
```

## Simulation (Wokwi)

`diagram.json`, `libraries.txt` and `wokwi.toml` describe an ESP32 with 4 servos for
[Wokwi](https://wokwi.com). Paste the sketch, `diagram.json` and `libraries.txt` into a new ESP32
project on wokwi.com, set `#define ENABLE_WIFI 0` and press Play. Then drive the arm by typing the
Serial commands above into the serial monitor.

* `ENABLE_WIFI 0` is a Serial-only test mode. The Wi-Fi hotspot does boot in Wokwi, but very slowly,
  and you can't open its web page from your own browser without Wokwi's paid private gateway.
  Test the web page on real hardware.
* `test/robot_arm.test.yaml` is an automated scenario for
  `wokwi-cli --scenario test/robot_arm.test.yaml .`. It needs a Wokwi API token and a build in
  `build/` compiled with `ENABLE_WIFI=0`.

## What was fixed (vs. the original sketch)

* Crash/race: the WebSocket callback now only queues commands, and `loop()` owns all state.
  Recording during playback can no longer corrupt memory.
* Playback is a non-blocking state machine, so **Play OFF** stops instantly.
* Strict input parsing and per-joint limits: garbage or out-of-range values can no longer slam
  servos to 0° or drive them past their end stops.
* Speed-limited motion for live control, homing and playback.
* Recording is capped (1000 steps), with pauses capped at 60 s, and stored in NVS flash.
* PIN authentication with lockout after 5 wrong attempts, a 4-client limit and a unique SSID per board.
* Locks are enforced on the board, not just in the page. State is broadcast to every client.
* The page works with both mouse and touch, adapts to the screen size and shows the connection status.
