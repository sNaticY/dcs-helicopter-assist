# DCS Helicopter Assist

Lightweight helicopter assist for DCS. It reads live flight telemetry from DCS via Export.lua (UDP), applies stabilization and assist logic, and writes control outputs to a vJoy virtual joystick (X/Y for cyclic, RZ for rudder).

This document covers:
- Prerequisites
- Get DCS helicopter assist (GitHub Releases or run from source)
- vJoy setup
- DCS Export.lua setup
- How to run and use (hotkeys, modes)
- How to modify config (config.json)

---

## 1) Prerequisites

- Windows 10/11
- DCS World (stable or open beta)
- vJoy driver installed and at least one device enabled
  - Download: http://vjoystick.sourceforge.net/site/

If you run from source (instead of the EXE):
- Python 3.9+ (64-bit recommended)
- Python packages: numpy, pyvjoy, keyboard

Install packages:
```
py -m pip install numpy pyvjoy keyboard
```

Note: pyvjoy may need Administrator privileges to access the vJoy driver.

---

## 2) Get DCS Helicopter Assist

Option A — Download prebuilt (recommended):
1) Go to the GitHub Releases page:
   - https://github.com/sNaticY/dcs-helicopter-assist/releases
2) Download the latest DCSHelicopterAssist.zip for Windows.
3) Extract it to a folder you trust (e.g., C:\Games\DCSHelicotperAssist).
4) Ensure config.json is in the same folder as the EXE so you can edit it.
5) Continue to vJoy setup and DCS Export.lua setup below.

Tips:
- First run may trigger SmartScreen; choose “More info” → “Run anyway”.
- If vJoy output doesn’t move, try running the EXE “As Administrator”.

Option B — Run from source (Python):
1) Clone or download this repository into a folder (e.g., C:\Games\DCSHelicotperAssist).
2) Install Python requirements:
   ```
   py -m pip install numpy pyvjoy keyboard
   ```
3) Continue to vJoy setup and DCS Export.lua setup below.
4) Run:
   ```
   py helicopter_assist.py
   ```

---

## 3) vJoy setup

Open “vJoyConf” (vJoy Configuration) and configure a device:
- Enable Device ID: choose one ID (default in config.json is 3)
- Axes: enable X, Y, Rx (others optional/unused)
- Buttons: not required
- POV: none required
- Apply/Save

In DCS Controls, bind the vJoy device axes:
- Helicopter Pitch → vJoy Axis Y
- Helicopter Roll → vJoy Axis X
- Helicopter Rudder → vJoy Axis RX

Recommendations:
- Unbind your physical joystick axes directly from DCS to avoid double control. The app reads your physical joystick and outputs to vJoy; DCS should only read vJoy.
- Optional: Bind your physical joystick axes with modifiers `TOGGLE_PAUSE_HOTKEY` so you can temporarily by pass the assist and control directly with physical joystick
- Do not invert axes in DCS. The app already writes Y as inverted when sending to vJoy.

You can change the vJoy device ID in config.json (see section 7).

---

## 4) DCS Export.lua setup

This project includes Export/Export.lua which sends telemetry to the app via UDP.

1) Locate your DCS Saved Games Scripts folder:
- Stable: %USERPROFILE%\Saved Games\DCS\Scripts
- Open Beta: %USERPROFILE%\Saved Games\DCS.openbeta\Scripts

2) Backup your existing Export.lua if present.

3) Copy project file:
- From: c:\Games\HelicopterAssist\Export\Export.lua
- To:   %SavedGames%\DCS...\Scripts\Export.lua

Export.lua sends JSON lines at 50 Hz to 127.0.0.1:28777 by default. If you run the app on another machine, change the target in Export.lua and config.json, and allow UDP in the firewall.

What is exported:
- Linear velocity (Vx,Vy,Vz) in world coordinates (m/s)
- Linear acceleration (Ax,Ay,Az) in world coordinates (m/s²)
- Pitch, Roll, Yaw (rad)
- PitchRate, RollRate, YawRate (rad/s)
- World position (PosX, PosY, PosZ) in meters (x=East, y=Up, z=North)

The Python side parses these in dcs_telemetry.DcsTelemetry.

---

## 5) Running and usage

Running the EXE:
- Double-click the HelicopterAssist.exe (place config.json next to it).
- Start a DCS mission; Export.lua begins sending telemetry automatically.

Running from source:
```
py helicopter_assist.py
```

If vJoy doesn’t move:
- Confirm vJoy device ID matches config.json
- Confirm Export.lua is in Saved Games\DCS...\Scripts and a mission is running
- Try running as Administrator

Hotkeys (defaults; configurable in config.json):
- Toggle Rudder Assist: F8
- Toggle Cyclic Assist Mode: F9
  - Each press cycles: OFF → ON → HOVERING → OFF …
  - OFF: vJoy outputs pass through your manual inputs (after smoothing/curves)
  - ON: assist blends with your inputs; stabilization helps rate/attitude
  - HOVERING: adds stronger hold to help steady hover
- Pause (hold): Left Ctrl
  - While held, outputs are blocked
  - On release, assist modules reset to avoid bumps

Sounds:
- On: short high beep
- Hover: double mid beep
- Off: short low beep

Manual input:
- Your joystick inputs are smoothed and shaped (expo) before assist and vJoy output.

---

## 6) Updating

- Prebuilt EXE: download the new ZIP from Releases, extract over the old folder (or to a new folder). Keep your config.json.
- From source: pull latest changes (git pull) or replace files; keep your config.json.

---

## 7) Config (config.json)

Config is a plain JSON file at the project root:
- c:\Games\HelicopterAssist\config.json

Default content:
```
{
  "UDP_HOST": "127.0.0.1",
  "UDP_PORT": 28777,
  "VJOY_DEVICE_ID": 3,
  "TOGGLE_RUDDER_HOTKEY": "f8",
  "TOGGLE_CYCLIC_HOTKEY": "f9",
  "TOGGLE_PAUSE_HOTKEY": "left ctrl",
  "EMA_ALPHA": 0.25
}
```

Fields:
- UDP_HOST / UDP_PORT: where the app listens for telemetry from Export.lua
- VJOY_DEVICE_ID: vJoy device index as configured in vJoyConf
- TOGGLE_*_HOTKEY: keyboard hotkeys (see “How to use”)
- EMA_ALPHA: smoothing factor used in filters

How to modify:
- Edit config.json in a text editor
- Restart the app to apply changes

---

## 8) Project structure (key files)

- helicopter_assist.py: entry point; telemetry, input processing, assist modules, vJoy output
- dcs_telemetry.py: UDP receiver for DCS Export.lua JSON lines
- motion_state.py: transforms world data to body-frame velocities/accelerations
- cyclic_helper.py: cyclic assist logic
- rudder_helper.py: rudder assist logic
- input_processor.py: manual input smoothing, expo curves, tiny output dither
- utils.py: helpers (EMA, shaping, transforms, vJoy normalization)
- Export/Export.lua: DCS-side telemetry exporter

---

## 9) Troubleshooting

- vJoy not moving in DCS:
  - Verify vJoy device ID matches config.json
  - Bind vJoy axes in DCS controls; unbind physical device axes
  - Run app as Administrator (pyvjoy sometimes needs it)

- No telemetry / “frozen” values:
  - Confirm Export.lua is in Saved Games\DCS...\Scripts
  - Start a mission to activate export
  - Check UDP port and firewall rules

- Inverted axis:
  - Y axis is inverted once in the app; don’t invert again in DCS

- Minor oscillations:
  - Toggle out of HOVERING mode (F9), stabilize, then re-enter
  - PID tuning lives in cyclic_helper.py and rudder_helper.py

---

Enjoy flying!