# 0114 First Successful Kick — Project Overview

This project has been reorganized into a modular structure. The descriptions below assume the **project root** (`...\0114`). The main entry point is `src/main.py`.

---

## 1. Project Structure (Current)

- `src/`: main program and modules
  - `main.py`: GUI entry (vision, NRF, Monitor / Simulator).
  - `vision/`
    - `imageprocess.py`: vision processing and localization (ball, robots, angles).
    - `vision_ui.py`: UI for calibration / HSV / ColorMask.
  - `strategies/`
    - `challenge3_forward.py`: attack strategy (Challenge3).
    - `challenge3_keeper_final_v2.py`: defense strategy (Challenge3).
    - `ui/`
      - `challenge_ui.py`: Challenge UI + execution logic (including sending commands and logs).
      - `strategy_ui.py`: strategy selection UI.
  - `simulator/`
    - `simulator.py`: realtime simulator (opened from main UI).
    - `simulator_test.py`: **standalone full simulator** (currently uses Challenge3 strategy).
    - `simulator_appearance.py`, `vec_cal_func.py`: appearance + vector math.
  - `nrf/`
    - `nrf_controller.py`: NRF communication.
    - `nrf_ui.py`: NRF UI (scan ports, send commands, status).
  - `recording/`
    - `data_record.py`: data recording logic.
    - `recording_ui.py`: recording UI.
  - `config/`
    - `constants.py`: field constants (`FIELD/BOUNDARY/CENTER`) and data paths.
    - calibration / models / settings:
      - `calibration_*`, `color_settings*.json`, `strategy.txt`, `pi.pstates`, etc.
    - subfolders:
      - `calibration/`: `calib_cam*.npz`, `calibration_matrix_*`, `distortion_coefficients_*`, `field*.txt`, `points*.npy`, `HSV*.txt`
      - `images/`: calibration / test images
      - `models/`: model weights (`*.pth`, `*.pkl`)
- `camera_test/`: camera/calibration test scripts (not used directly by main program)

---

## 2. How to Run

### 2.1 Main GUI

From project root:

```
python src/main.py
```

GUI functions:
- **Vision Detection**: start vision processing and calibration
- **Monitor**: simplified simulator view (no dist/ang/vector overlay)
- **Simulator**: full simulator view (with overlay)
- **Challenge**: Challenge3 strategy selection and run
- **NRF**: scan serial ports / send fixed command

### 2.2 Standalone Simulator

```
python src/simulator/simulator_test.py
```

Notes:
- Does not depend on `main.py` GUI.
- Uses `strategies/challenge3_forward.py` as the strategy module.
- Right-click sets **robot position**.

---

## 3. Command Output Rate (Important)

### 3.1 `main.py`
No command rate limiting. It only calls `nrf_ui.send_cmd_safe(...)` from UI events.

### 3.2 `nrf_ui.py`
No rate limiting; it just sends commands.

### 3.3 `challenge_ui.py`
Challenge3 worker thread includes:

```
time.sleep(3)
```

So in Challenge3 mode, commands are sent roughly **once every 3 seconds**.

---

## 4. Common Config Locations

- Calibration points / HSV: `src/config/calibration_points.json`, `src/config/calibration_hsv.json`
- Calibration data: `src/config/calibration/`
- Models: `src/config/models/`

---

## 5. Notes

- Strategy files:
  - `strategies/challenge3_forward.py` (attack)
  - `strategies/challenge3_keeper_final_v2.py` (defense)
- Field boundary / center:
  - `src/config/constants.py`
