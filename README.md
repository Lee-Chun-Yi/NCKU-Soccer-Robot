# 1. NCKU-Soccer-Robot — GUI-Based Robotic Soccer System

This project presents a GUI-based robotic soccer system for **kicking and defensive behaviors**.
The system integrates **ArUco marker–based robot localization**, **ball visual detection**, **NRF-based command transmission**, and **soccer strategy execution**, enabling real-time perception, decision-making, and actuation within a unified interface.

## Relative folder:

* [Main GUI entry](./src/main.py) → GUI entry point integrating vision, NRF communication, and monitor/simulator modules

* [Vision system](./src/vision) → Visual perception and localization

  * [imageprocess.py](./src/vision/imageprocess.py) → Ball and robot detection, angle estimation, and localization
  * [vision_ui.py](./src/vision/vision_ui.py) → Calibration UI (HSV tuning and color masking)

* [Strategy module](./src/strategies) → Kicking and defensive strategies

  * [challenge3_forward.py](./src/strategies/challenge3_forward.py) → Attack strategy (Challenge 3)
  * [challenge3_keeper_final_v2.py](./src/strategies/challenge3_keeper_final_v2.py) → Defense strategy (Challenge 3)
  * [Challenge UI](./src/strategies/ui/challenge_ui.py) → Strategy execution UI, command dispatch, and logging
  * [Strategy selection UI](./src/strategies/ui/strategy_ui.py) → Strategy selection interface

* [Simulator](./src/simulator) → Real-time visualization and simulation

  * [simulator.py](./src/simulator/simulator.py) → Simulator launched from the main GUI
  * [simulator_test.py](./src/simulator/simulator_test.py) → Standalone full simulator (Challenge 3)
  * [simulator_appearance.py](./src/simulator/simulator_appearance.py) → Visualization settings
  * [vec_cal_func.py](./src/simulator/vec_cal_func.py) → Vector and geometry calculations

* [NRF communication](./src/nrf) → Wireless command transmission

  * [nrf_controller.py](./src/nrf/nrf_controller.py) → NRF communication logic
  * [nrf_ui.py](./src/nrf/nrf_ui.py) → NRF UI (port scanning, command sending, status)

* [Data recording](./src/recording) → Runtime data logging

  * [data_record.py](./src/recording/data_record.py) → Data recording logic
  * [recording_ui.py](./src/recording/recording_ui.py) → Recording control UI

* [Configuration](./src/config) → Field, calibration, and model configuration

  * [constants.py](./src/config/constants.py) → Field constants (FIELD / BOUNDARY / CENTER) and data paths
  * [Calibration data](./src/config/calibration) → Camera calibration matrices, distortion coefficients, field geometry
  * [Images](./src/config/images) → Calibration and testing images
  * [Models](./src/config/models) → Model weights (`*.pth`, `*.pkl`)
  * Calibration and settings files → HSV, color masks, strategy settings, robot states

* [Camera test scripts](./camera_test) → Camera and calibration test utilities (not used in main program)

---

## How to Run

### Main GUI

From project root:

```
python src/main.py
```

Main GUI functions:

* Vision Detection → Start visual detection and calibration
* Monitor → Simplified simulator view
* Simulator → Full simulator with overlays
* Challenge → Challenge 3 strategy selection and execution
* NRF → Serial port scanning and command transmission

---

### Standalone Simulator

```
python src/simulator/simulator_test.py
```

Notes:

* Does not depend on the main GUI
* Uses `challenge3_forward.py` as the strategy module
* Right-click interaction sets robot position

---

### Command Output Rate

* `main.py` → No command rate limiting; commands are triggered by UI events
* `nrf_ui.py` → Direct command transmission without rate limiting
* `challenge_ui.py` → Worker thread includes `time.sleep(3)`, resulting in command output approximately **once every 3 seconds** in Challenge 3 mode

---

### Common Configuration Locations

* Calibration points and HSV settings → `src/config/`
* Camera calibration data → `src/config/calibration/`
* Model weights → `src/config/models/`
* Field boundary and center definitions → `src/config/constants.py`


