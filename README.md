# 0114 第一次成功踢到球 — 專案說明

本專案已整理為模組化結構。以下內容以 **專案根目錄**（`...\0114`）為基準說明，主要執行入口為 `src/main.py`。

---

## 一、專案結構（最新版）

- `src/`：主程式與模組
  - `main.py`：GUI 主入口（影像偵測、NRF、Monitor / Simulator）。
  - `vision/`
    - `imageprocess.py`：影像處理與定位（球、機器人、角度）。
    - `vision_ui.py`：影像校正 / HSV / ColorMask UI。
  - `strategies/`
    - `challenge3_forward.py`：進攻策略（Challenge3）。
    - `challenge3_keeper_final_v2.py`：防守策略（Challenge3）。
    - `ui/`
      - `challenge_ui.py`：Challenge UI + 執行邏輯（含送指令與顯示）。
      - `strategy_ui.py`：策略選擇 UI。
  - `simulator/`
    - `simulator.py`：即時模擬器（主程式按鈕呼叫）。
    - `simulator_test.py`：**可獨立執行的完整模擬器**（目前設為 Challenge3 策略）。
    - `simulator_appearance.py`、`vec_cal_func.py`：模擬器外觀與向量計算。
  - `nrf/`
    - `nrf_controller.py`：NRF 通訊。
    - `nrf_ui.py`：NRF UI（搜尋、送指令、狀態）。
  - `recording/`
    - `data_record.py`：資料記錄。
    - `recording_ui.py`：記錄 UI。
  - `config/`
    - `constants.py`：場地常數（`FIELD/BOUNDARY/CENTER`）與資料目錄。
    - 校正 / 模型 / 設定：
      - `calibration_*`, `color_settings*.json`, `strategy.txt`, `pi.pstates`...
    - 子資料夾：
      - `calibration/`：`calib_cam*.npz`、`calibration_matrix_*`、`distortion_coefficients_*`、`field*.txt`、`points*.npy`、`HSV*.txt`
      - `images/`：校正/測試影像
      - `models/`：模型/權重（`*.pth`, `*.pkl`）
- `camera_test/`：攝影機與校正測試工具（主程式未直接調用）

---

## 二、執行方式

### 1) 主程式（GUI）

在專案根目錄執行：

```
python src/main.py
```

GUI 功能：
- **影像偵測**：啟動影像處理與校正流程
- **Monitor**：簡化版模擬視窗（無 dist/ang/向量箭頭）
- **Simulator**：完整模擬視窗（含 overlay）
- **挑戰**：Challenge3 的策略選擇與執行
- **NRF**：搜尋序列埠 / 送固定指令

### 2) 模擬器（獨立）

```
python src/simulator/simulator_test.py
```

說明：
- 不依賴 `main.py` GUI。
- 使用 `strategies/challenge3_forward.py` 作為策略來源。
- 右鍵設定 **機器人位置**。

---

## 三、指令送出頻率（重點整理）

### 1) `main.py`
**不限制**機器人指令輸出頻率。  
僅透過 UI 呼叫 `nrf_ui.send_cmd_safe(...)`，沒有節流/限速。

### 2) `nrf_ui.py`
**不限制**指令頻率，只負責送出。

### 3) `challenge_ui.py`
Challenge3 執行緒內有 **固定間隔**：

```
time.sleep(3)
```

所以 Challenge3 模式下 **約每 3 秒**送一次指令。

---

## 四、常見設定檔位置

- 校正點 / HSV：`src/config/calibration_points.json`, `src/config/calibration_hsv.json`
- 校正資料：`src/config/calibration/`
- 模型檔：`src/config/models/`

---

## 五、備註

- 若要變更策略，主要設定在：
  - `strategies/challenge3_forward.py`（進攻）
  - `strategies/challenge3_keeper_final_v2.py`（防守）
- 若要更改場地邊界或中心點，請修改：
  - `src/config/constants.py`

