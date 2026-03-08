# 更新紀錄 — 2025/03/05

## 1. 射線投影座標校正（Ray-Cast Coordinate Mapping）

### 問題

`_map_tvec_to_field()` 使用硬編碼的 15 點二次多項式擬合做座標校正，換場地或碰到相機就要重新量測 15 個點並擬合係數。

### 解法

新增射線投影（Ray Casting）座標校正：利用使用者已有的**場地 4 角點** + **相機內參 K** + **機器人高度**，從數學上精確計算「相機 → 機器人高度平面」的射線交點，取代多項式擬合。

### 原理

```
1. 使用者選 4 場地角點（現有流程）
2. solvePnP → 相機在場地座標系中的位置 (R, t)
3. ArUco 像素中心 → K⁻¹ → 射線方向
4. 射線與 z = robot_height 平面取交點 → 場地座標 (X, Y) cm
```

### 修改檔案

**`src/vision/imageprocess.py`**

#### 新增函式

| 函式 | 說明 |
|---|---|
| `_compute_new_camera_matrix()` | 計算去畸變後的最佳內參矩陣 |
| `_compute_k_rotated()` | 計算旋轉 90° 後影像的內參矩陣 |
| `_compute_camera_extrinsics()` | 用場地 4 角點 + K 做 solvePnP，算出相機位置 |
| `_ray_cast_to_height()` | 射線投影到機器人高度平面 |
| `_rotate_pixel_to_rotated_frame()` | ArUco 像素座標旋轉對齊 |

#### 修改的函式

| 函式 | 修改內容 |
|---|---|
| `process_robot_pipeline()` | 新增 `cam_extrinsics`、`K_rot` 參數；有外參時自動用射線投影 |
| `_processing_loop()` | 計算 K_rot、相機外參，傳遞給 pipeline |
| `main()` | 同上 |

### 不影響的部分

- ✅ 角度計算（`_compute_angle_from_R`、`_mode_angle`）
- ✅ 方向向量（`vec_x`、`vec_y`）
- ✅ 多面姿態映射（`_compute_top_pose`）
- ✅ Ball 追蹤（透視變換 + HSV）

### 驗證方式

Robot Track 視窗左上角同時顯示新舊座標：
- `X` / `Y` — 主座標（射線投影）
- `OldX` / `OldY`（橙色）— 舊多項式擬合
- `RayX` / `RayY`（亮綠）— 射線投影

### 回退方式

在 `process_robot_pipeline()` 中找到以下程式碼並註解掉：
```python
if ray_fX is not None:
    fX, fY = ray_fX, ray_fY  # 註解掉退回舊方法
```

---

## 2. Bug 修復：`select_ball_reference_rect()`

修復複製貼上錯誤 — 原本函式主體為空（提前 return），實際邏輯被誤放在 `select_ball_reference_polygon()` 的 `return True` 之後成為 dead code。現已歸位到正確的函式中。
