# 🐾 NXP Jaguar: Motor Zero Calibration & Joint Offset Guide
# 🐾 恩智浦美洲豹機械狗：電機零點校準與關節偏移量指南

> **Languages / 語言:**  
> - 🇬🇧 **[English](#english-guide)**  
> - 🇹🇼 **[繁體中文 (Traditional Chinese)](#繁體中文指南-traditional-chinese)**

---

<a name="english-guide"></a>
## 🇬🇧 English Guide

### 1. Overview & Core Philosophy

This guide explains how **Mechanical Zero Calibration** and **Joint Angular Offsets** work on the NXP Jaguar quadruped robot powered by RoboStride RS00 BLDC motors.

#### Why Do We Need Offsets?
When assembling, adjusting bolts, or replacing motors on the robot, setting the motor encoder zero point directly in the sitting pose can be mechanically imprecise or difficult to reproduce. 

Instead, we use a two-pose system:
1. **Relax Pose (Calibration Reference Pose):** A highly repeatable mechanical pose (e.g., legs fully extended/flattened against a calibration jig or flat ground). Motors are flashed to **$\theta_{\text{raw}} = 0.0$** in this pose.
2. **Sitting Pose (Standby Pose, $\theta_{\text{joint}} = 0.0\text{ rad}$):** The folded sitting posture on the floor from which the robot stands up.
3. **Standing Pose (Default Stand Pose):** The nominal standing posture for locomotion.

Because the geometric distance between the **Relax Pose** and the **Sitting Pose** is fixed, the **Angular Offset (`MOTOR_OFFSET_ANGLE`) is permanent**. Any developer can set mechanical zero at the Relax Pose and achieve an exact `0.000 rad` sitting posture without recalculating offsets!

---

### 2. Kinematic Formulation

In both Python (`robstride_motor_lib.py`) and C++ (`robstride_hardware_manager.hpp`), joint angle calculation follows:

$$\theta_{\text{joint}} = (\theta_{\text{raw}} \times \text{MOTOR\_DIR}) + \text{MOTOR\_OFFSET\_ANGLE}$$

- **At Relax Pose ($\theta_{\text{raw}} = 0.0$):**
  $$\theta_{\text{joint}} = (0.0 \times \text{MOTOR\_DIR}) + \text{OFFSET} = \mathbf{\text{OFFSET}}$$
  *(The reading at Relax Pose directly equals the joint offset!)*

- **At Sitting Pose ($\theta_{\text{joint}} = 0.0\text{ rad}$):**
  $$\theta_{\text{raw, sit}} = -\mathbf{\text{OFFSET}} \times \text{MOTOR\_DIR}$$
  *(The motor automatically lands at exact 0.0 rad joint angle after offset addition!)*

---

### 3. Calibrated Joint Offsets Table

These values are calibrated for all 12 RoboStride RS00 motors:

| Joint Name | CAN Bus | CAN ID | Direction (`dir`) | Joint Offset (`rad`) | Angle (`deg`) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **BL_collar_joint** | `can1` | `#4` | `+1` | **`+0.3676 rad`** | $+21.1^\circ$ |
| **BL_hip_joint** | `can1` | `#5` | `-1` | **`-1.2600 rad`** | $-72.2^\circ$ |
| **BL_knee_joint** | `can1` | `#6` | `-1` | **`+0.1237 rad`** | $+7.1^\circ$ |
| **BR_collar_joint** | `can0` | `#4` | `+1` | **`-0.3638 rad`** | $-20.8^\circ$ |
| **BR_hip_joint** | `can0` | `#5` | `+1` | **`-1.2773 rad`** | $-73.2^\circ$ |
| **BR_knee_joint** | `can0` | `#6` | `+1` | **`-0.0247 rad`** | $-1.4^\circ$ |
| **FL_collar_joint** | `can1` | `#1` | `-1` | **`+0.3557 rad`** | $+20.4^\circ$ |
| **FL_hip_joint** | `can1` | `#2` | `-1` | **`-1.2592 rad`** | $-72.1^\circ$ |
| **FL_knee_joint** | `can1` | `#3` | `-1` | **`+0.0090 rad`** | $+0.5^\circ$ |
| **FR_collar_joint** | `can0` | `#1` | `-1` | **`-0.2932 rad`** | $-16.8^\circ$ |
| **FR_hip_joint** | `can0` | `#2` | `+1` | **`-1.1618 rad`** | $-66.6^\circ$ |
| **FR_knee_joint** | `can0` | `#3` | `+1` | **`-0.0539 rad`** | $-3.1^\circ$ |

---

### 4. Code Implementation Locations

#### Python: `scripts/parameters.py`
```python
MOTOR_OFFSET_ANGLE = [
     0.3845, -1.2983, -0.0788,  # BL (can1: Collar, Hip, Knee)
    -0.4017, -1.2976, -0.0339,  # BR (can0: Collar, Hip, Knee)
     0.3526, -1.2427,  0.0033,  # FL (can1: Collar, Hip, Knee)
    -0.3181, -1.2067, -0.0627,  # FR (can0: Collar, Hip, Knee)
]
```

#### C++: `include/jaguar_control/robstride_hardware_manager.hpp`
*(Note: C++ uses opposite sign because C++ calculates `pos = raw - cfg.angle_offset`)*
```cpp
// BL (can1)
joint_configs_[0] = {"BL_collar_joint", "can1", 4,  1, -0.3845, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[1] = {"BL_hip_joint",    "can1", 5, -1,  1.2983, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[2] = {"BL_knee_joint",   "can1", 6, -1,  0.0788, -0.10,  2.80, 20.0, 17.0, {}};

// BR (can0)
joint_configs_[3] = {"BR_collar_joint", "can0", 4,  1,  0.4017, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[4] = {"BR_hip_joint",    "can0", 5,  1,  1.2976, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[5] = {"BR_knee_joint",   "can0", 6,  1,  0.0339, -0.10,  2.80, 20.0, 17.0, {}};

// FL (can1)
joint_configs_[6] = {"FL_collar_joint", "can1", 1, -1, -0.3526, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[7] = {"FL_hip_joint",    "can1", 2, -1,  1.2427, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[8] = {"FL_knee_joint",   "can1", 3, -1, -0.0033, -0.10,  2.80, 20.0, 17.0, {}};

// FR (can0)
joint_configs_[9] = {"FR_collar_joint", "can0", 1, -1,  0.3181, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[10] ={"FR_hip_joint",    "can0", 2,  1,  1.2067, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[11] ={"FR_knee_joint",   "can0", 3,  1,  0.0627, -0.10,  2.80, 20.0, 17.0, {}};
```

---

### 5. Developer Standard Operating Procedure (SOP)

Whenever you adjust mechanical bolts, rebuild the leg frame, or swap a motor:

```
[1. Place in Relax Pose] ➔ [2. Run set_robostride_zero.py] ➔ [3. Verify via test_sit_stand.py]
```

1. **Step 1: Place robot into Relax Pose**  
   Manually align and rest all 4 legs in the standardized **Relax Pose**.
2. **Step 2: Flash Zero to Motors**  
   Execute the zero script:
   ```bash
   python3 scripts/set_robostride_zero.py
   ```
   Press `ENTER` to commit zero position to all 12 motors' flash memory.
3. **Step 3: Verify Passive Diagnostics**  
   Check that angles in Relax Pose match the offset values:
   ```bash
   python3 scripts/check_joints.py
   ```
4. **Step 4: Test Sit & Stand**  
   Place the robot on the ground in sitting pose and run:
   ```bash
   python3 scripts/test_sit_stand.py
   ```
   - Press **`[1]`** to hold Sit Pose (`0.000 rad`).
   - Press **`[2]`** to Stand Up smoothly (`Hip: -1.40 rad, Knee: +1.36 rad`).

---
---

<a name="繁體中文指南-traditional-chinese"></a>
## 🇹🇼 繁體中文指南 (Traditional Chinese)

### 1. 概述與核心概念

本指南說明恩智浦美洲豹（NXP Jaguar）四足機器人在使用 RoboStride RS00 無刷電機時，**機械零點校準（Mechanical Zero）** 與 **關節偏移量（Joint Offsets）** 的工作原理與操作規範。

#### 為什麼需要關節偏移量（Offset）？
在機械狗組裝、調整螺絲或更換電機時，如果每次都直接在「坐下姿態」設定零點，容易產生人為誤差且難以精準復現。

因此，我們採用**雙姿態校準體系**：
1. **放鬆姿態（Relax Pose / 校準基準姿態）：** 一個極具重複性與機械一致性的基準姿態（例如腿部完全伸展/平放於校準夾具或平整地面）。在此姿態下，電機內部編碼器被寫入為 **$\theta_{\text{raw}} = 0.0$**。
2. **坐下姿態（Sitting / Standby Pose，$\theta_{\text{joint}} = 0.0\text{ rad}$）：** 機器人摺疊靜止於地面、準備起立的待機姿態。
3. **站立姿態（Standing Pose）：** 機器人運動控制的額定站立姿態。

因為**放鬆姿態**到**坐下姿態**的機械幾何距離是固定的，所以**關節偏移量（`MOTOR_OFFSET_ANGLE`）是永久固定的常數**。任何開發者只需在「放鬆姿態」執行零點寫入，機器人便能自動在「坐下姿態」精準獲得 `0.000 rad`，無需重新調整代碼！

---

### 2. 運動學校準公式

在 Python（`robstride_motor_lib.py`）與 C++（`robstride_hardware_manager.hpp`）底層驅動中，關節角度計算統一遵循：

$$\theta_{\text{joint}} = (\theta_{\text{raw}} \times \text{MOTOR\_DIR}) + \text{MOTOR\_OFFSET\_ANGLE}$$

- **在放鬆姿態下（$\theta_{\text{raw}} = 0.0$）：**
  $$\theta_{\text{joint}} = (0.0 \times \text{MOTOR\_DIR}) + \text{OFFSET} = \mathbf{\text{OFFSET}}$$
  *（在放鬆姿態讀取的關節角度，恰好等於該關節的偏移量數值！）*

- **在坐下姿態下（目標 $\theta_{\text{joint}} = 0.0\text{ rad}$）：**
  $$\theta_{\text{raw, sit}} = -\mathbf{\text{OFFSET}} \times \text{MOTOR\_DIR}$$
  *（電機自動轉動相應角度，加上 Offset 後精準歸零 `0.000 rad`！）*

---

### 3. 已校準的關節偏移量表（12 個電機）

以下為整機 12 顆 RoboStride RS00 電機的精準校準數據：

| 關節名稱 (Joint Name) | CAN 總線 | 電機 ID | 旋轉方向 (`dir`) | **關節偏移量 (`rad`)** | 角度值 (`deg`) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **BL_collar_joint**（左後側展） | `can1` | `#4` | `+1` | **`+0.3676 rad`** | $+21.1^\circ$ |
| **BL_hip_joint**（左後大腿） | `can1` | `#5` | `-1` | **`-1.2600 rad`** | $-72.2^\circ$ |
| **BL_knee_joint**（左後小腿） | `can1` | `#6` | `-1` | **`+0.1237 rad`** | $+7.1^\circ$ |
| **BR_collar_joint**（右後側展） | `can0` | `#4` | `+1` | **`-0.3638 rad`** | $-20.8^\circ$ |
| **BR_hip_joint**（右後大腿） | `can0` | `#5` | `+1` | **`-1.2773 rad`** | $-73.2^\circ$ |
| **BR_knee_joint**（右後小腿） | `can0` | `#6` | `+1` | **`-0.0247 rad`** | $-1.4^\circ$ |
| **FL_collar_joint**（左前側展） | `can1` | `#1` | `-1` | **`+0.3557 rad`** | $+20.4^\circ$ |
| **FL_hip_joint**（左前大腿） | `can1` | `#2` | `-1` | **`-1.2592 rad`** | $-72.1^\circ$ |
| **FL_knee_joint**（左前小腿） | `can1` | `#3` | `-1` | **`+0.0090 rad`** | $+0.5^\circ$ |
| **FR_collar_joint**（右前側展） | `can0` | `#1` | `-1` | **`-0.2932 rad`** | $-16.8^\circ$ |
| **FR_hip_joint**（右前大腿） | `can0` | `#2` | `+1` | **`-1.1618 rad`** | $-66.6^\circ$ |
| **FR_knee_joint**（右前小腿） | `can0` | `#3` | `+1` | **`-0.0539 rad`** | $-3.1^\circ$ |

---

### 4. 代碼配置位置

#### Python 端：`scripts/parameters.py`
```python
MOTOR_OFFSET_ANGLE = [
     0.3845, -1.2983, -0.0788,  # BL (can1: Collar, Hip, Knee)
    -0.4017, -1.2976, -0.0339,  # BR (can0: Collar, Hip, Knee)
     0.3526, -1.2427,  0.0033,  # FL (can1: Collar, Hip, Knee)
    -0.3181, -1.2067, -0.0627,  # FR (can0: Collar, Hip, Knee)
]
```

#### C++ 端：`include/jaguar_control/robstride_hardware_manager.hpp`
*(注意：C++ 節點底層計算為 `pos = raw - cfg.angle_offset`，因此數值為負號相反數)*
```cpp
// BL (can1)
joint_configs_[0] = {"BL_collar_joint", "can1", 4,  1, -0.3845, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[1] = {"BL_hip_joint",    "can1", 5, -1,  1.2983, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[2] = {"BL_knee_joint",   "can1", 6, -1,  0.0788, -0.10,  2.80, 20.0, 17.0, {}};

// BR (can0)
joint_configs_[3] = {"BR_collar_joint", "can0", 4,  1,  0.4017, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[4] = {"BR_hip_joint",    "can0", 5,  1,  1.2976, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[5] = {"BR_knee_joint",   "can0", 6,  1,  0.0339, -0.10,  2.80, 20.0, 17.0, {}};

// FL (can1)
joint_configs_[6] = {"FL_collar_joint", "can1", 1, -1, -0.3526, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[7] = {"FL_hip_joint",    "can1", 2, -1,  1.2427, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[8] = {"FL_knee_joint",   "can1", 3, -1, -0.0033, -0.10,  2.80, 20.0, 17.0, {}};

// FR (can0)
joint_configs_[9] = {"FR_collar_joint", "can0", 1, -1,  0.3181, -0.40,  0.40, 20.0, 17.0, {}};
joint_configs_[10] ={"FR_hip_joint",    "can0", 2,  1,  1.2067, -3.14,  3.14, 20.0, 17.0, {}};
joint_configs_[11] ={"FR_knee_joint",   "can0", 3,  1,  0.0627, -0.10,  2.80, 20.0, 17.0, {}};
```

---

### 5. 開發者標準操作流程（SOP）

未來任何時候只要您重新鎖緊機械螺絲、拆裝腿部結構或更換新電機：

```
[1. 手動擺入放鬆姿態] ➔ [2. 執行 set_robostride_zero.py] ➔ [3. 執行 test_sit_stand.py 驗證]
```

1. **步驟 1：手動擺入「放鬆姿態」**  
   將機器人四條腿手動對齊並擺入標準的 **放鬆姿態（Relax Pose）**。
2. **步驟 2：寫入電機硬體零點**  
   在終端機執行零點寫入腳本：
   ```bash
   python3 scripts/set_robostride_zero.py
   ```
   終端提示時按下 `ENTER`，指令會透過 CAN 總線將當前位置寫入 12 顆電機的 Flash 記憶體（通訊模式 Type 6）。
3. **步驟 3：被動診斷讀取驗證**  
   此時在放鬆姿態下檢查關節數值，讀數應大致等於 Offset 偏移量：
   ```bash
   python3 scripts/check_joints.py
   ```
4. **步驟 4：測試坐下與起立動作**  
   將機器人置於地面坐下姿態，執行測試腳本：
   ```bash
   python3 scripts/test_sit_stand.py
   ```
   - 按下 **`[1]`**：鎖定坐下待機姿態（所有關節精準處於 `0.000 rad`）。
   - 按下 **`[2]`**：平滑起立至站立姿態（大腿: `-1.40 rad`，小腿: `+1.36 rad`）。
