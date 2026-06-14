# Plan: 在 modules/algorithm 中添加通用 LQR 控制器

## Context

**问题：** `modules/algorithm/algorithm.md` 文档提到了 `LQR.h` 模块，但该文件实际不存在。

**发现：**
- 项目已有应用层专用实现 `application/chassis/wbr_lqr_calc.h`（轮腿机器人整车10状态LQR，使用离线K表+双线性插值）
- 算法模块缺少通用的、可复用的 LQR 控制器实现
- 现有算法模块包括：PID控制器、卡尔曼滤波器、四元数EKF、CRC校验、工具函数库

**目标：** 创建通用 LQR 控制器模块，提供标准的线性二次调节器接口，同时保留应用层的优化实现。

---

## Implementation Approach

### 设计决策

#### 1. **架构模式：实例化设计**
- 参考 `controller.h` 的 `PIDInstance` 模式
- 支持多个独立的LQR控制器实例（左右腿、整车、云台等）
- 比全局单例模式更灵活，适合复杂系统

#### 2. **分层设计**
```
通用层: modules/algorithm/lqr_controller.h/c
  └─ 基础LQR计算框架
     ├─ 状态反馈 u = -Kx
     ├─ 参考跟踪 u = -K(x - x_ref)
     ├─ 动态增益更新
     └─ ARM CMSIS-DSP 矩阵运算

应用层: application/chassis/wbr_lqr_calc.h（保持独立）
  └─ 特化实现
     ├─ 离线K表存储（10×10网格）
     ├─ 双线性插值
     ├─ 应用特定状态组装
     └─ （可选）内部调用通用模块
```

#### 3. **核心特性**
- **实例化管理**：每个LQR控制器独立配置和运行
- **动态内存分配**：自动分配所需矩阵空间（使用 `user_malloc`，兼容FreeRTOS）
- **ARM CMSIS-DSP 集成**：高效矩阵运算，与Kalman滤波器一致
- **特性标志**：通过位标志启用可选功能（参考跟踪、前馈、限幅、动态增益）
- **用户钩子函数**：支持自定义预处理、后处理、增益调度

---

## Critical Files to Create/Modify

### 新建文件

#### 1. `modules/algorithm/lqr_controller.h`
**用途：** LQR控制器头文件，定义数据结构和接口

**核心结构体：**
```c
typedef struct lqr_t {
    /* 维度信息 */
    uint8_t StateSize;      // 状态维度 n
    uint8_t ControlSize;    // 控制维度 m
    
    /* 特性标志 */
    uint8_t Features;       // 功能启用标志位
    
    /* 状态和控制向量 */
    float *State;           // 当前状态 x[n]
    float *Reference;       // 参考状态 x_ref[n]
    float *Control;         // 控制输出 u[m]
    float *Feedforward;     // 前馈控制 u_ff[m]
    
    /* 增益矩阵 K[m×n] */
    float *GainMatrix;      // 增益矩阵存储（行主序）
    mat K;                  // ARM DSP 矩阵包装
    
    /* 输出限幅 */
    float *OutputMin;       // 输出下限[m]
    float *OutputMax;       // 输出上限[m]
    
    /* 临时计算空间 */
    float *StateError;      // 状态误差 x_err[n]
    mat x_err, u;           // ARM DSP 矩阵包装
    
    /* 用户钩子函数 */
    void (*PreProcess)(struct lqr_t *lqr);
    void (*PostProcess)(struct lqr_t *lqr);
    void (*UpdateGain)(struct lqr_t *lqr);
    
    /* 调试信息 */
    uint32_t UpdateCount;
    int8_t MatStatus;
} LQRInstance;
```

**主要接口：**
- `void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config)`
- `float *LQRCalculate(LQRInstance *lqr, const float *state)`
- `float *LQRCalculateWithReference(LQRInstance *lqr, const float *state, const float *ref)`
- `void LQRUpdateGain(LQRInstance *lqr, const float *new_gain)`
- `void LQRSetFeedforward(LQRInstance *lqr, const float *feedforward)`
- `void LQRReset(LQRInstance *lqr)`

#### 2. `modules/algorithm/lqr_controller.c`
**用途：** LQR控制器实现

**核心功能：**
- 初始化：分配内存、配置矩阵、设置钩子
- 计算核心：状态反馈计算 `u = -K*(x - x_ref) + u_ff`
- 增益更新：动态更新K矩阵（支持增益调度）
- 限幅处理：输出饱和限制

**关键实现点：**
```c
float *LQRCalculate(LQRInstance *lqr, const float *state) {
    // 1. 更新状态
    memcpy(lqr->State, state, sizeof(float) * lqr->StateSize);
    
    // 2. 用户预处理（角度归一化等）
    if (lqr->PreProcess != NULL) {
        lqr->PreProcess(lqr);
    }
    
    // 3. 动态增益更新（增益调度）
    if ((lqr->Features & LQR_USE_DYNAMIC_GAIN) && lqr->UpdateGain != NULL) {
        lqr->UpdateGain(lqr);
    }
    
    // 4. 计算状态误差
    if (lqr->Features & LQR_USE_REFERENCE_TRACKING) {
        for (uint8_t i = 0; i < lqr->StateSize; i++) {
            lqr->StateError[i] = lqr->State[i] - lqr->Reference[i];
        }
    } else {
        memcpy(lqr->StateError, lqr->State, sizeof(float) * lqr->StateSize);
    }
    
    // 5. 矩阵乘法 u = -K * x_err
    lqr->MatStatus = Matrix_Multiply(&lqr->K, &lqr->x_err, &lqr->u);
    for (uint8_t i = 0; i < lqr->ControlSize; i++) {
        lqr->Control[i] = -lqr->Control[i];
    }
    
    // 6. 添加前馈
    if (lqr->Features & LQR_USE_FEEDFORWARD) {
        for (uint8_t i = 0; i < lqr->ControlSize; i++) {
            lqr->Control[i] += lqr->Feedforward[i];
        }
    }
    
    // 7. 输出限幅
    if (lqr->Features & LQR_USE_OUTPUT_LIMIT) {
        for (uint8_t i = 0; i < lqr->ControlSize; i++) {
            if (lqr->Control[i] < lqr->OutputMin[i]) {
                lqr->Control[i] = lqr->OutputMin[i];
            } else if (lqr->Control[i] > lqr->OutputMax[i]) {
                lqr->Control[i] = lqr->OutputMax[i];
            }
        }
    }
    
    // 8. 用户后处理（自定义限幅等）
    if (lqr->PostProcess != NULL) {
        lqr->PostProcess(lqr);
    }
    
    lqr->UpdateCount++;
    return lqr->Control;
}
```

### 修改文件

#### 3. `modules/algorithm/algorithm.md`
**修改内容：**
- 更新第16行，将 `LQR.h` 改为 `lqr_controller.h`
- 补充详细的LQR模块说明和使用示例

---

## Reused Patterns from Existing Code

### 1. **从 controller.h (PID) 复用**
- 实例化结构设计
- 初始化配置结构体 (`PID_Init_Config_s` → `LQR_Init_Config_s`)
- 特性标志位管理 (`Improve_e` → `LQR_Feature_e`)
- 计算函数返回内部指针

### 2. **从 kalman_filter.h 复用**
- ARM CMSIS-DSP 矩阵运算封装
- 动态内存分配策略（`user_malloc`）
- 矩阵类型定义宏：
  ```c
  #define mat arm_matrix_instance_f32
  #define Matrix_Init arm_mat_init_f32
  #define Matrix_Multiply arm_mat_mult_f32
  ```
- 用户钩子函数扩展机制

### 3. **从 QuaternionEKF.h 复用**
- 预处理/后处理钩子函数模式
- 调试计数器和状态标志

---

## Dependencies

**已有依赖（项目中已使用）：**
- `arm_math.h` - ARM CMSIS-DSP 数学库（Kalman滤波器也在使用）
- `stdint.h`, `stdlib.h`, `string.h` - 标准库
- `user_lib.h` - 通用工具函数（限幅、数学函数）

**内存管理：**
```c
#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc  // FreeRTOS
#else
#define user_malloc malloc        // 标准库
#endif
#endif
```

**矩阵运算：** 完全依赖 ARM CMSIS-DSP 库（已在项目中）

---

## Usage Examples

### 示例1：简单倒立摆控制（固定增益）
```c
LQRInstance pendulum_lqr;

void PendulumInit(void) {
    static float K_data[2] = {-15.8114f, -3.1623f};  // K = [-15.81, -3.16]
    static float output_min = -10.0f, output_max = 10.0f;
    
    LQR_Init_Config_s config = {
        .StateSize = 2,
        .ControlSize = 1,
        .Features = LQR_USE_OUTPUT_LIMIT,
        .InitialGain = K_data,
        .OutputMin = &output_min,
        .OutputMax = &output_max,
    };
    
    LQRInit(&pendulum_lqr, &config);
}

void PendulumTask(void) {
    float state[2] = {theta, theta_dot};
    float *control = LQRCalculate(&pendulum_lqr, state);
    ApplyForce(control[0]);
}
```

### 示例2：整车轮腿平衡（动态增益+参考跟踪）
```c
LQRInstance wbr_lqr;

void WbrLqrUpdateGain(LQRInstance *lqr) {
    extern float left_leg_len, right_leg_len;
    float K_interpolated[4][10];
    WbrLqrInterpolateK(left_leg_len, right_leg_len, K_interpolated);
    LQRUpdateGain(lqr, (float *)K_interpolated);
}

void WbrLqrPreProcess(LQRInstance *lqr) {
    lqr->State[2] = WrapPi(lqr->State[2]);  // yaw角归一化
}

void WbrLqrInit(void) {
    static float output_min[4] = {-2.5f, -2.5f, -12.0f, -12.0f};
    static float output_max[4] = {2.5f, 2.5f, 12.0f, 12.0f};
    
    LQR_Init_Config_s config = {
        .StateSize = 10,
        .ControlSize = 4,
        .Features = LQR_USE_REFERENCE_TRACKING | LQR_USE_OUTPUT_LIMIT | LQR_USE_DYNAMIC_GAIN,
        .OutputMin = output_min,
        .OutputMax = output_max,
        .PreProcess = WbrLqrPreProcess,
        .UpdateGain = WbrLqrUpdateGain,
    };
    
    LQRInit(&wbr_lqr, &config);
}

void WbrLqrTask(void) {
    float state[10] = {s, s_dot, yaw, yaw_dot, left_theta, left_theta_dot,
                       right_theta, right_theta_dot, pitch, pitch_dot};
    float reference[10] = {target_s, target_v, target_yaw, target_wz, 0,0,0,0,0,0};
    
    memcpy(wbr_lqr.Reference, reference, sizeof(reference));
    float *control = LQRCalculate(&wbr_lqr, state);
    
    left_wheel_torque = control[0];
    right_wheel_torque = control[1];
    left_hip_torque = control[2];
    right_hip_torque = control[3];
}
```

---

## Performance Analysis

### 内存占用（WBR LQR 示例）
- 状态维度 n=10，控制维度 m=4
- 增益矩阵 K: 4×10×4 = 160 bytes
- 状态向量: 10×4 = 40 bytes
- 控制向量: 4×4 = 16 bytes
- 临时计算: 10×4 = 40 bytes
- **总计约 300 bytes/实例**

### 计算复杂度
- 矩阵乘法 u = -K×x: O(m×n) = O(40) 次浮点乘加
- H723 @ 480 MHz，使用DSP指令: **约 1-2 μs**
- 双线性插值（动态增益）: **约 5-10 μs**
- **总计 < 15 μs，完全满足 1 kHz 控制频率**

---

## Verification Plan

### 1. **单元测试**
- 固定增益测试：2状态1控制倒立摆
- 参考跟踪测试：验证 u = -K(x - x_ref)
- 限幅测试：验证输出饱和限制
- 动态增益测试：验证 UpdateGain 钩子

### 2. **集成测试**
- 与现有应用层代码 wbr_lqr_calc.h 共存
- 在实际轮腿控制任务中使用
- 悬空验证输出方向和幅值

### 3. **性能测试**
- 使用 `bsp_dwt.h` 测量计算时间
- 验证 1 kHz 控制频率下 CPU 占用率 < 2%
- 验证内存分配成功且无泄漏

---

## Integration Notes

### 与现有 wbr_lqr_calc.h 的关系

**选项A：保持独立（推荐）**
- wbr_lqr_calc.h 继续使用当前优化实现（已验证、高效）
- 通用 LQR 模块用于其他应用场景（单腿、云台、新控制器）
- 两者共存，各司其职

**选项B：内部集成**
- wbr_lqr_calc.h 内部调用通用模块的核心计算函数
- 插值逻辑保留在 wbr_lqr_calc.h
- 矩阵运算委托给通用模块

### 构建集成
无需修改 Makefile，只需在需要使用 LQR 的应用文件中：
```c
#include "lqr_controller.h"
```

---

## Risks and Mitigations

### 风险1：动态内存分配失败
**缓解措施：**
- 初始化时检查 malloc 返回值
- 提供静态内存分配版本（通过编译宏切换）
- 在系统启动阶段完成所有 LQR 初始化

### 风险2：矩阵运算精度损失
**缓解措施：**
- 使用 ARM CMSIS-DSP 优化库（已在项目中验证）
- 单元测试覆盖边界情况
- 提供调试标志输出中间计算结果

### 风险3：与现有代码冲突
**缓解措施：**
- 文件命名清晰：`lqr_controller.h`（不与 wbr_lqr_calc.h 冲突）
- 结构体命名：`LQRInstance`（不与现有类型冲突）
- 仅在新应用中使用，不修改现有稳定代码

---

## File Structure Summary

```
modules/algorithm/
  ├── algorithm.md                 # 更新：添加LQR模块说明
  ├── lqr_controller.h             # 新增：LQR控制器接口
  ├── lqr_controller.c             # 新增：LQR控制器实现
  ├── controller.h/c               # 已有：PID控制器
  ├── kalman_filter.h/c            # 已有：卡尔曼滤波器
  ├── QuaternionEKF.h/c            # 已有：四元数EKF
  ├── user_lib.h/c                 # 已有：工具函数库
  └── crc8/crc16.h/c               # 已有：CRC校验

application/chassis/
  ├── wbr_lqr_calc.h               # 已有：轮腿专用LQR（保持独立）
  ├── balance.h/c                  # 已有：平衡控制应用
  └── ...
```

---

## Success Criteria

1. ✅ **编译通过**：新增文件无编译错误或警告
2. ✅ **单元测试通过**：基础功能验证（固定增益、参考跟踪、限幅）
3. ✅ **性能达标**：计算时间 < 15 μs @ 1 kHz
4. ✅ **内存可控**：单实例 < 500 bytes（10状态4控制）
5. ✅ **文档完整**：algorithm.md 更新，接口注释完整
6. ✅ **零破坏**：不影响现有 wbr_lqr_calc.h 和其他算法模块
7. ✅ **可扩展**：支持新的LQR控制器需求（云台、单腿等）

---

## Implementation Order

1. **创建头文件** `lqr_controller.h`
   - 定义数据结构
   - 声明接口函数
   - 添加文件头注释

2. **实现核心功能** `lqr_controller.c`
   - `LQRInit()` - 初始化和内存分配
   - `LQRCalculate()` - 核心计算逻辑
   - `LQRUpdateGain()` - 增益更新
   - 辅助函数

3. **更新文档** `algorithm.md`
   - 修正 LQR.h 引用为 lqr_controller.h
   - 添加使用说明和示例

4. **验证测试**
   - 编译验证
   - 简单示例测试（倒立摆）
   - 性能基准测试

---

## Notes

- 本实现遵循项目现有代码风格（参考 controller.h 和 kalman_filter.h）
- 完全基于已验证的依赖库（ARM CMSIS-DSP）
- 设计为非破坏性添加，不影响现有功能
- 为未来扩展预留钩子函数接口
- 支持 FreeRTOS 和裸机环境
