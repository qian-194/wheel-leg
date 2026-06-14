/**
 ******************************************************************************
 * @file    tcm_region.h
 * @brief   STM32H723 ITCM / DTCM 内存区域放置宏
 *
 * 【内存架构速查】
 *  ITCMRAM  0x00000000  64 KB   CPU取指零等待，不走AXI总线，DMA不可访问
 *  DTCMRAM  0x20000000 128 KB   CPU读写零等待，不走AXI总线，DMA不可访问  ← 栈也在这里
 *  AXI SRAM 0x24000000 128 KB   普通.bss/.data，DMA可访问，总线仲裁有延迟
 *  RAM_D2   0x30000000  32 KB   DMA专用缓冲区
 *
 * 【链接脚本段对应关系】(STM32H723VGTx_FLASH.ld)
 *  ITCM_FUNC   → .itcm_text   → >ITCMRAM AT>FLASH   启动时自动从FLASH拷贝
 *  DTCM_DATA   → .dtcm_data   → >DTCMRAM AT>FLASH   启动时自动从FLASH拷贝（有初始值）
 *  DTCM_BSS    → .dtcm_bss    → >DTCMRAM            启动时自动清零
 *
 * 【使用原则】
 *  ITCM  ← 放函数（高频热路径）：500Hz WBC雅可比、五连杆IK、PID计算、INS四元数更新
 *  DTCM  ← 放数据（高频读写）  ：MPC预测矩阵、五连杆状态、WBC中间量、控制器状态结构体
 *
 ******************************************************************************
 */

#ifndef TCM_REGION_H
#define TCM_REGION_H

#ifdef __GNUC__

/* ----------------------------------------------------------------
 * ITCM：函数放置宏
 *   - 仅用于 .c 文件中的函数【定义】处，头文件声明不需要加
 *   - static inline 函数不可用（inline会内联展开，无法放入固定段）
 *   - ITCM 64KB，放入代码后务必通过 .map 文件确认未溢出
 *
 * 示例：
 *   ITCM_FUNC void WBC_JacobianUpdate(float *J, const float *q) { ... }
 *   ITCM_FUNC float PIDCalculate_Fast(PIDInstance *pid, float meas, float ref);
 * ---------------------------------------------------------------- */
#define ITCM_FUNC   __attribute__((section(".itcm_text"), noinline))

/* ----------------------------------------------------------------
 * DTCM：有初始值的全局/静态变量
 *   - 初始值存在FLASH，启动时自动拷贝到DTCM
 *   - 适合：需要非零默认值的增益表、查找表、常量系数
 *
 * 示例：
 *   DTCM_DATA static float kMpcWeightQ[6] = {10,10,1,1,1,1};
 *   DTCM_DATA const float kFiveLinkL[4]   = {0.15f, 0.25f, 0.25f, 0.15f};
 * ---------------------------------------------------------------- */
#define DTCM_DATA   __attribute__((section(".dtcm_data")))

/* ----------------------------------------------------------------
 * DTCM：零初始化全局/静态变量（最常用）
 *   - 不占用FLASH空间，启动时自动清零
 *   - 适合：大型矩阵缓冲区、状态量、中间计算量
 *   - DTCM 128KB，当前栈占用约20KB，可用约108KB
 *
 * 示例：
 *   DTCM_BSS static float g_mpc_A[12*12];       // MPC预测矩阵
 *   DTCM_BSS static float g_wbc_jacobian[6*12];  // WBC雅可比
 *   DTCM_BSS static float g_five_link_state[10]; // 五连杆状态量
 * ---------------------------------------------------------------- */
#define DTCM_BSS    __attribute__((section(".dtcm_bss")))

/* ----------------------------------------------------------------
 * RAM_D2：DMA 安全访问区
 *   - 外设DMA收发缓冲区，必须放这里避免Cache一致性问题
 *   - 已有 .dma_buffer 段用于DMA，此宏用于非DMA但需外设访问的数据
 *
 * 示例：
 *   RAM_D2_BSS uint8_t g_can_rx_shadow[8];
 * ---------------------------------------------------------------- */
#define RAM_D2_BSS  __attribute__((section(".ram_d2")))

#else
/* 非GCC编译器（Keil MDK/IAR）回退为空宏，不影响功能 */
#define ITCM_FUNC
#define DTCM_DATA
#define DTCM_BSS
#define RAM_D2_BSS
#endif /* __GNUC__ */


/* ================================================================
 * 使用规范速查表（针对 MPC + WBC + 五连杆 串腿方案）
 * ================================================================
 *
 *  放 ITCM 的函数（频率越高越值得放）：
 *  ┌─────────────────────────────────────┬────────┬──────────────┐
 *  │ 函数                                │ 频率   │ 估算代码大小 │
 *  ├─────────────────────────────────────┼────────┼──────────────┤
 *  │ WBC_JacobianUpdate()                │ 500Hz  │ ~1 KB        │
 *  │ WBC_TorqueAllocation()              │ 500Hz  │ ~1 KB        │
 *  │ FiveLink_ForwardKinematics()        │ 500Hz  │ ~0.5 KB      │
 *  │ FiveLink_InverseKinematics()        │ 500Hz  │ ~1 KB        │
 *  │ PIDCalculate()  ← 可拷贝一个快速版 │ 1kHz   │ ~0.5 KB      │
 *  │ QuaternionUpdate()                  │ 1kHz   │ ~0.5 KB      │
 *  │ MPC_PredictStep()                   │ 100Hz  │ ~2 KB        │
 *  └─────────────────────────────────────┴────────┴──────────────┘
 *  合计约 6~7 KB，ITCM 64KB 足够
 *
 *  放 DTCM_BSS 的数据（高频读写的状态量/矩阵）：
 *  ┌─────────────────────────────────────┬──────────────┐
 *  │ 变量                                │ 估算大小     │
 *  ├─────────────────────────────────────┼──────────────┤
 *  │ MPC 预测矩阵 A_bar, B_bar (Np=10)  │ ~2 KB        │
 *  │ WBC 雅可比矩阵 J[6×12]             │ ~288 B       │
 *  │ 五连杆关节状态 q[10], dq[10]       │ ~80 B        │
 *  │ WBC 力矩输出缓存 tau[12]           │ ~48 B        │
 *  │ MPC 状态向量 x[12]                 │ ~48 B        │
 *  └─────────────────────────────────────┴──────────────┘
 *  合计约 3~5 KB，DTCM 108KB 可用空间充足
 * ================================================================ */

#endif /* TCM_REGION_H */
