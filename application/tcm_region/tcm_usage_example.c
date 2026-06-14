/**
 ******************************************************************************
 * @file    tcm_usage_example.c
 * @brief   ITCM / DTCM 使用示例（针对 MPC + WBC + 五连杆串腿方案）
 *
 * 本文件【不参与编译】，仅作为代码模板和使用规范的参考
 * 实际使用时将对应片段复制到你的 wbc.c / mpc.c / five_link.c 等文件中
 *
 * 加入 Makefile 前请删除下面的 #error 行
 ******************************************************************************
 */
/* 防止被意外加入 Makefile 编译 —— 如需参与编译请删除此行 */
#error "tcm_usage_example.c is a TEMPLATE file. Do NOT add it to Makefile C_SOURCES. Copy the snippets you need into your own .c files."

#include "tcm_region.h"
#include "arm_math.h"   // arm_matrix_instance_f32, arm_mat_mult_f32 等
#include "controller.h" // PIDInstance


/* ================================================================
 * 第一部分：DTCM_BSS —— 大块矩阵和状态量（零初始化，不占FLASH）
 * ================================================================
 * 放在文件作用域（全局或static），不能放在函数内部局部变量
 * ================================================================ */

/* --- WBC 雅可比矩阵（6腿×2关节 = 6×12，单精度浮点 = 288 B） --- */
DTCM_BSS static float g_J_wbc[6][12];       // 约束雅可比 J
DTCM_BSS static float g_JT_wbc[12][6];      // J 的转置
DTCM_BSS static float g_tau_wbc[12];        // WBC 力矩输出

/* --- 五连杆串腿状态量 --- */
DTCM_BSS static float g_q_five_link[10];    // 10个关节角 (rad)
DTCM_BSS static float g_dq_five_link[10];   // 关节角速度 (rad/s)
DTCM_BSS static float g_foot_pos[5][3];     // 5个末端位置 xyz (m)
DTCM_BSS static float g_foot_vel[5][3];     // 5个末端速度 xyz (m/s)

/* --- MPC 预测矩阵（Np=10，状态12维，输入12维） ---
 * A_bar: (12*Np) × 12 = 120×12 → 1440个float → 5760 B ≈ 5.6 KB
 * B_bar: (12*Np) × (12*Np) = 120×120 → 57600 B ← 太大！建议缩减Np或降维
 * 实际建议：状态6维(质心位姿+速度)，输入4维(4条腿法向力)，Np=8
 * 缩减后 B_bar: 48×32 → 6144 B ≈ 6 KB，合理
 */
#define MPC_STATE_DIM   6   // 质心位置xyz + 速度xyz
#define MPC_INPUT_DIM   4   // 4腿支撑力（法向）
#define MPC_PRED_HORIZON 8  // 预测步数

DTCM_BSS static float g_mpc_A_bar[MPC_STATE_DIM * MPC_PRED_HORIZON][MPC_STATE_DIM];
DTCM_BSS static float g_mpc_B_bar[MPC_STATE_DIM * MPC_PRED_HORIZON][MPC_INPUT_DIM * MPC_PRED_HORIZON];
DTCM_BSS static float g_mpc_x[MPC_STATE_DIM];       // 当前状态向量
DTCM_BSS static float g_mpc_u_opt[MPC_INPUT_DIM];   // 最优输入（本拍）


/* ================================================================
 * 第二部分：DTCM_DATA —— 有非零初始值的增益/参数表
 * ================================================================
 * 初始值存在FLASH，启动时自动拷贝到DTCM
 * 适合：PID增益、权重矩阵对角线、连杆长度参数
 * ================================================================ */

/* --- 五连杆几何参数（单位：m） --- */
DTCM_DATA static const float kFiveLinkLen[5] = {
    0.060f,  // L0: 髋关节到第一连杆
    0.200f,  // L1: 大腿杆
    0.200f,  // L2: 小腿杆
    0.060f,  // L3: 脚踝到末端
    0.250f   // L4: 躯干到髋关节横向距离
};

/* --- MPC 状态权重对角线 Q, 控制权重 R --- */
DTCM_DATA static float kMpcWeightQ[MPC_STATE_DIM] = {
    100.0f, 100.0f, 200.0f,  // 位置 xyz
     10.0f,  10.0f,  20.0f   // 速度 xyz
};
DTCM_DATA static float kMpcWeightR[MPC_INPUT_DIM] = {
    1e-4f, 1e-4f, 1e-4f, 1e-4f  // 各腿支撑力
};

/* --- WBC 任务权重 --- */
DTCM_DATA static float kWbcWeightBody[6]   = {10.f, 10.f, 10.f, 1.f, 1.f, 1.f};
DTCM_DATA static float kWbcWeightContact[12] = {
    1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f
};


/* ================================================================
 * 第三部分：ITCM_FUNC —— 高频热路径函数
 * ================================================================
 * 规则：
 *  1. 宏只加在 .c 文件的函数【定义】处，头文件声明不加
 *  2. 不能是 static inline（去掉inline改为普通函数）
 *  3. 函数内部可以正常调用其他函数（包括非ITCM函数）
 *  4. ITCM 函数访问的数据建议也放 DTCM，否则仍有总线延迟
 * ================================================================ */

/* --- 五连杆正运动学（500Hz，放ITCM） --- */
/**
 * @brief 五连杆闭链正运动学：由关节角计算末端位置
 * @param q     关节角数组 [theta1, theta2] (rad)
 * @param pos   输出末端位置 [x, z] (m)
 */
ITCM_FUNC void FiveLink_ForwardKinematics(const float *q, float *pos)
{
    // 示例：平面五连杆（两主动关节 theta1, theta2）
    float s1 = arm_sin_f32(q[0]);
    float c1 = arm_cos_f32(q[0]);
    float s2 = arm_sin_f32(q[1]);
    float c2 = arm_cos_f32(q[1]);

    float x1 = kFiveLinkLen[1] * c1;
    float z1 = kFiveLinkLen[1] * s1;
    float x2 = kFiveLinkLen[2] * c2;
    float z2 = kFiveLinkLen[2] * s2;

    pos[0] = x1 + x2;  // 末端 x
    pos[1] = z1 + z2;  // 末端 z
}

/* --- 五连杆逆运动学（Newton-Raphson迭代，500Hz，放ITCM） --- */
/**
 * @brief 五连杆闭链逆运动学
 * @param pos_des   期望末端位置 [x, z] (m)
 * @param q_init    初始猜测关节角（上一拍的解），就地更新为新解
 * @param max_iter  最大迭代次数（建议3~5次即可收敛）
 */
ITCM_FUNC void FiveLink_InverseKinematics(const float *pos_des, float *q_init, int max_iter)
{
    float q[2] = {q_init[0], q_init[1]};
    float f[2], J[4], J_inv[4];
    float dq[2];

    for (int iter = 0; iter < max_iter; iter++)
    {
        /* 1. 计算当前正运动学残差 f = FK(q) - pos_des */
        float pos[2];
        FiveLink_ForwardKinematics(q, pos);
        f[0] = pos[0] - pos_des[0];
        f[1] = pos[1] - pos_des[1];

        /* 2. 收敛判断 */
        if (f[0]*f[0] + f[1]*f[1] < 1e-8f) break;

        /* 3. 计算雅可比（解析式，避免数值差分）*/
        float s1 = arm_sin_f32(q[0]), c1 = arm_cos_f32(q[0]);
        float s2 = arm_sin_f32(q[1]), c2 = arm_cos_f32(q[1]);
        J[0] = -kFiveLinkLen[1] * s1;  // df_x/dq1
        J[1] = -kFiveLinkLen[2] * s2;  // df_x/dq2
        J[2] =  kFiveLinkLen[1] * c1;  // df_z/dq1
        J[3] =  kFiveLinkLen[2] * c2;  // df_z/dq2

        /* 4. 2×2 矩阵解析求逆（比调库快）*/
        float det = J[0]*J[3] - J[1]*J[2];
        if (det < 1e-6f && det > -1e-6f) break; // 奇异判断
        float inv_det = 1.0f / det;
        J_inv[0] =  J[3] * inv_det;
        J_inv[1] = -J[1] * inv_det;
        J_inv[2] = -J[2] * inv_det;
        J_inv[3] =  J[0] * inv_det;

        /* 5. Newton-Raphson 更新 dq = -J^{-1} * f */
        dq[0] = -(J_inv[0]*f[0] + J_inv[1]*f[1]);
        dq[1] = -(J_inv[2]*f[0] + J_inv[3]*f[1]);
        q[0] += dq[0];
        q[1] += dq[1];
    }

    q_init[0] = q[0];
    q_init[1] = q[1];
}

/* --- WBC 雅可比更新（500Hz，放ITCM） --- */
/**
 * @brief 根据当前关节角更新接触雅可比矩阵
 * @param q     全身关节角 (10维)
 * @param J_out 输出雅可比 [6×12]（前向声明使用 DTCM 中的 g_J_wbc）
 */
ITCM_FUNC void WBC_JacobianUpdate(const float *q, float J_out[][12])
{
    // 实际项目中根据你的机器人运动学展开解析雅可比
    // 此处为框架示意：清零后按各腿独立填写
    for (int leg = 0; leg < 5; leg++)
    {
        float c1 = arm_cos_f32(q[leg*2]);
        float s1 = arm_sin_f32(q[leg*2]);
        float c2 = arm_cos_f32(q[leg*2 + 1]);
        float s2 = arm_sin_f32(q[leg*2 + 1]);

        // x方向偏导
        J_out[leg][leg*2]   = -kFiveLinkLen[1]*s1;
        J_out[leg][leg*2+1] = -kFiveLinkLen[2]*s2;
        // z方向偏导（行 leg+5 为z方向）
        J_out[leg+5][leg*2]   =  kFiveLinkLen[1]*c1;  // 注意:此处仅示意,实际需匹配你的状态维度
        J_out[leg+5][leg*2+1] =  kFiveLinkLen[2]*c2;
        (void)c2; (void)s2;  // 消除unused warning
    }
}

/* --- WBC 力矩分配（500Hz，放ITCM） --- */
/**
 * @brief 全身控制力矩分配：tau = J^T * F_contact
 * @param J         接触雅可比 [6×12]
 * @param F_contact 接触力向量 [6]
 * @param tau_out   输出关节力矩 [12]
 */
ITCM_FUNC void WBC_TorqueAllocation(const float J[][12],
                                    const float *F_contact,
                                    float *tau_out)
{
    // tau = J^T * F，J为6×12，J^T为12×6
    for (int j = 0; j < 12; j++)
    {
        tau_out[j] = 0.0f;
        for (int i = 0; i < 6; i++)
            tau_out[j] += J[i][j] * F_contact[i];
    }
}

/* --- MPC 单步预测（100Hz，放ITCM，可选） --- */
/**
 * @brief MPC线性化模型单步状态预测：x_{k+1} = A*x_k + B*u_k
 * @param A     离散系统矩阵 [STATE_DIM × STATE_DIM]
 * @param B     输入矩阵    [STATE_DIM × INPUT_DIM]
 * @param x     当前状态
 * @param u     当前输入
 * @param x_next 输出预测状态
 */
ITCM_FUNC void MPC_PredictStep(const float *A, const float *B,
                                const float *x, const float *u,
                                float *x_next)
{
    // x_next = A*x + B*u，展开矩阵-向量乘法（避免CMSIS-DSP动态内存）
    for (int i = 0; i < MPC_STATE_DIM; i++)
    {
        float sum = 0.0f;
        for (int j = 0; j < MPC_STATE_DIM; j++)
            sum += A[i * MPC_STATE_DIM + j] * x[j];
        for (int j = 0; j < MPC_INPUT_DIM; j++)
            sum += B[i * MPC_INPUT_DIM + j] * u[j];
        x_next[i] = sum;
    }
}


/* ================================================================
 * 第四部分：如何在任务里调用
 * ================================================================ */

/*
// --- wbc_task.c（500Hz） ---
#include "tcm_region.h"

// DTCM_BSS 全局变量已在顶部声明（g_J_wbc、g_tau_wbc 等）

void WBC_Task(void)
{
    // 1. 更新雅可比（ITCM函数 + DTCM数据 = 最快路径）
    WBC_JacobianUpdate(g_q_five_link, g_J_wbc);

    // 2. 逆运动学求解（ITCM函数）
    static float q_ik[2] = {0};
    float pos_des[2] = {0.0f, -0.35f};
    FiveLink_InverseKinematics(pos_des, q_ik, 4);

    // 3. 力矩分配
    float F_contact[6] = {0};  // 由MPC给出
    WBC_TorqueAllocation((const float(*)[12])g_J_wbc, F_contact, g_tau_wbc);
}

// --- mpc_task.c（100Hz） ---
void MPC_Task(void)
{
    // 离散系统矩阵A/B放DTCM_BSS
    MPC_PredictStep(
        (float*)g_mpc_A_bar,
        (float*)g_mpc_B_bar,
        g_mpc_x,
        g_mpc_u_opt,
        g_mpc_x   // 就地更新
    );
}
*/


/* ================================================================
 * 第五部分：验证放置是否成功（编译后在.map文件检查）
 * ================================================================
 *
 * 编译后打开 build/Basic_Framework_MC02.map，搜索函数名：
 *
 *   搜索 "WBC_JacobianUpdate"，地址应为 0x00000xxx  ← ITCM ✅
 *   搜索 "g_J_wbc"，            地址应为 0x20000xxx  ← DTCM ✅
 *   搜索 "kFiveLinkLen"，       地址应为 0x20000xxx  ← DTCM ✅
 *
 *   若地址为 0x08xxxxxx → 仍在 FLASH（函数）或
 *            0x24xxxxxx → 仍在 AXI SRAM（数据）
 *   说明宏未生效，检查是否漏加或用了 inline
 *
 * ================================================================ */
