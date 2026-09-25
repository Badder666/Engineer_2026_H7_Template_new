/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : GravityCompensation.c
  * @brief          : 6-DOF 机械臂重力补偿前馈力矩 (移植自 Python 工程
  *                   robot_gravity_compensation.py 中的 RobotModel).
  *
  *                   坐标变换链使用 CMSIS-DSP arm_mat_mult_f32 完成 4x4
  *                   矩阵乘法; 叉乘/点乘等小尺寸运算直接展开以提速.
  *
  *                   单位:
  *                     - 角度: rad
  *                     - 长度: m
  *                     - 质量: kg
  *                     - 力矩: N·m
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "GravityCompensation.h"

/* Private macros ------------------------------------------------------------*/
#define DEG2RAD(x)   ((float)((x) * (PI / 180.0f)))

/* Public variables ----------------------------------------------------------*/
GravityComp_Info_TypeDef gGravityComp;

/* Private function prototypes -----------------------------------------------*/
static void DH_Transform(float a, float alpha, float d, float theta, float T_out[16]);
static void Forward_Kinematics(const float theta[GRAV_COMP_DOF],
                               float Ts[GRAV_COMP_DOF + 1][16]);

/* ===========================================================================
 * 初始化: 与 Python 工程参数完全一致
 * =========================================================================*/
void GravityComp_Init(void)
{
    /* ---- 改良(经典)DH 参数 ---- */
    gGravityComp.alpha[0] = DEG2RAD(0.0f);
    gGravityComp.alpha[1] = DEG2RAD(90.0f);
    gGravityComp.alpha[2] = DEG2RAD(0.0f);
    gGravityComp.alpha[3] = DEG2RAD(90.0f);
    gGravityComp.alpha[4] = DEG2RAD(90.0f);
    gGravityComp.alpha[5] = DEG2RAD(-90.0f);

    gGravityComp.a[0] = 0.0f;
    gGravityComp.a[1] = 0.0f;
    gGravityComp.a[2] = 0.322f;
    gGravityComp.a[3] = 0.0f;
    gGravityComp.a[4] = 0.0f;
    gGravityComp.a[5] = 0.0f;

    gGravityComp.d[0] = 0.127f;   /*!< 基座到 J2 (肩) 的高度 */
    gGravityComp.d[1] = 0.0f;
    gGravityComp.d[2] = 0.0f;
    gGravityComp.d[3] = 0.266f;   /*!< 前臂长度 (J3 -> J4 沿 J3 的 -y) */
    gGravityComp.d[4] = 0.0f;
    gGravityComp.d[5] = 0.081f;   /*!< 末端法兰到工具中心 */

    gGravityComp.theta_offset[0] = DEG2RAD(90.0f);
    gGravityComp.theta_offset[1] = DEG2RAD(0.0f);
    gGravityComp.theta_offset[2] = DEG2RAD(90.0f);
    gGravityComp.theta_offset[3] = DEG2RAD(0.0f);
    gGravityComp.theta_offset[4] = DEG2RAD(0.0f);
    gGravityComp.theta_offset[5] = DEG2RAD(0.0f);

    /* ---- 各连杆质心 (父系下, m) ----
     * 与 Python: self.com_local 完全一致
     */
    /* 连杆1 (父系=世界) */
    gGravityComp.com_local[0][0] = 0.0f;
    gGravityComp.com_local[0][1] = 0.0f;
    gGravityComp.com_local[0][2] = 0.0f;
    /* 连杆2 (父系=关节1) */
    gGravityComp.com_local[1][0] = 0.0f;
    gGravityComp.com_local[1][1] = 0.0f;
    gGravityComp.com_local[1][2] = 0.0f;
    /* 连杆3 (父系=关节2) */
    gGravityComp.com_local[2][0] = 0.114f;
    gGravityComp.com_local[2][1] = 0.0f;
    gGravityComp.com_local[2][2] = 0.0f;
    /* 连杆4 (父系=关节3) */
    gGravityComp.com_local[3][0] = 0.0f;
    gGravityComp.com_local[3][1] = -0.122f;
    gGravityComp.com_local[3][2] = 0.0f;
    /* 连杆5 (父系=关节4) */
    gGravityComp.com_local[4][0] = 0.0f;
    gGravityComp.com_local[4][1] = 0.0f;
    gGravityComp.com_local[4][2] = -0.0147f;
    /* 连杆6 (父系=关节5) */
    gGravityComp.com_local[5][0] = 0.0f;
    gGravityComp.com_local[5][1] = 0.1767f;
    gGravityComp.com_local[5][2] = 0.0f;

    /* ---- 各连杆质量 (kg) ---- */
    gGravityComp.masses[0] = 3.092f;
    gGravityComp.masses[1] = 1.333f;
    gGravityComp.masses[2] = 0.955f;
    gGravityComp.masses[3] = 1.631f;
    gGravityComp.masses[4] = 0.239f;
    gGravityComp.masses[5] = 0.746f;

    /* ---- 重力向量 (世界系, m/s^2) ----
     * Python 中使用 g = -9.79; Config.h 提供 GravityAccel = 9.7880, 这里直接对齐 Python.
     */
    gGravityComp.gravity[0] = 0.0f;
    gGravityComp.gravity[1] = 0.0f;
    gGravityComp.gravity[2] = -9.7880f;

    /* ---- 运行时缓冲清零 ---- */
    memset(gGravityComp.current_theta, 0, sizeof(gGravityComp.current_theta));
    memset(gGravityComp.feedforward_torque, 0, sizeof(gGravityComp.feedforward_torque));
}

/* ===========================================================================
 * 计算重力补偿
 *
 *   τ[joint] = -Σ_link ((r_link - o_joint) × (m_link · g)) · z_joint
 *
 * 与 Python 一致: 计算完成后再循环左移 1 位, 使结果索引与关节 1..6 对齐.
 * =========================================================================*/
void GravityComp_Compute(const float theta[GRAV_COMP_DOF],
                         float torques_out[GRAV_COMP_DOF])
{
    /* 0. 缓存输入并执行正运动学 */
    memcpy(gGravityComp.current_theta, theta, sizeof(gGravityComp.current_theta));

    float Ts[GRAV_COMP_DOF + 1][16];
    Forward_Kinematics(theta, Ts);

    /* 1. 计算各连杆质心在世界坐标系下的位置 */
    float com_world[GRAV_COMP_DOF][3];
    for (uint8_t i = 0U; i < GRAV_COMP_DOF; i++)
    {
        const float *T = Ts[i];                 /*!< 父坐标系: Ts[i] */
        const float x = gGravityComp.com_local[i][0];
        const float y = gGravityComp.com_local[i][1];
        const float z = gGravityComp.com_local[i][2];

        com_world[i][0] = T[0] * x + T[1] * y + T[2]  * z + T[3];
        com_world[i][1] = T[4] * x + T[5] * y + T[6]  * z + T[7];
        com_world[i][2] = T[8] * x + T[9] * y + T[10] * z + T[11];
    }

    /* 2. 累计每个关节受到的重力力矩 */
    float torques[GRAV_COMP_DOF] = {0.0f};

    for (uint8_t link_idx = 0U; link_idx < GRAV_COMP_DOF; link_idx++)
    {
        const float Fg[3] = {
            gGravityComp.masses[link_idx] * gGravityComp.gravity[0],
            gGravityComp.masses[link_idx] * gGravityComp.gravity[1],
            gGravityComp.masses[link_idx] * gGravityComp.gravity[2]
        };
        const float *com = com_world[link_idx];

        for (uint8_t joint_idx = 0U; joint_idx <= link_idx; joint_idx++)
        {
            float zaxis[3];
            float origin[3];

            if (joint_idx == 0U)
            {
                /* 关节1对应世界系 z 轴与原点 */
                zaxis[0] = 0.0f; zaxis[1] = 0.0f; zaxis[2] = 1.0f;
                origin[0] = 0.0f; origin[1] = 0.0f; origin[2] = 0.0f;
            }
            else
            {
                const float *T = Ts[joint_idx];
                /* 4x4 行主序: 第三列(z 轴)分量在 T[2], T[6], T[10] */
                zaxis[0]  = T[2];
                zaxis[1]  = T[6];
                zaxis[2]  = T[10];
                origin[0] = T[3];
                origin[1] = T[7];
                origin[2] = T[11];
            }

            const float rx = com[0] - origin[0];
            const float ry = com[1] - origin[1];
            const float rz = com[2] - origin[2];

            /* cross(r, F_g) */
            const float cx = ry * Fg[2] - rz * Fg[1];
            const float cy = rz * Fg[0] - rx * Fg[2];
            const float cz = rx * Fg[1] - ry * Fg[0];

            /* contribution = -(cross · z_axis) */
            const float contribution = -(cx * zaxis[0] + cy * zaxis[1] + cz * zaxis[2]);
            torques[joint_idx] += contribution;
        }
    }

    /* 3. roll(-1): 与 Python 对齐, 使 [τ1..τ6] 与关节 1..6 对应 */
    for (uint8_t i = 0U; i < GRAV_COMP_DOF; i++)
    {
        const uint8_t src = (uint8_t)((i + 1U) % GRAV_COMP_DOF);
        torques_out[i] = torques[src];
        gGravityComp.feedforward_torque[i] = torques[src];
    }
}

/* ===========================================================================
 * 仅刷新全局前馈数组
 * =========================================================================*/
void GravityComp_UpdateFeedforward(const float theta[GRAV_COMP_DOF])
{
    GravityComp_Compute(theta, gGravityComp.feedforward_torque);
}

const float *GravityComp_GetFeedforward(void)
{
    return (const float *)gGravityComp.feedforward_torque;
}

/* ===========================================================================
 * 模型参数运行时修改接口
 * =========================================================================*/
void GravityComp_SetMass(uint8_t link_idx, float mass)
{
    if (link_idx >= GRAV_COMP_DOF) { return; }
    gGravityComp.masses[link_idx] = mass;
}

void GravityComp_SetCoM(uint8_t link_idx, float x, float y, float z)
{
    if (link_idx >= GRAV_COMP_DOF) { return; }
    gGravityComp.com_local[link_idx][0] = x;
    gGravityComp.com_local[link_idx][1] = y;
    gGravityComp.com_local[link_idx][2] = z;
}

void GravityComp_SetGravity(float gx, float gy, float gz)
{
    gGravityComp.gravity[0] = gx;
    gGravityComp.gravity[1] = gy;
    gGravityComp.gravity[2] = gz;
}

/* ===========================================================================
 * Private functions
 * =========================================================================*/

/**
  * @brief  经典 DH 变换: 与 Python 中 RobotModel.dh_transform 完全一致.
  *         T = | ct      -st      0      a   |
  *             | st·ca   ct·ca   -sa   -d·sa |
  *             | st·sa   ct·sa    ca    d·ca |
  *             | 0       0        0     1    |
  * @param  T_out  16 个 float, 行主序 4x4 矩阵.
  */
static void DH_Transform(float a, float alpha, float d, float theta, float T_out[16])
{
    const float ct = arm_cos_f32(theta);
    const float st = arm_sin_f32(theta);
    const float ca = arm_cos_f32(alpha);
    const float sa = arm_sin_f32(alpha);

    T_out[0]  = ct;        T_out[1]  = -st;       T_out[2]  = 0.0f;   T_out[3]  = a;
    T_out[4]  = st * ca;   T_out[5]  = ct * ca;   T_out[6]  = -sa;    T_out[7]  = -d * sa;
    T_out[8]  = st * sa;   T_out[9]  = ct * sa;   T_out[10] = ca;     T_out[11] = d * ca;
    T_out[12] = 0.0f;      T_out[13] = 0.0f;      T_out[14] = 0.0f;   T_out[15] = 1.0f;
}

/**
  * @brief  正运动学链: 计算 Ts[0..6], Ts[i] 表示关节 i 坐标系到世界系的变换.
  *         Ts[0] = I, Ts[i] = Ts[i-1] · DH(i).
  *         矩阵乘法使用 CMSIS-DSP arm_mat_mult_f32.
  */
static void Forward_Kinematics(const float theta[GRAV_COMP_DOF],
                               float Ts[GRAV_COMP_DOF + 1][16])
{
    /* Ts[0] = identity */
    memset(Ts[0], 0, sizeof(float) * 16);
    Ts[0][0]  = 1.0f;
    Ts[0][5]  = 1.0f;
    Ts[0][10] = 1.0f;
    Ts[0][15] = 1.0f;

    float Ti[16];
    arm_matrix_instance_f32 mPrev;
    arm_matrix_instance_f32 mTi;
    arm_matrix_instance_f32 mNext;

    for (uint8_t i = 0U; i < GRAV_COMP_DOF; i++)
    {
        const float th = theta[i] + gGravityComp.theta_offset[i];

        DH_Transform(gGravityComp.a[i],
                     gGravityComp.alpha[i],
                     gGravityComp.d[i],
                     th,
                     Ti);

        arm_mat_init_f32(&mPrev, 4, 4, Ts[i]);
        arm_mat_init_f32(&mTi,   4, 4, Ti);
        arm_mat_init_f32(&mNext, 4, 4, Ts[i + 1U]);

        (void)arm_mat_mult_f32(&mPrev, &mTi, &mNext);
    }
}
