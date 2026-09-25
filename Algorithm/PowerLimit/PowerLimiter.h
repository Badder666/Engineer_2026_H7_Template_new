//
// Created by Pazfic on 2025/3/12.
//

#ifndef DHSentry_POWER_LIMITATOR_H
#define DHSentry_POWER_LIMITATOR_H

#include "Struct_Typedef.h"
#include "Torque.h"

typedef enum {
    MODEL_3508 = 0,
    MODEL_6020,
}Model_Type;

// 功率限制器
typedef struct PowerLimiter {
    fp32       m[4];            // 模型系数
    fp32       k;               // 放缩系数
    Model_Type type;            // 电机模型类型

    fp32    speedFeedback;      // 转速反馈
    fp32    torqueCmd;          // 力矩输入
    fp32    absSpeedError;      // 转速2误差绝对值
    fp32    predictPower;       // 模型正解得出预测功率

    uint8_t limitFlag;          // 是否限制功率
    fp32    assignedPower;      // 被分配的允许功率
    fp32    torqueRoot[2];      // 由模型逆解得出的当前允许功率下的最大输出力矩
    int16_t absMaxMessage;      // 最大允许输出报文
}Limiter_t;

// 功率限制器句柄指针
typedef struct PowerLimiter* Limiter_Handle;

// 功率限制器调度
typedef struct PowerLimiterScheduler {
    Limiter_Handle*  list;              // 限制器列表
    uint8_t          motorCnt;          // 电机数量

    int16_t          powerLimit;         // 功率上限
    fp32             totalAbsError;      // 单刻误差绝对值总和
    fp32             totalPredictPower;  // 单刻预测功率总和

    fp32             E_upper;            // 转速差总和上界
    fp32             E_lower;            // 转速差总和下界
    fp32             K_coe;              // 置信度

}LimiterScheduler_t;

// 功率限制器调度句柄指针
typedef struct PowerLimiterScheduler* LimiterScheduler_Handle;

/**
 * @brief 初始化功率限制器调度
 * @param _scheduler 调度句柄
 * @param _list      调度器列表
 * @param _upper     转速差总和上界
 * @param _lower     转速差总和下界
 */
void powerInitialiseLimiterScheduler(LimiterScheduler_Handle _scheduler, uint8_t _cnt, Limiter_Handle *_list, fp32 _upper, fp32 _lower);

/**
 * @brief 底盘总功率更新
 * @param[in] _scheduler  调度句柄
 * @param[in] _powerLimit 功率上限
 */
void powerSchedulerUpdate(LimiterScheduler_Handle _scheduler, int16_t _powerLimit);

/**
 * @brief 初始化功率限制器
 * @param[in] _limiter      限制器句柄指针
 * @param[in] _type         限制器对应电机类型
 */
void powerInitialiseLimiter(Limiter_Handle _limiter, Model_Type _type);

/**
 * @brief 限制器更新限制器输出
 * @param _limiter    限制器句柄指针
 * @param _rpm        转速反馈
 * @param _message    报文输入
 * @param _absError   控制器单刻误差绝对值
 */
void powerLimiterUpdate(Limiter_Handle _limiter, int16_t _rpm, int16_t _message, fp32 _error);

/**
 * @brief 获取限制器输出
 * @param _limiter      限制器句柄指针
 * @param _message      报文输入
 * @return    限制器输出(报文)
 */
int16_t powerGetLimiterUpdate(Limiter_Handle _limiter, int16_t _message);


#endif
