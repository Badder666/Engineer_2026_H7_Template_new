//
// Created by Pazfic on 2025/3/12.
//

#include "PowerLimiter.h"
#include "ModelConfig.h"
#include "stdlib.h"
#include "math.h"

/**
 * @brief 通过模型正解得出预测功率
 * @param _limiter
 * @return
 */
static fp32 modelCalc_Predict_Power(Limiter_Handle _limiter) {
    fp32 M0 = _limiter->m[0] * _limiter->speedFeedback * _limiter->torqueCmd;
    fp32 M1 = _limiter->m[1] * fabsf(_limiter->speedFeedback);
    fp32 M2 = _limiter->m[2] * _limiter->torqueCmd * _limiter->torqueCmd;
    fp32 P_predict = M0 + M1 + M2 + _limiter->m[3];
    _limiter->predictPower = P_predict;
    return P_predict;
}


/**
 * @brief 计算置信度
 * @param _scheduler 调度器句柄
 * @return
 */
static fp32 powerCalc_K_coe(LimiterScheduler_Handle _scheduler) {
    // 计算置信区间分布
    if(_scheduler->totalAbsError >= _scheduler->E_upper) {
        _scheduler->K_coe = 1.0f;
    } else if(_scheduler->totalAbsError <= _scheduler->E_lower) {
        _scheduler->K_coe = 0.0f;
    } else {
        _scheduler->K_coe = (_scheduler->totalAbsError - _scheduler->E_lower) / (_scheduler->E_upper - _scheduler->E_lower);
    }
    return _scheduler->K_coe;
}

/**
 * @brief 重新分配功率
 * @param _limiter
 * @param _totalAbsError
 * @param _totalPredictPower
 * @param _K_coe
 * @return
 */
fp32 K_ = 0.0f;
static fp32 powerRearrangement(Limiter_Handle _limiter, fp32 _totalAbsError, fp32 _totalPredictPower, fp32 _coe, fp32 _maxPower) {
    fp32 K = 0.0f;
    
    // 避免除零
    if(_totalAbsError > 1e-6f && _totalPredictPower > 1e-6f) {
        K = _coe * (_limiter->absSpeedError / _totalAbsError) + (1 - _coe) * (_limiter->predictPower / _totalPredictPower);
    } else if(_totalPredictPower > 1e-6f) {
        K = (_limiter->predictPower / _totalPredictPower);
    } else if(_totalAbsError > 1e-6f) {
        K = (_limiter->absSpeedError / _totalAbsError);
    } else {
        K = 1.0f / 4.0f;
    }
    K_ = K;

    if(K > 1.0f) K = 1.0f;
    if(K < 0.0f) K = 0.0f;
    
    _limiter->k = K;
    _limiter->assignedPower = K * _maxPower;
    return _limiter->assignedPower;
}

/**
 * @brief 模型逆解得到两个根
 * @param _limiter
 */
static void modelCalc_Root(Limiter_Handle _limiter) {
    fp32 A = _limiter->m[1] * fabsf(_limiter->speedFeedback) + _limiter->m[3] - _limiter->assignedPower;
    fp32 delta = _limiter->m[0]*_limiter->m[0]*_limiter->speedFeedback*_limiter->speedFeedback - 4 * A * _limiter->m[2];
    // 计算根
    if(delta > 0) {
        _limiter->torqueRoot[0] = (-(_limiter->m[0]*_limiter->speedFeedback) + sqrtf(delta)) / (2*_limiter->m[2]);
        _limiter->torqueRoot[1] = (-(_limiter->m[0]*_limiter->speedFeedback) - sqrtf(delta)) / (2*_limiter->m[2]);
    } else {
        _limiter->torqueRoot[0] = _limiter->torqueRoot[1] = (-(_limiter->m[0]*_limiter->speedFeedback)) / (2*_limiter->m[2]);
    }
}

/**
 * @brief 初始化功率限制器调度
 * @param _scheduler 调度句柄
 * @param _cnt       调度的电机数量
 * @param _list      调度器列表
 * @param _upper     转速差总和上界
 * @param _lower     转速差总和下界
 */
void powerInitialiseLimiterScheduler(LimiterScheduler_Handle _scheduler, uint8_t _cnt, Limiter_Handle* _list, fp32 _upper, fp32 _lower) {
    // 避免调度器列表为空
    if(_list == NULL) return;
    _scheduler->list = _list;

    _scheduler->motorCnt = _cnt;
    _scheduler->E_upper = _upper;
    _scheduler->E_lower = _lower;

    _scheduler->totalAbsError = 0;
    _scheduler->totalPredictPower = 0;
    _scheduler->K_coe = 0;
}

/**
 * @brief 初始化功率限制器
 * @param[in] _limitator    限制器句柄指针
 * @param[in] _type         限制器对应电机类型
 */
void powerInitialiseLimiter(Limiter_Handle _limiter, Model_Type _type) {
    _limiter->type = _type;
    // 参数赋值
    if(_type == MODEL_3508) {
        _limiter->m[0] = m_0_3508;
        _limiter->m[1] = m_1_3508;
        _limiter->m[2] = m_2_3508;
        _limiter->m[3] = m_3_3508;
    // } else if(_type == MODEL_6020) {
    //     _limiter->m[0] = m_0_6020;
    //     _limiter->m[1] = m_1_6020;
    //     _limiter->m[2] = m_2_6020;
    //     _limiter->m[3] = m_3_6020;
    }
    _limiter->torqueRoot[0] = _limiter->torqueRoot[1] = 0.0f;
    _limiter->torqueCmd = 0;
    _limiter->speedFeedback = 0;
    _limiter->absSpeedError = 0;
    _limiter->assignedPower = 0;
    _limiter->predictPower = 0;
    _limiter->limitFlag = 0;
    _limiter->absMaxMessage = 0;
}

/**
 * @brief 更新限制器反馈量，并通过模型正向解出预测功率
 * @param _limiter    限制器句柄指针
 * @param _rpm        转速反馈
 * @param _message    报文输入
 * @param _error      控制器单刻误差
 */
void powerLimiterUpdate(Limiter_Handle _limiter, int16_t _rpm, int16_t _message, fp32 _error) {
    _limiter->speedFeedback = _rpm;

    if(_limiter->type == MODEL_3508) {
        _limiter->torqueCmd = torqueMessageTransform3508Inv(_message);
    } else if(_limiter->type == MODEL_6020) {
        _limiter->torqueCmd = torqueMessageTransform6020Inv(_message);
    }

    _limiter->absSpeedError = fabsf(_error);
    _limiter->predictPower = modelCalc_Predict_Power(_limiter);
}

/**
 * @brief 底盘总功率更新
 * @param[in] _scheduler  调度句柄
 * @param[in] _powerLimit 功率上限
 * @param[in] _isCapOn    电容开启标志位
 */
void powerSchedulerUpdate(LimiterScheduler_Handle _scheduler, int16_t _powerLimit) {
    if(_powerLimit == -1) {
        // 当功率限制为unlimited时，设置为120W
        _scheduler->powerLimit = 120;
    } else {
        _scheduler->powerLimit = _powerLimit;
    }

    _scheduler->totalAbsError  = 0;
    _scheduler->totalPredictPower = 0;

    for(int i = 0; i < _scheduler->motorCnt; ++i) {
        _scheduler->totalAbsError += _scheduler->list[i]->absSpeedError;
        _scheduler->totalPredictPower += _scheduler->list[i]->predictPower;
    }

    // 计算置信度
    powerCalc_K_coe(_scheduler);

    for(int i = 0; i < _scheduler->motorCnt; ++i) {
        // 只要功率上限被设置（>0），就应该做功率限制
        if(_scheduler->powerLimit > 0) {
            _scheduler->list[i]->limitFlag = 1;
            if(_scheduler->totalPredictPower >= (fp32)_powerLimit) {
                // 如果预测功率超出了功率上限，重新分配每个电机的功率
                powerRearrangement(_scheduler->list[i], _scheduler->totalAbsError, _scheduler->totalPredictPower,
                                   _scheduler->K_coe, _scheduler->powerLimit);
            } else {
                // 预测功率没超标，但仍然需要限制每个电机，防止个别电机超功率
                // 按预测功率比例分配
                if(_scheduler->totalPredictPower > 1e-6f) {
                    _scheduler->list[i]->assignedPower = _scheduler->powerLimit *
                        (_scheduler->list[i]->predictPower / _scheduler->totalPredictPower);
                    _scheduler->list[i]->assignedPower = _scheduler->list[i]->predictPower;
                } else {
                    // 预测功率为0，均匀分配
                    _scheduler->list[i]->assignedPower = _scheduler->powerLimit / _scheduler->motorCnt;
                }
            }
        } else {
            // 功率限制为0，不做限制
            _scheduler->list[i]->limitFlag = 0;
        }
    }
}

/**
 * @brief 获取限制器输出
 * @param[in] _limiter      限制器句柄指针
 * @param[in] _message      报文输入
 * @return    限制器输出(报文)
 */
int16_t powerGetLimiterUpdate(Limiter_Handle _limiter, int16_t _message) {
    int16_t result = 0;
    // 如果单刻下需要对功率控制，进入计算环节
    if(_limiter->limitFlag == 1) {
        fp32 Root = 0.0f;
        int16_t maxMessage = 0;
        // 计算力矩输出
        modelCalc_Root(_limiter);
        // 根据输入报文符号选择根
        if (_limiter->torqueRoot[0] == _limiter->torqueRoot[1]) {
            Root = _limiter->torqueRoot[0];
        } else {
            if (_message >= 0) {
                Root = _limiter->torqueRoot[0];
            } else if (_message < 0) {
                Root = _limiter->torqueRoot[1];
            }
        }

        if (_limiter->type == MODEL_3508)
            maxMessage = torqueMessageTransform3508(Root);
        else if (_limiter->type == MODEL_6020)
            maxMessage = torqueMessageTransform6020(Root);

        if (maxMessage > 16384) {
            maxMessage = 16384;
        } else if (maxMessage < -16384) {
            maxMessage = -16384;
        }

        _limiter->absMaxMessage = abs(maxMessage);

        // 如果输入的报文宽度超过了设定的报文宽度，对报文限幅
        if (abs(_message) > abs(_limiter->absMaxMessage)) {
            if (_message < 0) result = -(_limiter->absMaxMessage);
            else if (_message > 0) result = _limiter->absMaxMessage;
        } else {
            result = _message;
        }
    } else {
        // 否则不对功率做限制
        result = _message;
    }

    return result;
}
