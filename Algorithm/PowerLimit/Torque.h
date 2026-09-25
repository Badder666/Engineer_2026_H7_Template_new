//
// Created by Pazfic on 2025/3/15.
//

#ifndef SENTRY_TORQUE_H
#define SENTRY_TORQUE_H

// 通讯报文转换为实际电流的系数
#define MESSAGE2CURRENT_3508 0.001220703125f        // ±16384->±20A
#define MESSAGE2CURRENT_6020 0.00018310546875f      // ±16384->±3A
// 电流转换为通讯报文的系数
#define CURRENT2MESSAGE_3508 819.2f
#define CURRENT2MESSAGE_6020 5461.333f
// 报文转换为力矩的系数
#define MESSAGE2TORQUE_3508 0.0003662109375f
#define MESSAGE2TORQUE_6020 0.00013568115234375f
// 力矩转换为报文的系数
#define TORQUE2MESSAGE_3508 2730.66f
#define TORQUE2MESSAHE_6020 7370.22f

// 3508报文转换为力矩
#define torqueMessageTransform3508Inv(_message) (_message * MESSAGE2TORQUE_3508)
// 6020报文转换为力矩
#define torqueMessageTransform6020Inv(_message) (_message * MESSAGE2TORQUE_6020)
// 3508力矩转换为报文
#define torqueMessageTransform3508(_torque) (_torque * TORQUE2MESSAGE_3508)
// 6020力矩转换为报文
#define torqueMessageTransform6020(_torque) (_torque * TORQUE2MESSAHE_6020)

#endif
