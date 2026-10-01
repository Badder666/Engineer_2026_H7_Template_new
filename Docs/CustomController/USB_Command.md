# 新自定义控制器 USB 接收

小电脑源码位于同级 `MiniPC/New_Custom_Controller`。自定义控制器发 42 字节状态帧，小电脑映射后向本固件发 64 字节命令帧。完整字段表见 `MiniPC/New_Custom_Controller/docs/usb_command_protocol.md`。

## 接收与控制路径

```text
USB CDC 回调 → System/usb_comm 环形缓冲
             → Detect_Task 中 MiniPC_Receive_Info
             → Device/MiniPC/custom_controller_protocol 字节流解析
             → MiniPC_Data → 既有关节换算、目标斜坡及 CAN 电机控制
```

新帧为小端序：帧头 `A5 5A`，版本 `1`，类型 `0x10`，总长度 `64`，CRC16 覆盖前 62 字节；算法是初值 `0xffff`、反射多项式 `0x8408`、最终异或 0。

解析器同时兼容旧的 60 字节 `AA...55` 累加校验命令。不能假设一次 USB 回调等于一帧，支持拆包、粘包、噪声与破损帧后的重同步。1024 字节缓冲溢出可通过 `USBComm_GetDroppedBytes()` 观察；溢出和 USB 会话重启会重置半帧。

## 调试字段

`MiniPC_Data.joint_pos_data` 是机械臂关节目标，单位为弧度，沿用原有下位机的关节到电机换算。只有命令有效位为 1 的轴会更新，其余保留上一目标。

新增字段：

- `controller_buttons`：完整 16 位控制器按键。
- `valid_joint_mask`：最近有效命令的六轴有效位。
- `command_sequence`：上位机序号。
- `controller_time_ms`：控制器采集时间。

键鼠、摇杆和面板字段映射到既有字段，`btn1..btn4` 从 `handle_buttons` bit 0..3 拆出。控制器原始按键到这些动作字段的源位默认全部为 `-1`，硬件确定后在上位机配置里调整。例如把 `handle_button_bits[3]` 设为 11，会将控制器 bit 11 接到既有 `btn4` 夹爪动作。

命令帧校验失败或含 NaN/Inf 时整帧拒绝，不刷新在线计数。操作有效标志为 0 时，键鼠与面板清零，摇杆回中位；六轴目标仍按独立有效位更新。

原有 `MiniPC_Offline_Monitor()` 在线状态行为保持；没有新增 500 ms 无帧关闭电机逻辑。USB 实际位置返回仍用旧 27 字节反馈帧，发送忙时不会覆盖正在传输的缓冲。

## 构建与测试

```powershell
cmake --build --preset Debug
```

跨语言测试位于上位机 `src/custom_controller_usb/test/test_command_protocol.py`：Python 编码命令后用主机 gcc 编译本工程接收端和 USB 缓冲，以验证协议、分割、CRC 恢复、无效轴保持、旧帧兼容与缓冲溢出。测试不驱动实际电机。

接收复用现有检测任务，未新增 RTOS 任务或外设，因此本次不需要修改 `.ioc`。
