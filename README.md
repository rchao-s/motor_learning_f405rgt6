2026.8.19
1. Use DMA to establish the UART receive framework.
2. Implement a basic command protocol, supporting speed control commands such as duty 30, duty 50, as well as START, STOP, and STATUS.
3. Limit the PWM duty cycle range.
4. Add a soft-start ramp.
5. 主要问题与解决：

串口乱码
原因是 CubeMX 中 HSE 配置为 25 MHz，但实际晶振为 8 MHz。修改为 HSE=8MHz，PLLM=8，PLLN=336 后，串口通信恢复正常。

串口接收无反应
原因是使用了 HAL_UART_Receive_IT()，但回调函数写的是 HAL_UARTEx_RxEventCallback()。改为 HAL_UARTEx_ReceiveToIdle_IT() 后，可以正常接收不定长命令。

start/stop 命令无效
原因是串口助手发送命令时带有 \r\n，导致长度判断失败。通过去除末尾换行符，再使用 strcmp() 判断命令，问题解决。

电机有时上电不转
原因是电机静止时霍尔信号不变化，外部中断不会触发，hall_state 没有更新。通过上电主动读取一次霍尔状态，并在运行时保底读取霍尔状态，启动稳定性提升。

2026.8.20
1. Read Hall edge or commutation state; calculate RPM using Hall edges.
2. Calculate RPM; output current RPM via UART STATUS command; pay attention to filtering and timeout at low speed.
3. Add working modes: support MODE_DUTY and MODE_RPM.
4. Implement target speed command: support formats like rpm 1000, rpm 1200, etc.
5. Implement a simple PI speed closed-loop control.
6. 主要问题与解决：

电机抖动剧烈
原因是每次霍尔中断都更新 PWM，高转速时中断频率过高，寄存器刷新过于频繁，在主循环中增加 20ms 定时判断，限制 PWM 更新频率为 50Hz

低速测速抖动严重
原因是固定滤波窗口在低速时导致转速更新严重滞后（最长 80ms 才更新一次），改为动态滤波——周期 > 10ms 时不滤波直接响应，中高速才启用移动平均

PI调速效果不好
原因Kp，Ki本身就比较小，给定目标转速与实际转速相差8,900转时算出来的占空比变化量是个0.几的数，而占空比变化量又是个整数，控制量直接变成0了，最后调速中占空比变化量改为浮点，最后调好了再转为整数，就好了。


2026。8.20

添加状态机和故障保护：使用枚举定义 STATE_IDLE/STARTING/RUNNING/FAULT/STOPPING，实现状态切换函数和故障检测逻辑

一开机一直打印idle，电机不转
状态机Switch没写break，状态一直跳 补齐break后正常
