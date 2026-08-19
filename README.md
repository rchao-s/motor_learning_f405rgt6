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
