# armor_processor

## Update log
### Date:2022-11-01 预测部分添加小陀螺横移模型
$$
\begin{cases}
    x=x_0+r\cos(\omega t)+Vt\cos(\theta), &x方向\\
    y=y_0+r\sin(\omega t)+Vt\sin(\theta), &y方向
\end{cases}
$$
        方程组中r为小陀螺旋转半径，w为旋转角速度，V为横移速度,θ为目标相对于自己的横移方位角。
        此运动模型假设目标一边小陀螺转动，一边进行横向移动，用于移动目标的反陀螺。

## Debug log
### Date:2022-11-05 Debug processor node.

### Date:2022-11-01 运动模型待调试

### Date:2022-10-26 Lapack library needs to be linked manually.  