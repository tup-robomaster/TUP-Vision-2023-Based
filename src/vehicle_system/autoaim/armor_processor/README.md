# armor_processor

## Update log
### Date:2022-11-23 基于‘当前’统计模型（CS model）的卡尔曼滤波算法 
    Singer模型
        机动模型把机动控制项作为相关噪声（有色噪声）进行建模。
        目标加速度a(t)作为指数自相关的零均值随机过程，即：
$$
    R(\tau)=E[a(t)a(t+\tau)]=\sigma_m^2e^{-\alpha\lVert\tau\rVert}
$$

$\sigma_m^2$为目标机动加速度的方差，根据目标机动加速度的概率密度函数计算，$\alpha$为目标机动频率。

####
机动目标加速度分布作如下假定：

1.目标机动加速度处于极大值$a_{M}$的概率为$P_{M}$，处于极小值$-a_{M}$的概率也为$P_{M}$；

2.机动加速度等于0的概率为$P_{0}$(非机动)；

3.机动加速度在区间[$-a_{M}$, $a_{M}$]上近似服从均匀分布。

    由以上假设可推出目标加速度的概率密度函数，并进而计算得到目标机动加速度方差为：
$$
    \sigma_m^2=\frac{a_{M}^2}{3}(1+4P_{M}-P_{0})
$$

    应用于卡尔曼滤波时要对加速度a(t)“白化”。
    白化后得到如下关系式：
$$
    \dot{a_{t}}=-\alpha{a_{t}}+w_{t}
$$

其中$a_{t}$输入为白噪声的一阶时间相关模型，$w_{t}$是均值为0、方差为$2\alpha{\sigma_m^2}$的高斯白噪声，即：
$$
    E[w(t)w(\tau)]=2\alpha\sigma_m^2\delta(t-\tau)
$$

根据以上理论，假设目标的状态向量$\chi=[x,\dot{x},\ddot{x}]$，$\ddot{x}$=$a$，则上述一阶时间相关模型如下：    
$$
    \dot{\chi_{t}}=A\chi_t+w_t
$$
其中
$$
A=
\begin{bmatrix}
    0 & 1 & 0 \\
    0 & 0 & 1 \\
    0 & 0 & -\alpha
\end{bmatrix}
，w=[0, 0, v]
$$
将A和$w$代入上式得：
$\dot\chi_t$=$\begin{bmatrix}0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & -\alpha \end{bmatrix}$ $\begin{bmatrix}\chi_t \\ \dot{\chi_t} \\ \ddot{\chi_t}\end{bmatrix}$+$\begin{bmatrix}0 \\ 0 \\ v\end{bmatrix}$

离散状态采样时间间隔T有：
$$
    \chi_{k+1}=F_k\chi_{k}+w_k
$$

式中：
$$
    F_k=e^{AT}=\begin{bmatrix}
        1 & T & \frac{\alpha{T}-1+e^{-\alpha{T}}}{\alpha^2} \\
        0 & 1 & \frac{1-e^{-\alpha{T}}}{\alpha} \\
        0 & 0 & e^{-\alpha{T}}
    \end{bmatrix},
    w_k的噪声协方差矩阵Q_k=2\alpha\sigma_m^2\begin{bmatrix}
        q_{11} & q_{12} & q_{13} \\
        q_{21} & {q_22} & q_{23} \\
        q_{31} & q_{32} & q_{33}
    \end{bmatrix} 
$$
其中Q为对称矩阵，且
$$
\begin{cases}
    q_{11}=(\frac{[1-e^{-2\alpha{T}}+2\alpha{T}+\frac{2\alpha^3T^3}{3}-2\alpha^2T^2-4\alpha{T}e^{-\alpha{T}}]}{2\alpha^{5}}) \\
    q_{12}=\frac{[e^{-2\alpha{T}}+1-2e^{-\alpha{T}}+2\alpha{T}e^{-\alpha{T}}-2\alpha{T}+\alpha^2{T}^2]}{2\alpha^4} \\
    q_{13}=\frac{[1-e^{-2\alpha{T}}-2\alpha{T}e^{-\alpha{T}}]}{2\alpha^3} \\
    q_{22}=\frac{[4e^{-\alpha{T}}-3-e^{-2\alpha{T}}+2\alpha{T}]}{2\alpha^3} \\
    q_{23}=\frac{[e^{-2\alpha{T}}+1-2e^{-\alpha{T}}]}{2\alpha^2} \\
    q_{33}=\frac{[1-e^{-2\alpha{T}}]}{2\alpha}
\end{cases}
$$
若目标机动频率$\alpha\to\infty$时，此时状态转移矩阵为
$$
    F=e^{AT}=\begin{bmatrix}
        1 & T & \frac{T^2}{2} \\
        0 & 1 & T \\
        0 & 0 & 1
    \end{bmatrix}
$$
过程噪声协方差矩阵为
$$
    Q=q\begin{bmatrix}
        \frac{T^5}{20} & \frac{T^4}{8} & \frac{T^3}{6} \\
        \frac{T^4}{8} & \frac{T^3}{3} & \frac{T^2}{2} \\
        \frac{T^3}{6} & \frac{T^2}{2} & T
    \end{bmatrix}
$$
模型此时等效于为匀加速运动模型，q取一个较小值，如q=0.05。

若目标机动频率$\alpha\to0$，此时状态转移矩阵
$$
F=\begin{bmatrix}
    1 & T & 0 \\
    0 & 1 & 0 \\
    0 & 0 & 1  
\end{bmatrix}
$$
此时模型等效于匀速运动模型。

    以上可以看到匀加速运动模型和匀速运动模型是Singer模型的两个特例，即Singer模型针对的是处于匀加速运动和匀速运动之间的机动目标。但是Singer模型认为目标机动加速度符合零均值的
    均匀分布并不符合大多数的实际情况。

基于此，出现了对Singer模型的改进算法——“当前”统计模型（CS Model）
    
    CS模型提出两条新的假设：
    1.加速度非零均值；
    2.加速度的概率密度服从修正的瑞利分布。
类似于Singer模型，CS模型的离散时间状态方程可以表示如下：
$\\
    X_{k+1}=F_{k}X_{k}+G_{k}\overset{-}{a_{k}}+W_{k}
$

其中，$G_{k}$为加速度输入控制矩阵，$\overset{-}a_{k}$为目标加速度均值。
$$
    G_{k}=\begin{bmatrix}
        \frac{(-T+\frac{\alpha{T^2}}{2}+\frac{1-e^{-\alpha{T}}}{\alpha})}{\alpha} \\
        T-\frac{(1-e^{-\alpha{T}})}{\alpha} \\
        1-e^{-\alpha{T}}   
    \end{bmatrix}
$$
$\overset{-}a_k$可用当前时刻状态估计中的预测加速度代替，即
$
    \overset{-}a_k=\hat{\ddot{\chi}}_{k|k-1}
$

噪声$W_k$的协方差矩阵和Singer模型的$Q$相同，但是机动加速度的方差$\sigma_a^2$计算公式为：
$$
    \sigma_a^2=\begin{cases}
        \frac{4-\Pi}{\Pi}[a_{max}-\overline{a}_{k}]^2，\overline{a}_{k}>0 \\
        \frac{4-\Pi}{\Pi}[a_{-max}+\overline{a}_{k}]^2，\overline{a}_{k}<0
    \end{cases}
$$
其中$a_{-max}$为负加速度上限，不必等于$-a_{max}$。

可以看出模型可以利用目标加速度的预测值自适应地调整加速度方差$\sigma^2$，对目标有更快的响应速度。

    TODO:预测部分调试说明：
    程序预测部分卡尔曼滤波的状态向量为X=[x,v,a]，输入仅为目标x轴或y轴的位置信息,v和a由卡尔曼滤波预测得出(可能会造成卡尔曼滤波发散，初始化滤波时可以考虑传入目标的速度和加速度信息，但考虑到通过帧差法计算出的速度和加速度误差较大，不可避免地会造成预测抖动，因此并未使用)，Siner模型待调的参数包括目标机动频率α、目标加速度方差σ^2、采样周期T。CS模型待调的参数包括目标机动频率α、采样周期T、机动最大加速度a_max、机动最大负加速度a_-max。
    TODO:调试经验：
    比赛场上目标不会机动过快，可能的就是1s机动一次，可以稍微往高了调，提高响应速度；采样周期T要根据程序帧间运行时间、通信延迟、发弹延迟计算；目标加速度方差σ^2作为噪声协方差矩阵的控制量，较小可以平滑预测点，减小抖动，但是会出现一定程度滞后；较大会出现一定程度抖动，但是可以提高预测效果。
                    
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
### Date:2022-11-18 预测部分实现动态配置调试，代码略显冗余，但后续方便调试。

### Date:2022-11-17 预测部分加入基于Singer模型的扩展卡尔曼滤波，算法将持续优化，后续会添加CS模型。

### Date:2022-11-05 Debug processor node.

### Date:2022-11-01 运动模型待调试

### Date:2022-10-26 Lapack library needs to be linked manually.  