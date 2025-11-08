# RoboMaster弹道解算

## 引言

在RoboMaster机器人比赛里，想提高射击命中率，精准的弹道模型可是关键。子弹飞的时候，会受到重力和空气阻力的影响，尤其远距离射击，这些因素会让弹道明显下偏。下面就来详细说说几种弹道解算模型的数学推导，从简单的抛物线模型一直到考虑空气阻力的复杂模型。

## 基础模型：抛物线运动

### 数学模型推导

最简单的就是理想抛物线模型，它只考虑重力的影响。根据牛顿运动定律，能建立这样的运动方程：

**运动方程：**
$$
\begin{aligned}
x &= v_{x0}t \\
y &= v_{y0}t - \frac{1}{2}gt^2
\end{aligned}
$$

这里面：
- $v_{x0} = v_0 \cos\theta$ 是水平初速度
- $v_{y0} = v_0 \sin\theta$ 是垂直初速度
- $g$ 是重力加速度
- $\theta$ 是发射仰角

这个模型基于经典力学公式，能算出子弹在重力作用下的下落高度。但实际用的时候，因为没考虑空气阻力，远距离射击时误差会比较大。

## 单方向空气阻力模型

### 模型假设与数学推导

考虑到子弹飞行角度一般不大（通常小于10°），我们主要考虑水平方向的空气阻力，垂直方向的阻力分量可以忽略。空气阻力和速度的平方成正比：

**阻力模型：**
$$
f = k_0 v^2
$$

水平方向的阻力是：
$$
f_x = k_0 v_x^2
$$

**运动微分方程：**
根据牛顿第二定律：
$$
\begin{aligned}
-m\frac{dv_x}{dt} &= k_0 v_x^2 \\
\frac{dv_x}{dt} &= -\frac{k_0}{m} v_x^2
\end{aligned}
$$

让 $k_1 = \frac{k_0}{m}$，就有：
$$
\frac{dv_x}{dt} = -k_1 v_x^2
$$

**求解速度方程：**
分离变量：
$$
\begin{aligned}
\frac{dv_x}{v_x^2} &= -k_1 dt \\
\int_{v_{x0}}^{v_x} \frac{dv_x}{v_x^2} &= -\int_0^t k_1 dt \\
\left[-\frac{1}{v_x}\right]_{v_{x0}}^{v_x} &= -k_1 t \\
\frac{1}{v_x} - \frac{1}{v_{x0}} &= k_1 t
\end{aligned}
$$

解出来水平方向的速度模型是：
$$
v_x = \frac{v_{x0}}{k_1 v_{x0} t + 1}
$$

**求解位移方程：**
$$
\begin{aligned}
\frac{dx}{dt} &= v_x = \frac{v_{x0}}{k_1 v_{x0} t + 1} \\
dx &= \frac{v_{x0}}{k_1 v_{x0} t + 1} dt \\
\int_0^x dx &= \int_0^t \frac{v_{x0}}{k_1 v_{x0} t + 1} dt \\
x &= \frac{1}{k_1} \ln(k_1 v_{x0} t + 1)
\end{aligned}
$$

**垂直方向运动：**
垂直方向只考虑重力：
$$
y = v_{y0}t - \frac{1}{2}gt^2
$$

**模型退化验证：**
当阻力系数 $k_1 \to 0$ 时：
$$
\begin{aligned}
\lim_{k_1 \to 0} x &= \lim_{k_1 \to 0} \frac{\ln(k_1 v_{x0} t + 1)}{k_1} \\
&= \lim_{k_1 \to 0} \frac{v_{x0} t}{k_1 v_{x0} t + 1} \quad \text{(洛必达法则)} \\
&= v_{x0} t
\end{aligned}
$$

能看出来，当阻力系数趋近于零时，这个模型就退化成了理想抛物线模型。

## 完全空气阻力模型

### 模型改进与详细推导

对于大角度射击的情况，就得考虑两个方向的空气阻力了。阻力可以分解为：
$$
\begin{aligned}
f_x &= k_0 v^2 \cos\theta = k_0 v v_x \\
f_y &= k_0 v^2 \sin\theta = k_0 v v_y
\end{aligned}
$$

为了简化计算，做个近似：
$$
\begin{aligned}
\hat{f_x} &= k_0 v_x^2 \\
\hat{f_y} &= k_0 v_y^2
\end{aligned}
$$

### 上升段推导

**运动方程：**
$$
\begin{aligned}
-m\frac{dv_y}{dt} &= mg + k_0 v_y^2 \\
\frac{dv_y}{dt} &= -g - k_1 v_y^2
\end{aligned}
$$

这里 $k_1 = \frac{k_0}{m}$

**求解速度方程：**
分离变量：
$$
\begin{aligned}
\frac{dv_y}{k_1 v_y^2 + g} &= -dt \\
\int_{v_{y0}}^{v_y} \frac{dv_y}{k_1 v_y^2 + g} &= -\int_0^t dt
\end{aligned}
$$

让 $a = \sqrt{\frac{k_1}{g}}$，则：
$$
\begin{aligned}
\int \frac{dv_y}{k_1 v_y^2 + g} &= \frac{1}{g} \int \frac{dv_y}{a^2 v_y^2 + 1} \\
&= \frac{1}{g} \cdot \frac{1}{a} \arctan(a v_y) \\
&= \frac{1}{\sqrt{k_1 g}} \arctan\left(\sqrt{\frac{k_1}{g}} v_y\right)
\end{aligned}
$$

代入边界条件：
$$
\begin{aligned}
\frac{1}{\sqrt{k_1 g}} \left[\arctan\left(\sqrt{\frac{k_1}{g}} v_y\right) - \arctan\left(\sqrt{\frac{k_1}{g}} v_{y0}\right)\right] &= -t \\
\arctan\left(\sqrt{\frac{k_1}{g}} v_y\right) &= \arctan\left(\sqrt{\frac{k_1}{g}} v_{y0}\right) - \sqrt{k_1 g} t
\end{aligned}
$$

解出速度方程：
$$
v_y = \sqrt{\frac{g}{k_1}} \tan\left(\arctan\left(\sqrt{\frac{k_1}{g}} v_{y0}\right) - \sqrt{k_1 g} t\right)
$$

让 $C = \frac{1}{\sqrt{k_1 g}} \arctan\left(\sqrt{\frac{k_1}{g}} v_{y0}\right)$，则：
$$
v_y = \sqrt{\frac{g}{k_1}} \tan\left(\sqrt{k_1 g} (C - t)\right)
$$

**求解位移方程：**
$$
\begin{aligned}
y &= \int_0^t v_y dt = \sqrt{\frac{g}{k_1}} \int_0^t \tan\left(\sqrt{k_1 g} (C - t)\right) dt
\end{aligned}
$$

让 $z = \sqrt{k_1 g} (C - t)$，则 $dz = -\sqrt{k_1 g} dt$：
$$
\begin{aligned}
y &= \sqrt{\frac{g}{k_1}} \int_{\sqrt{k_1 g} C}^{\sqrt{k_1 g} (C - t)} \tan z \cdot \left(-\frac{dz}{\sqrt{k_1 g}}\right) \\
&= -\frac{1}{k_1} \int_{\sqrt{k_1 g} C}^{\sqrt{k_1 g} (C - t)} \tan z dz \\
&= \frac{1}{k_1} \left[\ln|\cos z|\right]_{\sqrt{k_1 g} (C - t)}^{\sqrt{k_1 g} C} \\
&= \frac{1}{k_1} \ln\left(\frac{\cos\left(\sqrt{k_1 g} C\right)}{\cos\left(\sqrt{k_1 g} (C - t)\right)}\right)
\end{aligned}
$$

**最高点计算：**
当 $v_y = 0$ 时，子弹到达最高点：
$$
\begin{aligned}
\sqrt{k_1 g} (C - t_{max}) &= 0 \\
t_{max} &= C = \frac{1}{\sqrt{k_1 g}} \arctan\left(\sqrt{\frac{k_1}{g}} v_{y0}\right)
\end{aligned}
$$

最大高度是：
$$
\begin{aligned}
y_{max} &= \frac{1}{k_1} \ln\left(\frac{\cos(0)}{\cos\left(\sqrt{k_1 g} C\right)}\right) \\
&= -\frac{1}{k_1} \ln\left(\cos\left(\sqrt{k_1 g} C\right)\right)
\end{aligned}
$$

利用三角恒等式 $\cos(\arctan x) = \frac{1}{\sqrt{1+x^2}}$：
$$
\begin{aligned}
y_{max} &= -\frac{1}{k_1} \ln\left(\frac{1}{\sqrt{1 + \frac{k_1}{g} v_{y0}^2}}\right) \\
&= \frac{1}{2k_1} \ln\left(1 + \frac{k_1}{g} v_{y0}^2\right)
\end{aligned}
$$

### 下降段推导

下降段简单处理，只考虑重力：
$$
y = y_{max} - \frac{1}{2}g(t - t_{max})^2
$$

### 阻力系数补偿

因为推导时用了近似 $\hat{f_y} = k_0 v_y^2$，而实际阻力是 $f_y = k_0 v^2 \sin\theta$，所以需要补偿：

$$
\hat{k_1} = k_1 \cdot \frac{1}{\sin\alpha}
$$

这里 $\alpha$ 是发射初始角度。

## 四阶龙格库塔法（RK4）

### 数值方法原理

对于那些没法解析求解的微分方程，可以用数值方法。考虑空气阻力的运动方程是：

**微分方程组：**
$$
\begin{aligned}
\frac{du}{dx} &= -k_1 u \sqrt{1 + p^2} \\
\frac{dp}{dx} &= -\frac{g}{u^2}
\end{aligned}
$$

其中：
- $u$ 是水平方向速度分量
- $p = \frac{dy}{dx}$ 是弹道斜率

### RK4算法推导

四阶龙格库塔法的通用形式是：
$$
y_{n+1} = y_n + \frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)
$$

对于我们的方程组：

**第一步：**
$$
\begin{aligned}
k_{1u} &= f_u(x_n, u_n, p_n) = -k_1 u_n \sqrt{1 + p_n^2} \\
k_{1p} &= f_p(x_n, u_n, p_n) = -\frac{g}{u_n^2}
\end{aligned}
$$

**第二步：**
$$
\begin{aligned}
k_{2u} &= f_u\left(x_n + \frac{h}{2}, u_n + \frac{h}{2}k_{1u}, p_n + \frac{h}{2}k_{1p}\right) \\
k_{2p} &= f_p\left(x_n + \frac{h}{2}, u_n + \frac{h}{2}k_{1u}, p_n + \frac{h}{2}k_{1p}\right)
\end{aligned}
$$

**第三步：**
$$
\begin{aligned}
k_{3u} &= f_u\left(x_n + \frac{h}{2}, u_n + \frac{h}{2}k_{2u}, p_n + \frac{h}{2}k_{2p}\right) \\
k_{3p} &= f_p\left(x_n + \frac{h}{2}, u_n + \frac{h}{2}k_{2u}, p_n + \frac{h}{2}k_{2p}\right)
\end{aligned}
$$

**第四步：**
$$
\begin{aligned}
k_{4u} &= f_u\left(x_n + h, u_n + h k_{3u}, p_n + h k_{3p}\right) \\
k_{4p} &= f_p\left(x_n + h, u_n + h k_{3u}, p_n + h k_{3p}\right)
\end{aligned}
$$

**更新：**
$$
\begin{aligned}
u_{n+1} &= u_n + \frac{h}{6}(k_{1u} + 2k_{2u} + 2k_{3u} + k_{4u}) \\
p_{n+1} &= p_n + \frac{h}{6}(k_{1p} + 2k_{2p} + 2k_{3p} + k_{4p}) \\
y_{n+1} &= y_n + p_n \cdot h
\end{aligned}
$$

## 模型选择与总结

上面详细介绍了RoboMaster里的多种弹道解算模型，从简单的抛物线模型到考虑空气阻力的复杂模型，还给出了完整的数学推导。每种模型都有适合它的场景：

- **抛物线模型**：有扎实的理论基础，计算简单，适合近距离、对精度要求不高的场景
- **单方向空气阻力模型**：实用性强，在精度和效率之间平衡得不错，适合大多数比赛距离
- **完全空气阻力模型**：考虑的因素更全面，适合大角度射击场景
- **RK4数值方法**：精度最高，但计算复杂，适合远距离精确打击

实际用的时候，要根据具体场景选合适的模型，在精度和计算效率之间找到平衡。这些模型经过实际比赛验证，能明显提高机器人的射击命中率。

## 参考资料

1. [RoboMaster OSS 的迭代弹道模型](https://robomaster-oss.github.io/rmoss_tutorials/#/rmoss_core/rmoss_projectile_motion/projectile_motion_iteration)
2. [沈阳航空航天大学TUP2022年步兵视觉开源](https://github.com/tup-robomaster/TUP-InfantryVision-2022/tree/main/coordsolver)
