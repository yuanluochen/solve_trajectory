/**
 * @file solve_trajectory.c
 * @author yuanluochen
 * @brief 弹道解算文件
 * @version 0.1
 * @date 2025-04-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <cstdio>
#include <iostream>
#include <solve_trajectory.h>
#include <vector>

/**
 * @brief 计算子弹落点
 * @author yuanluochen
 * 
 * @param solve_trajectory 弹道计算结构体
 * @param x 水平距离
 * @param bullet_speed 弹速
 * @param theta 仰角
 * @return 子弹落点
 */
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta)
{
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));

    printf("x:= %f, theta:=%f, t:=%f", x, theta, solve_trajectory->flight_time);
    //计算子弹落点高度
    fp32 bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    return bullet_drop_z;
}

/**
 * @brief 计算弹道落点 -- 完全空气阻力模型 该模型适用于大仰角击打的击打
 * @author yuanluochen
 * 
 * @param solve_trajectory 弹道解算结构体
 * @param x 距离
 * @param bullet_speed 弹速
 * @param theta 仰角
 * @return 弹道落点
 */
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta)
{
    //子弹落点高度
    fp32 bullet_drop_z = 0;
    //计算总飞行时间
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));
    // printf("飞行时间%f", solve_trajectory->flight_time);
    if (theta > 0) 
    {
        //补偿空气阻力系数 对竖直方向
        //上升过程中 子弹速度方向向量的角度逐渐趋近于0，竖直空气阻力 hat(f_z) = f_z * sin(theta) 会趋近于零 ，水平空气阻力 hat(f_x) = f_x * cos(theta) 会趋近于 f_x ，所以要对竖直空气阻力系数进行补偿
        fp32 k_z = solve_trajectory->k1 * (1 / sin(theta));
        // 上升段
        // 初始竖直飞行速度
        fp32 v_z_0 = bullet_speed * sin(theta);
        // 计算上升段最大飞行时间
        fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
        // 判断总飞行时间是否小于上升最大飞行时间
        if (solve_trajectory->flight_time <= max_flight_up_time)
        {
            // 子弹存在上升段
            bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
        }
        else
        {
            // 超过最大上升飞行时间 -- 存在下降段
            // 计算最大高度
            fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // 计算下降
            bullet_drop_z = z_max - 0.5f * GRAVITY * pow((solve_trajectory->flight_time - max_flight_up_time), 2);
        }
    }
    else
    {
        bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    }


    return bullet_drop_z;
}
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta, float k1)
{
    //子弹落点高度
    fp32 bullet_drop_z = 0;
    //计算总飞行时间
    solve_trajectory->flight_time = (float)((exp(k1 * x) - 1) / (k1 * bullet_speed * cos(theta)));
    // printf("飞行时间%f", solve_trajectory->flight_time);
    if (theta > 0) 
    {
        //补偿空气阻力系数 对竖直方向
        //上升过程中 子弹速度方向向量的角度逐渐趋近于0，竖直空气阻力 hat(f_z) = f_z * sin(theta) 会趋近于零 ，水平空气阻力 hat(f_x) = f_x * cos(theta) 会趋近于 f_x ，所以要对竖直空气阻力系数进行补偿
        fp32 k_z = k1 * (1 / sin(theta));
        // 上升段
        // 初始竖直飞行速度
        fp32 v_z_0 = bullet_speed * sin(theta);
        // 计算上升段最大飞行时间
        fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
        // 判断总飞行时间是否小于上升最大飞行时间
        if (solve_trajectory->flight_time <= max_flight_up_time)
        {
            // 子弹存在上升段
            bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
        }
        else
        {
            // 超过最大上升飞行时间 -- 存在下降段
            // 计算最大高度
            fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // 计算下降
            bullet_drop_z = z_max - 0.5f * GRAVITY * pow((solve_trajectory->flight_time - max_flight_up_time), 2);
        }
    }
    else
    {
        bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    }


    return bullet_drop_z;
}
/**
 * @brief 四阶龙格库塔法拟合弹道
 * 
 * @param solve_trajectory 弹道解算结构体
 * @param x 距离
 * @param y 高度
 * @param bullet_speed 弹速
 * @param theta 仰角
 * @return 弹道落点
 */
static float calc_bullet_drop_in_RK4(solve_trajectory_t* solve_trajectory, float x, float y, float bullet_speed, float theta){
    // fp32 pitch_offset = 0.0;
    // TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
    // 初始化
    fp32 cur_x = x;
    fp32 cur_y = y;
    fp32 p = tan(theta / 180 * PI);
    fp32 v = bullet_speed;
    fp32 u = v / sqrt(1 + pow(p, 2));
    fp32 delta_x = x / RK_ITER;
    for (int j = 0; j < RK_ITER; j++)
    {
        fp32 k1_u = -solve_trajectory->k1 * u * sqrt(1 + pow(p, 2));
        fp32 k1_p = -GRAVITY / pow(u, 2);
        fp32 k1_u_sum = u + k1_u * (delta_x / 2);
        fp32 k1_p_sum = p + k1_p * (delta_x / 2);

        fp32 k2_u = -solve_trajectory->k1 * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
        fp32 k2_p = -GRAVITY / pow(k1_u_sum, 2);
        fp32 k2_u_sum = u + k2_u * (delta_x / 2);
        fp32 k2_p_sum = p + k2_p * (delta_x / 2);

        fp32 k3_u = -solve_trajectory->k1 * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
        fp32 k3_p = -GRAVITY / pow(k2_u_sum, 2);
        fp32 k3_u_sum = u + k3_u * (delta_x / 2);
        fp32 k3_p_sum = p + k3_p * (delta_x / 2);

        fp32 k4_u = -solve_trajectory->k1 * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
        fp32 k4_p = -GRAVITY / pow(k3_u_sum, 2);

        u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
        p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

        cur_x += delta_x;
        cur_y += p * delta_x;
    }
    return cur_y;
}

/**
 * @brief 二维平面弹道模型，计算pitch轴的仰角，
 * @author yuanluochen
 *
 * @param solve_tragectory 弹道计算结构体
 * @param x 水平距离
 * @param y 竖直距离
 * @param x_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的x轴偏移量
 * @param y_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的y轴偏移量
 * @param bullet_speed 弹速
 * @param mode 计算模式：
          置 1 完全空气阻力模型
          置 0 单方向空气阻力模型
 * @return 返回pitch轴数值
 */
float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, int mode)
{
    int count = 0;
    // 计算落点高度
    float bullet_drop_z = 0;
    //云台瞄准向量
    float aim_z = z;
    // 坐标变换量
    float gimbal2aim_x = x;
    float gimbal2aim_z = z;

    // 二维平面的打击角
    float theta = 0;
    // 计算值与真实值之间的误差
    float calc_and_actual_error = 0;
    // 比例迭代法
    for (int i = 0; i < MAX_ITERATE_COUNT; i++)
    {
        // 计算仰角
        theta = atan2(aim_z, x);
        // 坐标系变换，从机器人转轴系变为发射最大速度位置坐标系
        // 计算子弹落点高度
        if (mode == 1){
            bullet_drop_z =
              calc_bullet_drop_in_complete_air(
                  solve_trajectory,
                  x - (arm_cos_f32(theta) * x_offset -
                       arm_sin_f32(theta) * z_offset),
                  solve_trajectory->current_bullet_speed, theta) +
              (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
        }
        else if (mode == 2){
            bullet_drop_z =
              calc_bullet_drop_in_RK4(
                  solve_trajectory,
                  x - (arm_cos_f32(theta) * x_offset -
                       arm_sin_f32(theta) * z_offset),
                       aim_z - (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset),
                  solve_trajectory->current_bullet_speed, theta) +
              (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
        }
        else{
            bullet_drop_z =
              calc_bullet_drop(
                  solve_trajectory,
                  x - (arm_cos_f32(theta) * x_offset -
                       arm_sin_f32(theta) * z_offset),
                  solve_trajectory->current_bullet_speed, theta) +
              (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
        }
            
        // 计算误差
        calc_and_actual_error = z - bullet_drop_z;
        // 对瞄准高度进行补偿
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // printf("第%d次瞄准，发射系x:%f, z补偿%f, z发射系落点%f ,z机体系落点%f\n", count, x - (arm_cos_f32(theta) * x_offset), (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z - (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z);
        // 判断误差是否符合精度要求
        count++;
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
    printf("x = %f, 原始pitch = %f, pitch = %f, 迭代次数 = %d\n", x, -atan2(z, x) * 180 / 3.14 , -(theta * 180 / 3.14), count);
    //由于为右手系，theta为向下为正，所以置负
    return -theta;
}

struct pos{
    double x;
    double z;
};
float calc_error(const pos & t, solve_trajectory_t & solve_trajectory, fp32 x_offset, fp32 z_offset, float k1){
  float theta = 0;
  float bullet_drop_z =
      calc_bullet_drop_in_complete_air(
          &solve_trajectory,
          t.x - (arm_cos_f32(theta) * x_offset - arm_sin_f32(theta) * z_offset),
          solve_trajectory.current_bullet_speed, theta, k1) +
      (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
  
  // 误差输出
  fp32 error = pow((t.z - bullet_drop_z), 2);
  return error;
}

void offset_param(std::vector<pos> &measure,
                  solve_trajectory_t &solve_trajectory, fp32 x_offset,
                  fp32 z_offset) {
  float k1 = 0.09;             //初始估计值        
  float dx = 0.0001;              
  float learning_rate = 0.000001; //学习率
  int max_epochs = 10000;

  for (int epoch = 0; epoch < max_epochs; epoch++) {
    double total_gradient = 0;
    double total_error = 0;

    // 批量计算梯度和误差
    for (const auto &t : measure) {
      double error_plus =
          calc_error(t, solve_trajectory, x_offset, z_offset, k1 + dx);
      double error_current =
          calc_error(t, solve_trajectory, x_offset, z_offset, k1);
      double g_1 = (error_plus - error_current) / dx;
      total_gradient += g_1;
      total_error += error_current;
    }

    double avg_gradient = total_gradient / measure.size();
    double avg_error = total_error / measure.size();

    // 更新参数
    k1 -= learning_rate * avg_gradient;


    std::cout << "Epoch " << epoch << " k1:=" << k1
              << " gradient:=" << avg_gradient << " error:=" << avg_error
              << std::endl;

    // 收敛判断
    if (fabs(avg_gradient) < 1e-8 || avg_error < 0.005) {
      std::cout << "Converged at epoch " << epoch << std::endl;
      break;
    }
  }
  solve_trajectory.k1 = k1;
}

#if DEBUG_IN_COMPUTER
int main() {
  solve_trajectory_t s = {.current_bullet_speed = 25, .k1 = 0.1};
  fp32 x_offset = 0, z_offset = 0;
  fp32 x = 0, theta = 0;
  std::vector<pos> measure{};
  for (int i = 0; i <= 10; i++){
    x += 2  + 0.001 * i;
    fp32 z = calc_bullet_drop_in_complete_air(
          &s,
          x - (arm_cos_f32(theta) * x_offset - arm_sin_f32(theta) * z_offset),
          s.current_bullet_speed, theta) +
      (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
    measure.push_back({x, z});
  }
  offset_param(measure, s, x_offset, z_offset);
}
#endif