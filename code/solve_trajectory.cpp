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
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <solve_trajectory.h>
#include <vector>
#include <random>
#include<chrono>

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

    // printf("x:= %f, theta:=%f,? t:=%f", x, theta, solve_trajectory->flight_time);
    //计算子弹落点高度
    fp32 bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    return bullet_drop_z;
}
static double calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta, float k1)
{
    solve_trajectory->flight_time = (float)((exp(k1 * x) - 1) / (k1 * bullet_speed * cos(theta)));

    // printf("x:= %f, theta:=%f, t:=%f", x, theta, solve_trajectory->flight_time);
    //计算子弹落点高度
    double bullet_drop_z = (double)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
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
static double calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, double x, double bullet_speed, double theta, double k1)
{
    //子弹落点高度
    double bullet_drop_z = 0;
    //计算总飞行时间
    solve_trajectory->flight_time = (double)((exp(k1 * x) - 1) / (k1 * bullet_speed * cos(theta)));
    // printf("飞行时间%f", solve_trajectory->flight_time);
    if (theta > 0) 
    {
        //补偿空气阻力系数 对竖直方向
        //上升过程中 子弹速度方向向量的角度逐渐趋近于0，竖直空气阻力 hat(f_z) = f_z * sin(theta) 会趋近于零 ，水平空气阻力 hat(f_x) = f_x * cos(theta) 会趋近于 f_x ，所以要对竖直空气阻力系数进行补偿
        double k_z = k1 * (1 / sin(theta));
        // 上升段
        // 初始竖直飞行速度
        double v_z_0 = bullet_speed * sin(theta);
        // 计算上升段最大飞行时间
        double max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
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
            double z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // 计算下降
            bullet_drop_z = z_max - 0.5f * GRAVITY * pow((solve_trajectory->flight_time - max_flight_up_time), 2);
        }
    }
    else
    {
        bullet_drop_z = (double)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
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
static float calc_bullet_drop_in_RK4(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta){
    // fp32 pitch_offset = 0.0;
    // TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
    // 初始化
    fp32 cur_x = x;
    fp32 cur_y = 0;
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
static double calc_bullet_drop_in_RK4(solve_trajectory_t* solve_trajectory, double x, double bullet_speed, double theta, double k1){
    // fp32 pitch_offset = 0.0;
    // TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
    // 初始化
    double cur_x = x;
    double cur_y = 0;
    double p = tan(theta / 180 * PI);
    double v = bullet_speed;
    double u = v / sqrt(1 + pow(p, 2));
    double delta_x = x / RK_ITER;
    for (int j = 0; j < RK_ITER; j++)
    {
        double k1_u = -k1 * u * sqrt(1 + pow(p, 2));
        double k1_p = -GRAVITY / pow(u, 2);
        double k1_u_sum = u + k1_u * (delta_x / 2);
        double k1_p_sum = p + k1_p * (delta_x / 2);

        double k2_u = -k1 * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
        double k2_p = -GRAVITY / pow(k1_u_sum, 2);
        double k2_u_sum = u + k2_u * (delta_x / 2);
        double k2_p_sum = p + k2_p * (delta_x / 2);

        double k3_u = -k1 * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
        double k3_p = -GRAVITY / pow(k2_u_sum, 2);
        double k3_u_sum = u + k3_u * (delta_x / 2);
        double k3_p_sum = p + k3_p * (delta_x / 2);

        double k4_u = -k1 * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
        double k4_p = -GRAVITY / pow(k3_u_sum, 2);

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
double calc_error(const pos & t, solve_trajectory_t & solve_trajectory, fp32 x_offset, fp32 z_offset, float k1){
  double theta = 0;
  double bullet_drop_z =
      calc_bullet_drop_in_RK4(
          &solve_trajectory,
          t.x - (arm_cos_f32(theta) * x_offset - arm_sin_f32(theta) * z_offset),
          solve_trajectory.current_bullet_speed, theta, k1) +
      (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
  
  // 误差输出
  double error = pow((t.z - bullet_drop_z), 2);
  return error;
}
bool offset_param_adam(std::vector<pos> & measure,
                      solve_trajectory_t & solve_trajectory, 
                      fp32 x_offset, fp32 z_offset) {
    // Adam优化器参数
    double k1 = -1.01;           // 初始估计值
    double learning_rate = 0.001; // 学习率
    double beta1 = 0.9;           // 一阶矩衰减率
    double beta2 = 0.999;         // 二阶矩衰减率
    double epsilon = 1e-8;        // 防止除零的小常数
    
    // Adam状态变量
    double m = 0.0;  // 一阶矩估计
    double v = 0.0;  // 二阶矩估计
    int t = 0;       // 时间步
    
    size_t max_epochs = 50000;
    double dx = 1e-6;  // 数值微分的步长

    for (int epoch = 0; epoch < max_epochs; epoch++) {
        t++;  // 增加时间步
        
        double total_gradient = 0;
        double total_error = 0;
        int valid_samples = 0;

        // 计算当前批次的梯度
        for (const auto &t_data : measure) {
            // 使用中心差分法计算梯度
            double error_plus = calc_error(t_data, solve_trajectory, x_offset, z_offset, k1 + dx);
            double error_minus = calc_error(t_data, solve_trajectory, x_offset, z_offset, k1 - dx);
            double g = (error_plus - error_minus) / (2 * dx);

            // 梯度裁剪，防止异常值
            if (std::isfinite(g) && fabs(g) < 1e6) {
                total_gradient += g;
                total_error += calc_error(t_data, solve_trajectory, x_offset, z_offset, k1);
                valid_samples++;
            }
        }

        if (valid_samples == 0) {
            std::cout << "No valid gradients, stopping" << std::endl;
            break;
        }

        double avg_gradient = total_gradient / valid_samples;
        double avg_error = total_error / measure.size();

        // Adam更新步骤
        m = beta1 * m + (1 - beta1) * avg_gradient;           // 更新一阶矩估计
        v = beta2 * v + (1 - beta2) * pow(avg_gradient, 2);   // 更新二阶矩估计
        
        // 偏差修正
        double m_hat = m / (1 - pow(beta1, t));
        double v_hat = v / (1 - pow(beta2, t));
        
        // 参数更新
        k1 -= learning_rate * m_hat / (sqrt(v_hat) + epsilon);

        // 每100轮输出一次信息
        if (epoch % 100 == 0) {
            std::cout << "Epoch " << epoch 
                      << " k1:=" << k1
                      << " gradient:=" << avg_gradient
                      << " m_hat:=" << m_hat
                      << " v_hat:=" << v_hat
                      << " error:=" << avg_error
                      << std::endl;
        }

        // 收敛判断
        if (avg_error < 0.005 && fabs(avg_gradient) < 0.5) {
            std::cout << "Converged at epoch " << epoch << std::endl;
            solve_trajectory.k1 = k1;
            return true;
        }

        // 提前终止条件
        if (avg_error > 1e10 || !std::isfinite(avg_error) || !std::isfinite(k1)) {
            std::cout << "Diverged, stopping early at epoch " << epoch << std::endl;
            break;
        }
        
        // 学习率衰减
        if (epoch > 1000 && epoch % 1000 == 0) {
            learning_rate *= 0.99;
        }
    }
    
    std::cout << "Optimization completed without convergence" << std::endl;
    solve_trajectory.k1 = k1;  // 保存当前最佳值
    return false;
}
bool offset_param(std::vector<pos> & measure,
                  solve_trajectory_t & solve_trajectory, fp32 x_offset,
                  fp32 z_offset) {
  double k1 = -0.1;             //初始估计值
  double last_k1 = 0;     
  double dx = 0.0001;              
  long double learning_rate = 1e-9; //学习率
  size_t max_epochs = 200000;
  double avg_gradient = 0;
  double avg_error = 0; 
  

  for (int epoch = 0; epoch < max_epochs; epoch++) {
    double total_gradient = 0;
    double total_error = 0;
    int num = measure.size();
    // 批量计算梯度和误差
    for (const auto &t : measure) {
      
      double error_plus =
          calc_error(t, solve_trajectory, x_offset, z_offset, k1 + dx);
      double error_current =
          calc_error(t, solve_trajectory, x_offset, z_offset, k1 - dx);
      double g_1 = (error_plus - error_current) / (2 * dx);

      if (fabs(g_1) < 10e6){
        //为了防止梯度爆炸
        total_gradient += g_1;
        total_error += error_current;
      }
      else{
        num--;
      }
    }
    if (num != 0){
      avg_gradient = total_gradient / num;
      avg_error = total_error / measure.size();       
    }
    else{
      avg_gradient = 10e5;
      avg_error = total_error / measure.size();
    }
    

    // 更新参数
    last_k1 = k1;
    k1 -= learning_rate * avg_gradient;


    std::cout << "Epoch " << epoch << " k1:=" << k1
              << " gradient:=" << avg_gradient << " error:=" << avg_error
              << std::endl;


    // 收敛判断
    if (avg_error < 0.005 || fabs(avg_gradient) < 5) {
      std::cout << "Converged at epoch " << epoch << std::endl;
      solve_trajectory.k1 = k1;
      return true;
    }
  }
  std::cout << "offset false" << std::endl;
  return false;
}


using namespace std::chrono;

#if DEBUG_IN_COMPUTER
int main() {
  solve_trajectory_t s = {.current_bullet_speed = 25, .k1 = 0.1};
  fp32 x_offset = 0, z_offset = 0;
  fp32 x = 0, theta = 0.1;
  std::vector<pos> measure{};
  //生产一点噪声
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dis(0.0, 0.01); // 均值为0，标准差为1的正态分布

  for (int i = 0; i <= 100; i++){
    x += 1 + 0.05 * i;
    fp32 z = calc_bullet_drop_in_RK4(
          &s,
          x - (arm_cos_f32(theta) * x_offset - arm_sin_f32(theta) * z_offset),
          s.current_bullet_speed, theta) +
      (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset) + dis(gen);;
    measure.push_back({x + dis(gen), z});
  }
  auto start = system_clock::now();
  offset_param_adam(measure, s, x_offset, z_offset);
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  std::cout << double(duration.count()) * microseconds::period::num / microseconds::period::den << std::endl;
  return 0;
}
#endif