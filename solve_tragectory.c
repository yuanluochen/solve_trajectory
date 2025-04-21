#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define GRAVITY 9.8
#define MAX_ITERATE_COUNT 10
#define ITERATE_SCALE_FACTOR 0.3
#define PRECISION 0.0001


//接口适配
typedef float fp32;
#define arm_sin_f32 sin
#define arm_cos_f32 cos


//弹道计算结构体
typedef struct
{
    // 当前弹速
    fp32 current_bullet_speed;
    // 弹道系数
    fp32 k1;
    //子弹飞行时间
    fp32 flight_time;

} solve_trajectory_t;


/**
 * @brief 计算弹道落点 -- 完全空气阻力模型 该模型适用于大仰角击打的击打
 * 
 * @param solve_trajectory 弹道解算结构体
 * @param x 距离
 * @param bullet_speed 弹速
 * @param pitch 仰角
 * @return 弹道落点
 */
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta)
{
    //子弹落点高度
    fp32 bullet_drop_z = 0;
    //计算总飞行时间
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));
    printf("飞行时间%f", solve_trajectory->flight_time);
    if (theta > 0) 
    {
        printf("da\n");
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

/**
 * @brief 二维平面弹道模型，计算pitch轴的仰角，
 *
 * @param solve_tragectory 弹道计算结构体
 * @param x 水平距离
 * @param y 竖直距离
 * @param x_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的x轴偏移量
 * @param y_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的y轴偏移量
 * @param bullet_speed 弹速
 * @return 返回pitch轴数值
 */
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset)
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
        bullet_drop_z =
        calc_bullet_drop_in_complete_air(
                solve_trajectory,
                x - (arm_cos_f32(theta) * x_offset - arm_sin_f32(theta) * z_offset),
                solve_trajectory->current_bullet_speed, 
                theta) +
            (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
        // 计算误差
        calc_and_actual_error = z - bullet_drop_z;
        // 对瞄准高度进行补偿
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        printf("第%d次瞄准，高度为%f, 仰角%f\n", ++count, aim_z, theta);
        printf("第%d次瞄准，发射系x:%f, z补偿%f, z发射系落点%f ,z机体系落点%f\n", count, x - (arm_cos_f32(theta) * x_offset), (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z - (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z);
        // 判断误差是否符合精度要求
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
    //由于为右手系，theta为向下为正，所以置负
    return -theta;
}

int main(){
  solve_trajectory_t s = {
    .current_bullet_speed = 25,
  };
  calc_target_position_pitch_angle(&s, 4, -0.2, 0.111, 0);
  return 0;
}