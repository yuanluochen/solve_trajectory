/**
 * @file solve_trajectory.h
 * @author yuanluochen
 * @brief 弹道解算文件
 * @version 0.1
 * @date 2025-04-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef SOLVE_TRAJECTORY_H
#define SOLVE_TRAJECTORY_H

#define DEBUG_IN_COMPUTER 1
#if  DEBUG_IN_COMPUTER
#include <stdio.h>
#include <stdint.h>
#include <cmath>
//接口适配
typedef float fp32;
#define arm_sin_f32 sin
#define arm_cos_f32 cos
#else 
/*此处添加对应设备头文件 */
#endif 

//固定定义
#define PI 3.14
#define RK_ITER 60
#define GRAVITY 9.8
#define MAX_ITERATE_COUNT 30
#define ITERATE_SCALE_FACTOR 0.9
#define PRECISION 0.000001

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
float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, int mode);

#endif 
