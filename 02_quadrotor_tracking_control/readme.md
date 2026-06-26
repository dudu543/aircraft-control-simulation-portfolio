# Quadrotor Trajectory Tracking Control

## 1. Background

本项目用于实现四旋翼轨迹跟踪控制仿真，对比 PID 与 LQR 控制器在阶跃目标、圆形轨迹和外部扰动下的控制性能。

## 2. Model

计划建立简化四旋翼动力学模型，状态包括位置、速度、姿态角和角速度。模型可先采用小角度线性化假设，后续再扩展为非线性模型。

## 3. Method

设计 PID 控制器和 LQR 控制器，对比调节时间、超调量、稳态误差和轨迹跟踪误差。

## 4. Simulation Cases

- 阶跃位置跟踪
- 圆形轨迹跟踪
- 外部扰动下的抗扰响应
- PID 与 LQR 性能对比

## 5. Results

待补充：阶跃响应、圆轨迹、误差曲线、扰动响应。

## 6. How to Run

待补充。

## 7. Resume Description

搭建四旋翼轨迹跟踪仿真模型，对比 PID 与 LQR 控制器在阶跃、圆轨迹和扰动条件下的跟踪性能。