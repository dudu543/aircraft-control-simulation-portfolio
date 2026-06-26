# Aircraft Control Simulation Portfolio

本仓库整理了飞行器动力学建模、飞行控制、制导律仿真与状态估计相关的小型仿真项目，主要用于展示 MATLAB/Simulink/Python 环境下的建模、控制律设计、数值仿真与结果分析能力。

## Project List

| No. | Project | Topic | Methods | Outputs |
|---|---|---|---|---|
| 01 | Aircraft 6-DOF Simulation | 飞行器动力学建模 | 刚体六自由度模型、NED/机体系转换、数值积分 | 姿态角、速度、位置响应曲线 |
| 02 | Quadrotor Tracking Control | 四旋翼轨迹跟踪控制 | PID、LQR、扰动响应分析 | 阶跃响应、圆轨迹跟踪、误差对比 |
| 03 | eVTOL Wind Disturbance Simulation | eVTOL 垂直起降风扰仿真 | 城市风场模型、双环 PID/PD 控制、参数整定 | 无风/有风/调参后对比 |
| 04 | Proportional Navigation Guidance | 导弹比例导引律仿真 | PN 制导律、二维相对运动模型 | 拦截轨迹、LOS 角速度、脱靶量 |
| 05 | EKF Attitude Estimation | 姿态估计 | IMU 仿真数据、扩展卡尔曼滤波 | 姿态估计曲线、误差曲线 |

## Technical Keywords

- Flight dynamics
- 6-DOF rigid-body modeling
- MATLAB / Simulink / Python
- PID control
- LQR control
- eVTOL vertical takeoff and landing
- Wind disturbance modeling
- Proportional navigation guidance
- Extended Kalman Filter
- Attitude estimation

## Repository Structure

```text
aircraft-control-simulation-portfolio/
├─ 01_aircraft_6dof_sim/
├─ 02_quadrotor_tracking_control/
├─ 03_evtol_wind_disturbance/
├─ 04_pn_guidance_sim/
├─ 05_ekf_attitude_estimation/
└─ assets/