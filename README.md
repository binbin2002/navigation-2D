# navigation-2D
简单适配fastlio和movebase用于2D导航




# 2025.07.01
- 修改了一些bug,禁用了gazebo自带的odom->base_link的TF变换
- 同时发布了新的静态tf变换.具体在navigation.launch文件.

# 2025.09.01
- local planner 添加了teb规划
- 修改了move_base的部分参数