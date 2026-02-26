
帮我写一个json转换txt脚本,前提和要求如下：
1. json中Frames数据的格式为：
    root_position
    + root_orientation_quat
    + joints_positions
    + left_toe_pos
    + right_toe_pos
    + world_linear_vel
    + world_angular_vel
    + joints_vel
    + left_toe_vel
    + right_toe_vel
    + foot_contacts
2. txt需要的Frames格式为：
#For AWD and amp for hardware
#[root position, root orientation,
   joint poses (e.g. rotations), target toe positions, 
   linear velocity, angular velocity, 
   joint velocities, target toe velocities]
#[x, y, z, qx, qy, qz, qw, j1, j2, j3, j4, j5, j6, j7, j8, j9, j10, 
   l_toe_x, l_toe_y, l_toe_z, r_toe_x, r_toe_y, r_toe_z,  
   lin_vel_x, lin_vel_y, lin_vel_z, ang_vel_x, ang_vel_y, ang_vel_z, 
   j1_vel, j2_vel, j3_vel, j4_vel, j5_vel, j6_vel, j7_vel, j8_vel, j9_vel, j10_vel,
   l_toe_vel_x, l_toe_vel_y, l_toe_vel_z, r_toe_vel_x, r_toe_vel_y, r_toe_vel_z]

3. 可以参考/dataformat/mocap_motions中的具体格式，但要注意数据维度，如果维度不匹配就要返回报错 
参考整体格式：
{
"LoopMode": "Wrap",
"FrameDuration": 0.021,
"EnableCycleOffsetPosition": true,
"EnableCycleOffsetRotation": true,
"MotionWeight": 0.5,

"Frames":
[
  [0.00000, 0.00000, 0.30053, -0.00547, 0.06021, 0.04977, 0.99693, -0.03949, 0.77253, -1.87322, -0.09314, 0.69470, -1.75514, 0.12344, 0.59787, -1.62623, 0.00333, 0.70930, -1.42720, 0.20898, -0.14340, -0.23080, 0.21675, 0.10612, -0.25868, -0.13702, -0.10052, -0.27750, -0.19443, 0.13087, -0.30263, 0.64127, 0.04306, 0.10563, -0.31270, -0.22708, 0.46872, -1.18714, -9.49331, 0.90031, -0.09461, 2.68096, -0.13123, -0.22218, -5.86068, 2.97332, 0.34933, 2.13430, -0.01060, 2.11880, -0.24540, 0.52698, -0.66294, -0.03235, -0.08883, 1.28985, -0.05022, -0.08304, -0.64360, 0.10529, 0.04346],
  [0.01348, 0.00226, 0.30111, -0.00832, 0.05770, 0.05488, 0.99679, -0.06442, 0.57317, -1.85431, -0.09513, 0.75100, -1.75790, 0.11877, 0.47480, -1.56379, 0.01066, 0.75412, -1.42742, 0.25348, -0.14856, -0.21973, 0.20283, 0.10544, -0.26055, -0.10993, -0.10158, -0.27924, -0.20795, 0.13308, -0.30172, 0.63849, 0.04790, 0.12467, -0.28271, -0.46333, 0.56926, -1.15606, -8.87355, 2.57271, -0.06107, 2.49816, 0.45681, -0.54702, -5.40686, 3.31611, 0.44190, 2.46039, -0.07118, 1.85099, -0.21573, 0.52593, -0.68729, -0.02927, -0.14207, 1.17329, -0.14544, -0.03417, -0.73068, 0.13177, 0.09715],
  [...]
]
}
4. 生成的脚本放在本目录下，可以批量传入参数（--input_dir xxx/）,也可以单个文件转换（--input_file xxx.json）,输出目录为（--output）