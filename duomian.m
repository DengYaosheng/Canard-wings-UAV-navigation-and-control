clear;
clc;

% 仿真主要参数
d2r = pi/180;

Step_Size = 0.001;

Mass = 18;
u_trim = [0.20383 0.54103 0 0];
UNIT_GRAV = 9.81;
Sw = 1.485;
b = 3.0; %Wing span翼展
c_bar = 0.552; %Chord弦长
I_xx = 2.9;
I_yy = 3.1;
I_zz = 2.5;
I_xz = 0;
I_TENSOR = [ I_xx,  0.0, -I_xz ;                           
              0.0, I_yy,   0.0 ;
            -I_xz,  0.0,  I_zz ] ;
Init_Euler = [0 5 0]*d2r;
Init_Rate = [0 0 0]*d2r;
Init_Vel = [18.6923 0 1.6352]; %ub vb wb
Init_Pos = [0 0 0]; %xe ye ze

max_T = 97.02;
min_T = 0;
Max_del_T = 1;
Min_del_T = 0;
Max_del_e = 30.0;                               
Max_del_a = 30.0;                            
Max_del_r = 30.0;

% 时间参数
t_total = 10; % 总仿真时间
tspan = 0:Step_Size:t_total;

% 初始状态
init_state = [Init_Euler, Init_Rate, Init_Vel, Init_Pos];

% 控制输入
% 这里只是一个示例，你需要根据实际情况设置舵偏角随时间的变化曲线
% 这里我随机生成一个舵偏角随时间的变化曲线
num_steps = length(tspan);
elevator_input = zeros(1, num_steps);
ailerons_input = zeros(1, num_steps);
rudder_input = zeros(1, num_steps);

for i = 1:num_steps
    elevator_input(i) = Max_del_e * sin(2*pi*0.1*tspan(i));
    ailerons_input(i) = Max_del_a * cos(2*pi*0.05*tspan(i));
    rudder_input(i) = Max_del_r * sin(2*pi*0.2*tspan(i));
end

% 飞行控制系统模拟
[t, state] = ode45(@(t, state) aircraft_dynamics(t, state, ...
    elevator_input, ailerons_input, rudder_input), tspan, init_state);

% 提取模拟结果
Euler_angles = state(:, 1:3);
Angular_rates = state(:, 4:6);
Velocities = state(:, 7:9);
Positions = state(:, 10:12);

% 绘制舵偏角随时间的变化曲线
figure;
plot(t, elevator_input, 'r', 'LineWidth', 2);
hold on;
plot(t, ailerons_input, 'g', 'LineWidth', 2);
plot(t, rudder_input, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Control input');
legend('Elevator', 'Ailerons', 'Rudder');
title('Control Inputs vs Time');

% 仿真结果可视化（如需要）
% 这里你可以根据需求绘制飞机的姿态、速度、位置等可视化图形
