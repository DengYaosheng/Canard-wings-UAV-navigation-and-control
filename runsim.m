close all;
clear;

addpath('utils');

%% 飞行参数
% trajhandle = @traj_line;
% trajhandle = @traj_helix;

%% 制导率
%% 模拟一个四旋翼动画
 trajhandle = @traj_generator;
 waypoints = [0    0   0;
              1    1   1;
              2    0   2;
              3    -1  1;
              1    0   0;
              ]';
 trajhandle([],[],waypoints);


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);

tt = 1:1:size(state, 1);
figure(1)
plot(tt,state(:,8))
xlabel('t')
ylabel('攻角')
grid on
figure(2)
plot(tt,state(:,9))
xlabel('t')
ylabel('滚转')
grid on
figure(2)
plot(tt,state(:,10))
xlabel('t')
ylabel('偏航')
grid on
plot(tt,state(:,11))
xlabel('t')
ylabel('d攻角')
grid on
figure(2)
plot(tt,state(:,12))
xlabel('t')
ylabel('d滚转')
grid on
figure(2)
plot(tt,state(:,13))
xlabel('t')
ylabel('d偏航')
grid on
run main.m
