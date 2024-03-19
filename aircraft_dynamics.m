function state_dot = aircraft_dynamics(t, state, elevator_input, ailerons_input, rudder_input)
    % 飞行器动态方程
    
    % 仿真主要参数
    d2r = pi/180;
    tspan = 1;
    Mass = 18;
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
    
    % 状态变量
    % Euler角
    phi = state(1);
    theta = state(2);
    psi = state(3);
    
    % 角速度
    p = state(4);
    q = state(5);
    r = state(6);
    
    % 速度
    u = state(7);
    v = state(8);
    w = state(9);
    
    % 位置
    x = state(10);
    y = state(11);
    z = state(12);
    
    % 控制输入
    del_e = interp1(tspan, elevator_input, t);
    del_a = interp1(tspan, ailerons_input, t);
    del_r = interp1(tspan, rudder_input, t);
    
    % 这里根据提供的参数计算飞行器的动态行为
    
    % 计算力和力矩
    
    % 飞行器的动态微分方程
    state_dot = zeros(12, 1);
    state_dot(1) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    state_dot(2) = q*cos(phi) - r*sin(phi);
    state_dot(3) = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
    state_dot(4) = 0; % 假设为零
    state_dot(5) = 0; % 假设为零
    state_dot(6) = 0; % 假设为零
    state_dot(7) = 0; % 假设为零
    state_dot(8) = 0; % 假设为零
    state_dot(9) = 0; % 假设为零
    state_dot(10) = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    state_dot(11) = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    state_dot(12) = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);
    
end
