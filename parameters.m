% clear;clc;
%% 仿真主要参数
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