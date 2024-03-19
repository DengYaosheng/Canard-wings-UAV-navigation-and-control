%Hypersonic Simulation
clc; close all; clear all;
%constants
DTOR = pi/180; RTOD = 180/pi; 
L_F = 0.75; L_NO = 0.65; L_A = 0.90;
T1U = 3*DTOR; T1L = 20*DTOR; T2 = 20.62*DTOR;
h1 = 0.95; hi = 0.15; h4 = 1.25; width = 1.5;
%Cross-sectional areas
Ad = 0.33; An = 4.5; Ae = (hi*width)*An;
%Body constant values (xb,yb,xcm,ycm........)
xf = 0.77; zf = 0.07; x_no = 0.065;
x_inlet = 1.14; z_inlet = 0.1415;
za = 0.006; xa = 0.71;
zc = -0.10; xc = -0.70;
m = 1500; %kg
Iyy = 99000; %kg*m^2
H = 0.1; %step size
g = 9.81;

%end of constants

 %% NOTES
 %can use indices for functions so M(1) and M mean the same, only...
 %locations need to match up.
 %%
%control surface Se
Se = 0.0524; %>0

%initialize
M_1 = 9.5392;
h = 8.8984e3;
alpha = 0.1571;

[p_1 ,rho_1, T_1 ,a] = pvnrt(h);
V_1 = 2.8147e+03;
u = V_1*cos(alpha); w = V_1*sin(alpha);

% loop begins here
for i = 1:1
    
    [p_1 ,rho_1, T_1 ,a] = pvnrt(h(i)); %free-stream values [1]
    %flight speeds
    %constant b
    b = -(1/M_1(i)^2)*(M_1(i)^2+2) - 1.4*sin(T1L+alpha(i));
    %shock angle
    [TH_S, DEL] = (shockangle(M_1(i),T1L,alpha(i),RTOD)); THT_S = TH_S*DTOR;

    %downstream of oblique shock [ds]
    [p_ds, T_ds, M_ds] = oblique_down(p_1,T_1,M_1(i), THT_S, DEL);
    %AERO FORCE and ENGINE INLET TURNING FORCE
    F_xf = -p_ds*L_F*tan(T1L); F_zf = -p_ds*L_F; %in N
    M_f = F_xf*zf - F_zf*xf; %Nm
    F_xinlet = 1.4*M_ds^2*p_ds*(1-cos(T1L+alpha(i)))*(Ae/b)*(1/(Ad*An));
    F_zinlet = 1.4*M_ds^2*p_ds*sin(T1L+alpha(i))*(Ae/b)*(1/(Ad*An));
    M_inlet = z_inlet*F_xinlet - x_inlet*F_zinlet;

    %flow past diffuser.. at the combustor inlet [2]
    [M_2,p_2,T_2] = comb_inlet(M_ds,p_ds,T_ds, Ad);

    %AERO FORCE p_2
    F_zn = p_2*L_NO; M_n = -F_zn*x_no;
    %rayleigh flow parameters
    R = 287; cp = 1003;

    [m_dot,M_3,p_3,T_stag_3,T_3] = rayleigh_flow(M_2,T_2,p_2,R,hi,width);

    %expansion nozzle - from 3 to 4
    [M_e,p_e,T_e,THR] = exp_and_thrust(M_3,T_3,p_3,Ae,Ad,An,R,b,w,m_dot,V_1,p_1,p_ds);
    %AFTBODY PRESSURE (inc lift): NOTE: p_inf = p_1
    F_xa = p_1*L_A*(p_e/p_1)*log(p_e/p_1)*tan(T2+T1U)/(-1 + p_e/p_1);
    F_za = p_1*L_A*(p_e/p_1)*(log(p_e/p_1)/(-1 + p_e/p_1));
    M_a = za*F_xa - xa*F_za;

    %CONTROL SURFACE MODEL
    [F_xc, F_zc, Mc] = ctrlsurf(Se(i),alpha(i),M_1(i),p_1,RTOD,DTOR,zc,xc);

    %SUM UP FORCES AND MOMENTS
    EF_X(i) = F_xf + F_xinlet+THR(i)+ F_xa + F_xc;
    EF_Z(i) = F_zf + F_zinlet + F_za + F_zc;
    EM = M_f + M_inlet + M_n + M_a + Mc;

    %STATE VECTOR INITIALIZE q
    q(i) = EM/Iyy; %alpha(1) = see up
    
    %RK constants
    
end


    
