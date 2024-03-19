function [sys,x0,str,ts] = Canard_aero(t,x,u,flag)
% This function is used to calculate the aero force and moment by body axis.
% Gravity and engine propultion are not included.
% Input (10 dimension): da,de,dr,alpha,beta,p,q,r,V,qbar
% Output (6 dimension): Fx,Fy,Fz,L_aero,M_aero,N_aero
%
%%
%
switch flag,
  case 0,   % Initialization %
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1,  % Derivatives %
    sys=mdlDerivatives(t,x,u);
  case 2,   % Update %
    sys=mdlUpdate(t,x,u);
  case 3,   % Outputs %
    sys=mdlOutputs(t,x,u);
  case 4,   % GetTimeOfNextVarHit %
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,   % Terminate %
    sys=mdlTerminate(t,x,u);
  otherwise  % Unexpected flags %
    ctrlMsgUtils.error('Controllib:general:UnexpectedError',['Unhandled flag = ',num2str(flag)]);

end
% end csfunc
%%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 10;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];   
str = [];
ts  = [];
% end mdlInitializeSizes
%%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
function sys=mdlDerivatives(t,x,u)

sys=[];
% end mdlDerivatives
%%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step requirements.
%=============================================================================
function sys=mdlUpdate(t,x,u)

sys = [];
% end mdlUpdate
%%
%=============================================================================
% mdlOutputs
% Return the outputs.
%=============================================================================
function sys=mdlOutputs(t,x,u)
% F18 body parameters
% global cbar bar S;
cbar = 0.552;      %           % the average length of wing chord    unit: m 
bar  = 3.0;      %            % the length of wing span    unit: m 
S    = 1.485;     %     % Reference area of the wing  unit: m^2 
% Get the input parameters
da = u(1);      % Aileron angle     unit:degree
de = u(2);      % Elevator angle    unit:degre
dr = u(3);      % Rudder angle      unit:degre
alpha = u(4);   % Angle of attack   unit:degre
beta = u(5);    % Sideslip angle    unit:degre
p = u(6);       % Roll rate         unit:degree per second
q = u(7);       % Pitch rate        unit:degree per second
r = u(8);       % Yaw rate          unit:degree per second
V = u(9);       % Airspeed          unit:meter per second
qbar = u(10);   % Q = 0.5*AirDens*AirVelosity   unit:pa


clift0 = 0.1792; clift_alp = 0.011; clift_q = 0.071; clift_de = 0.0142;
clift = clift0 + clift_alp*alpha + (cbar/(2*V))*clift_q*q + clift_de*de;

cd0 = 0.013; cd_alp = 0.0021;
cd = cd0 + cd_alp*alpha;

cy_b = -0.0184; cy_p = 2.5853e-4; cy_r = 0.038; cy_dr = 0.0037;
cy = cy_b*beta + (bar/(2*V))*cy_p*p + (bar/(2*V))*cy_r*r + cy_dr*dr;

croll_b = -0.0016; croll_p = -0.0071; croll_r = 0.0023; croll_da = -0.0012; croll_dr = 2.2387e-4;
croll = croll_b*beta + (bar/(2*V))*croll_p*p + (bar/(2*V))*croll_r*r + croll_da*da + croll_dr*dr;

cm0 = -0.0037; cm_q = -0.082; cm_de = -0.0155;
cm = cm0 + (cbar/(2*V))*cm_q*q + cm_de*de;

cn_b = 0.0017; cn_p = -0.0011; cn_r = -0.0031; cn_da = 3.8171e-5; cn_dr = -0.0012;
cn = cn_b*beta + (bar/(2*V))*cn_p*p + (bar/(2*V))*cn_r*r + cn_da*da + cn_dr*dr;

L = qbar*S*clift;
D = qbar*S*cd;
Y = qbar*S*cy;
% Calculate 3 axises' components of aero force.
sina = sind(alpha);
cosa = cosd(alpha);
sinb = sind(beta);
cosb = cosd(beta);
%global Fx Fy Fz L_aero M_aero N_aero;
Fx = L*sina - Y*cosa*sinb - D*cosa*cosb;  % the x component of aero force 
Fy = Y*cosb - D*sinb;                     % the y component of aero force 
Fz = -L*cosa - Y*sina*sinb - D*sina*cosb; % the z component of aero force 
% Calculate the roll pitch and yaw moment.
L_aero = qbar*S*bar*croll;
M_aero = qbar*S*cbar*cm;                      
N_aero = qbar*S*bar*cn;
% output table
sys = [Fx Fy Fz L_aero M_aero N_aero];
%end mdlOutputs
%%
%=============================================================================
% mdlOutputs
% Return the time of the next hit.
%=============================================================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
% endmdlGetTimeOfNextVarHit
%%
%=============================================================================
% mdlOutputs
% Perform any end of simulation tasks.
%=============================================================================
function sys=mdlTerminate(t,x,u)

sys = [];
% endmdlTerminate