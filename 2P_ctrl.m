function [sys,x0,str,ts] = spacemodel(t,x,u,flag) 
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;  
sizes.NumOutputs     = 2;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  =[];
str = [];
ts  = [0 0];
function sys=mdlOutputs(t,x,u)
xd=u(1);
yd=u(2);
xd=t;dxd=1;
yd=sin(0.5*xd)+0.5*xd+1;
dyd=0.5*cos(0.5*xd)+0.5;
x1=u(3);y1=u(4);

xe=x1-xd;
ye=y1-yd;

a=3.0;p1=10;
b=3.0;p2=10;

u1=dxd-a*tanh(p1*xe);
u2=dyd-b*tanh(p2*ye);

thd=atan(u2/u1);
v=u1/cos(thd);

sys(1)=v;
sys(2)=thd;