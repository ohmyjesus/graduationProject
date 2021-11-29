function [sys,x0,str,ts] = plant2_test(t,x,u,flag)
% 此程序为快速非线性固定时间的仿真 
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case {2,4,9}
    sys=[];
  case 3
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;   
sys = simsizes(sizes);
x0  = [pi/60 1];
str = [];
ts  = [];

function sys=mdlDerivatives(t,x,u)
% 倒立摆状态方程
thd = sin(t);
dthd = cos(t);
ddthd = -sin(t);

ut = u(1);     
th = x(1);      % 摆角
dth = x(2);     % 摆速

% 参数的定义
f = -25*dth;
g = 133;

% d = 10*sin(t);

ddth = f + g * ut;

sys(1) = x(2);   
sys(2) = ddth;   

function sys=mdlOutputs(t,x,u)

sys(1) = x(1);
sys(2) = x(2);











