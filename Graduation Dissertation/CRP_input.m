function [sys,x0,str,ts] = CRP_input(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1
    sys=mdlDerivatives(t,x,u);
case 3
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
sizes.NumOutputs     = 18;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(t,x,u)
% ?Ƕȸ???ָ??
qd1 = 0.1*cos(2*t);
qd2 = 0.1*cos(2*t);
qd3 = 0.1*cos(2*t);
qd4 = 0.1*cos(2*t);
qd5 = 0.1*cos(2*t);
qd6 = 0.1*cos(2*t);

% ???ٶȸ???ָ??
dqd1 =  -0.1*2*sin(2*t);
dqd2 =  -0.1*2*sin(2*t);
dqd3 =  -0.1*2*sin(2*t);
dqd4 =  -0.1*2*sin(2*t);
dqd5 =  -0.1*2*sin(2*t);
dqd6 =  -0.1*2*sin(2*t);

% ?Ǽ??ٶȸ???ָ??
ddqd1 = -0.1*2*2*cos(2*t);
ddqd2 = -0.1*2*2*cos(2*t);
ddqd3 = -0.1*2*2*cos(2*t);
ddqd4 = -0.1*2*2*cos(2*t);
ddqd5 = -0.1*2*2*cos(2*t);
ddqd6 = -0.1*2*2*cos(2*t);

sys(1) = qd1;
sys(2) = qd2;
sys(3) = qd3;
sys(4) = qd4;
sys(5) = qd5;
sys(6) = qd6;
sys(7) = dqd1;
sys(8) = dqd2;
sys(9) = dqd3;
sys(10) = dqd4;
sys(11) = dqd5;
sys(12) = dqd6;
sys(13) = ddqd1;
sys(14) = ddqd2;
sys(15) = ddqd3;
sys(16) = ddqd4;
sys(17) = ddqd5;
sys(18) = ddqd6;

