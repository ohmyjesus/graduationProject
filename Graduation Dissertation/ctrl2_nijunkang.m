function [sys,x0,str,ts] = ctrl2_nijunkang(t,x,u,flag)
% 此程序为新型固定时间控制器 by nijunkang
switch flag
      case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
      case 1 %连续状态计算
    sys=mdlDerivatives(t,x,u);
      case {2,4,9}
    sys=[];
      case 3
    sys=mdlOutputs(t,x,u);
      otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
function [sys,x0,str,ts]=mdlInitializeSizes
global c b node gama
% 神经网络采用2-5-1结构
c = 3*[-1.5 -1 -0.5  -0 0.5 1 1.5;
       -1.5 -1 -0.5  -0 0.5 1 1.5];               % 高斯函数的中心点矢量 维度 IN * MID  2*7
b = 20;  % 高斯函数的基宽  维度MID * 1  1*1   b的选择很重要 b越大 网路对输入的映射能力越大  
node = 7;
gama = 20;
sizes = simsizes;
sizes.NumContStates  = node;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 4;   %设置系统输出的变量
sizes.NumInputs      = 2;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0*zeros(1,node);        % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c b gama  node
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
thd = sin(1/2*t);
dthd = 1/2*cos(1/2*t);
ddthd = -1/2*1/2*sin(1/2*t);

th = u(1);
dth = u(2);

% 参数的定义
alpha1 = 3; belta1 = 3; m1 = 9;n1 = 5; p1 = 7;q1 = 11;      % alpha1,alpha2,belta1,belta2 为正奇数
alpha2 = 5; belta2 = 5; m2 = 7;n2 = 3; p2 = 3;q2 = 11;       % 其余为正奇数 且平均数也为正奇数

e = thd - th;
de = dthd - dth;

% 滑模面
E2 = p1/q1;
E4 = p2/q2;
h = 100;        % h大于0
p = 1/2+m1/(2*n1)+(m1/(2*n1)-1/2)*sign(abs(-e)-1);
s = -de + alpha1*abs(-e)^p*sign(-e) + belta1*abs(-e)^E2*sign(-e);

Input = [e; de];
hw = zeros(node , 1);   %5*1矩阵
for i =1:node
    hw(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = x(1:node);
w_updata = gama * s  * hw;

for i = 1:node
    sys(i) = w_updata(i);
end

function sys=mdlOutputs(t,x,u)
global c b gama node
% 跟踪轨迹
thd = sin(1/2*t);
dthd = 1/2*cos(1/2*t);
ddthd = -1/2*1/2*sin(1/2*t);

th = u(1);
dth = u(2);

e = thd - th;
de = dthd - dth;

Input = [e; de];
hw = zeros(node , 1);   %5*1矩阵
for i =1:node
    hw(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = x(1:node);
fx = W'*hw;      % fx的估计值
    
% 参数的定义
alpha1 = 3; belta1 = 3; m1 = 9;n1 = 5; p1 = 7;q1 = 11;      % alpha1,alpha2,belta1,belta2 为正奇数
alpha2 = 5; belta2 = 5; m2 = 7;n2 = 3; p2 = 3;q2 = 11;       % 其余为正奇数 且平均数也为正奇数

% 系统状态方程的定义
f = -25*dth ;
g = 133;

% 滑模面的定义
E2 = p1/q1;
E4 = p2/q2;
h = 100;        % h大于0
p = 1/2+m1/(2*n1)+(m1/(2*n1)-1/2)*sign(abs(-e)-1);
s = -de + alpha1*abs(-e)^p*sign(-e) + belta1*abs(-e)^E2*sign(-e);
q = 1/2+(m2/(2*n2))+(m2/(2*n2)-1/2)*sign(abs(s)-1);

% 饱和函数的判断
temp = belta1*p1/q1*abs(-e)^(E2-1)*sign(-e)*-de;
if abs(temp)<h
    sat = temp;
else 
    sat = h*sign(temp);
end

T1 = fx + alpha1*p* (abs(-e)^(p-1)*sign(-e))*-de;
T2 = sat;
T3 = alpha2*abs(s)^q*sign(s) + belta2*abs(s)^E4*sign(s);
ut = -1/g * (T1+T2+T3) ;

sys(1) = ut;
sys(2) = belta2*abs(s)^E4*sign(s);
sys(3) = alpha2*abs(s)^q*sign(s);
sys(4) = s;

