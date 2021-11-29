function [sys,x0,str,ts] = SixJointFixTimeController(t,x,u,flag)
% 以下程序是 神经网络结合固定时间滑模  固定时间滑模采用的是NI Jun Kun的方法
switch flag
  case 0 %初始化
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1 %连续状态计算
    sys=mdlDerivatives(t,x,u);
  case {2,4,9} %离散状态计算，下一步仿真时刻，终止仿真设定
    sys=[];
  case 3 %输出信号计算
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
global c1 b1 node gama1 gama2 gama3 gama4 gama5 gama6 b2 b3 W_past1 W_new1 W_past2 W_new2 W_past3 W_new3 W_past4 W_new4 W_past5 W_new5 W_past6 W_new6
global q5_alpha11  q5_belta11 q5_M11 q5_n11 q5_P11 q5_Q11 q5_alpha12 q5_belta12 q5_M21 q5_n21 q5_P21 q5_Q21 q5_E2 q5_E4 
global q6_alpha11  q6_belta11 q6_M11 q6_n11 q6_P11 q6_Q11 q6_alpha12 q6_belta12 q6_M21 q6_n21 q6_P21 q6_Q21 q6_E2 q6_E4 
global alpha11  belta11 M11 n11 P11 Q11 alpha12 belta12 M21 n21 P21 Q21 E2 E4 

% 由于q5和q6跟踪效果较差，故单独给q5和q6使用一套控制器参数    增大p-q，m-n会降低固定时间
q5_alpha11 = 3; q5_belta11 = 3; q5_M11 = 11;q5_n11 = 3; q5_P11 = 7;q5_Q11 = 11;      % alpha1,alpha2,belta1,belta2 为正数
q5_alpha12 = 3; q5_belta12 = 27; q5_M21 = 11;q5_n21 = 3; q5_P21 = 3;q5_Q21 = 15;       % 其余为正奇数 且平均数也为正奇数 m1>n1 m2>n2 p1<q1 p2<q2 
q5_E2 = q5_P11/q5_Q11;
q5_E4 = q5_P21/q5_Q21;

q6_alpha11 = 3; q6_belta11 = 3; q6_M11 = 11;q6_n11 = 3; q6_P11 = 7;q6_Q11 = 11;      % alpha1,alpha2,belta1,belta2 为正数
q6_alpha12 = 3; q6_belta12 = 27; q6_M21 = 11;q6_n21 = 3; q6_P21 = 3;q6_Q21 = 15;       % 其余为正奇数 且平均数也为正奇数 m1>n1 m2>n2 p1<q1 p2<q2 
q6_E2 = q6_P11/q6_Q11;
q6_E4 = q6_P21/q6_Q21;

% q1~q4使用的控制器参数
alpha11 = 3; belta11 = 3; M11 = 11;n11 = 3; P11 = 7;Q11 = 11;      % alpha1,alpha2,belta1,belta2 为正数
alpha12 = 3; belta12 = 27; M21 = 11;n21 = 3; P21 = 3;Q21 = 15;       % 其余为正奇数 且平均数也为正奇数 m1>n1 m2>n2 p1<q1 p2<q2 
E2 = P11/Q11;
E4 = P21/Q21;

% 神经网络采用2-7-1结构
node = 7;              % 高斯函数的中心点矢量 维度 IN * MID  2*7
c1 = 1 * [-2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2];  
b1 = 30;  % 高斯函数的基宽  维度MID * 1   b的选择很重要 b越大 网路对输入的映射能力越大  
b2 = 10;    %q5使用的神经网络参数
b3 = 10;    %q6使用的神经网络参数
gama1 = 0.02   ;gama2 = 0.02;   gama3 = 0.05;  gama4 = 0.05;  gama5 = 0.05;  gama6 = 0.05;    % 学习率 越小学习越快，但可能导致过拟合
W_past1 = eye(7,1); W_past2 = eye(7,1); W_past3 = eye(7,1); W_past4 = eye(7,1); W_past5 = eye(7,1); W_past6 = eye(7,1);
W_new1 = W_past1; W_new2 = W_past2; W_new3 = W_past3; W_new4 = W_past4; W_new5 = W_past5; W_new6 = W_past6;
sizes = simsizes;
sizes.NumContStates  = node*6;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 30;   %设置系统输出的变量
sizes.NumInputs      = 36;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0*ones(node*6,1);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
%

function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c1 b1 node gama1 gama2 gama3 gama4 gama5 gama6 b2 b3 W_past1 W_new1 W_past2 W_new2 W_past3 W_new3 W_past4 W_new4 W_past5 W_new5 W_past6 W_new6
global q5_alpha11  q5_belta11 q5_M11 q5_n11 q5_P11 q5_Q11 q5_alpha12 q5_belta12 q5_M21 q5_n21 q5_P21 q5_Q21 q5_E2 q5_E4 
global q6_alpha11  q6_belta11 q6_M11 q6_n11 q6_P11 q6_Q11 q6_alpha12 q6_belta12 q6_M21 q6_n21 q6_P21 q6_Q21 q6_E2 q6_E4 
global alpha11  belta11 M11 n11 P11 Q11 alpha12 belta12 M21 n21 P21 Q21 E2 E4 
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
% 角度跟踪指令
qd1 = u(1);
qd2 = u(2);   
qd3 = u(3);
qd4 = u(4); 
qd5 = u(5);
qd6 = u(6); 

% 角速度跟踪指令
dqd1 = u(7);
dqd2 = u(8);   
dqd3 = u(9);
dqd4 = u(10); 
dqd5 = u(11);
dqd6 = u(12); 

% 角加速度跟踪指令
ddqd1 = u(13);
ddqd2 = u(14);   
ddqd3 = u(15);
ddqd4 = u(16); 
ddqd5 = u(17);
ddqd6 = u(18); 

q1 = u(19);
q2 = u(20);
q3 = u(21);
q4 = u(22);
q5 = u(23);
q6 = u(24);

dq1 = u(25);
dq2 = u(26);
dq3 = u(27);
dq4 = u(28);
dq5 = u(29);
dq6 = u(30);

% 角加速度
ddq1 = u(31);
ddq2 = u(32);
ddq3 = u(33);
ddq4 = u(34);
ddq5 = u(35);
ddq6 = u(36);

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
e3 = q3 - qd3;     
e4 = q4 - qd4;
e5 = q5 - qd5;      
e6 = q6 - qd6;

de1 = dq1 - dqd1;
de2 = dq2 - dqd2;
de3 = dq3 - dqd3;
de4 = dq4 - dqd4;
de5 = dq5 - dqd5;
de6 = dq6 - dqd6;

dde1 = ddq1 - ddqd1;
dde2 = ddq2 - ddqd2;
dde3 = ddq3 - ddqd3;
dde4 = ddq4 - ddqd4;
dde5 = ddq5 - ddqd5;
dde6 = ddq6 - ddqd6;

e = [e1; e2; e3; e4; e5; e6];
de = [de1; de2; de3; de4; de5; de6];

dq = [dq1; dq2; dq3; dq4; dq5; dq6];
q = [q1; q2; q3; q4; q5; q6];
dqd = [dqd1; dqd2; dqd3; dqd4; dqd5; dqd6;];
ddqd = [ddqd1; ddqd2; ddqd3; ddqd4; ddqd5; ddqd6];

q1_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e1)-1);
q2_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e2)-1);
q3_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e3)-1);
q4_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e4)-1);
q5_index = 1/2+q5_M11/(2*q5_n11)+(q5_M11/(2*q5_n11)-1/2)*sign(abs(e5)-1);
q6_index = 1/2+q6_M11/(2*q6_n11)+(q6_M11/(2*q6_n11)-1/2)*sign(abs(e6)-1);

% 滑模面
s1 = de1 + alpha11*abs(e1)^q1_index*sign(e1) + belta11*abs(e1)^E2*sign(e1);
s2 = de2 + alpha11*abs(e2)^q2_index*sign(e2) + belta11*abs(e2)^E2*sign(e2);
s3 = de3 + alpha11*abs(e3)^q3_index*sign(e3) + belta11*abs(e3)^E2*sign(e3);
s4 = de4 + alpha11*abs(e4)^q4_index*sign(e4) + belta11*abs(e4)^E2*sign(e4);
s5 = de5 + q5_alpha11*abs(e5)^q5_index*sign(e5) + q5_belta11*abs(e5)^q5_E2*sign(e5);
s6 = de6 + q6_alpha11*abs(e6)^q6_index*sign(e6) + q6_belta11*abs(e6)^q6_E2*sign(e6);

% s的指数项
s1_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s1)-1);
s2_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s2)-1);
s3_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s3)-1);
s4_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s4)-1);
s5_index = 1/2+(q5_M21/(2*q5_n21))+(q5_M21/(2*q5_n21)-1/2)*sign(abs(s5)-1);
s6_index = 1/2+(q6_M21/(2*q6_n21))+(q6_M21/(2*q6_n21)-1/2)*sign(abs(s6)-1);

% 第一个神经网络逼近f
Input = [e1;e2;e3;e4;e5;e6;de1;de2;de3;de4;de5;de6];
% --------------------------------------------- W权值的更新
hf1 = zeros(node , 1);   %7*1矩阵
hf2 = zeros(node , 1);   %7*1矩阵
hf3 = zeros(node , 1);   %7*1矩阵
hf4 = zeros(node , 1);   %7*1矩阵
hf5 = zeros(node , 1);   %7*1矩阵
hf6 = zeros(node , 1);   %7*1矩阵
for i =1:node
    hf1(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf2(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf3(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf4(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf5(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b2^2));
    hf6(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b3^2));
end
dw_fx1 = -1/gama1 * s1 * hf1; % 7*1矩阵
dw_fx2 = -1/gama2 * s2 * hf2; % 7*1矩阵
dw_fx3 = -1/gama3 * s3 * hf3; % 7*1矩阵
dw_fx4 = -1/gama4 * s4 * hf4; % 7*1矩阵
dw_fx5 = -1/gama5 * s5 * hf5; % 7*1矩阵
dw_fx6 = -1/gama6 * s6 * hf6; % 7*1矩阵

for i = 1:node
    sys(i)        = dw_fx1(i);
    sys(i+node)   = dw_fx2(i);
    sys(i+node*2) = dw_fx3(i);
    sys(i+node*3) = dw_fx4(i);
    sys(i+node*4) = dw_fx5(i);
    sys(i+node*5) = dw_fx6(i);
end
%
function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c1 b1 node gama1 gama2 gama3 gama4 gama5 gama6 b2 b3 W_past1 W_new1 W_past2 W_new2 W_past3 W_new3 W_past4 W_new4 W_past5 W_new5 W_past6 W_new6
global q5_alpha11  q5_belta11 q5_M11 q5_n11 q5_P11 q5_Q11 q5_alpha12 q5_belta12 q5_M21 q5_n21 q5_P21 q5_Q21 q5_E2 q5_E4 
global q6_alpha11  q6_belta11 q6_M11 q6_n11 q6_P11 q6_Q11 q6_alpha12 q6_belta12 q6_M21 q6_n21 q6_P21 q6_Q21 q6_E2 q6_E4 
global alpha11  belta11 M11 n11 P11 Q11 alpha12 belta12 M21 n21 P21 Q21 E2 E4 
% 角度跟踪指令
qd1 = u(1);
qd2 = u(2);   
qd3 = u(3);
qd4 = u(4); 
qd5 = u(5);
qd6 = u(6); 

% 角速度跟踪指令
dqd1 = u(7);
dqd2 = u(8);   
dqd3 = u(9);
dqd4 = u(10); 
dqd5 = u(11);
dqd6 = u(12); 

% 角加速度跟踪指令
ddqd1 = u(13);
ddqd2 = u(14);   
ddqd3 = u(15);
ddqd4 = u(16); 
ddqd5 = u(17);
ddqd6 = u(18); 

q1 = u(19);
q2 = u(20);
q3 = u(21);
q4 = u(22);
q5 = u(23);
q6 = u(24);

dq1 = u(25);
dq2 = u(26);
dq3 = u(27);
dq4 = u(28);
dq5 = u(29);
dq6 = u(30);

% 角加速度
ddq1 = u(31);
ddq2 = u(32);
ddq3 = u(33);
ddq4 = u(34);
ddq5 = u(35);
ddq6 = u(36);

ddq = [ddq1; ddq2; ddq3; ddq4; ddq5; ddq6];

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
e3 = q3 - qd3;     
e4 = q4 - qd4;
e5 = q5 - qd5;      
e6 = q6 - qd6;

de1 = dq1 - dqd1;
de2 = dq2 - dqd2;
de3 = dq3 - dqd3;
de4 = dq4 - dqd4;
de5 = dq5 - dqd5;
de6 = dq6 - dqd6;

dde1 = ddq1 - ddqd1;
dde2 = ddq2 - ddqd2;
dde3 = ddq3 - ddqd3;
dde4 = ddq4 - ddqd4;
dde5 = ddq5 - ddqd5;
dde6 = ddq6 - ddqd6;

e = [e1; e2; e3; e4; e5; e6];
de = [de1; de2; de3; de4; de5; de6];

dq = [dq1; dq2; dq3; dq4; dq5; dq6];
q = [q1; q2; q3; q4; q5; q6];
dqd = [dqd1; dqd2; dqd3; dqd4; dqd5; dqd6;];
ddqd = [ddqd1; ddqd2; ddqd3; ddqd4; ddqd5; ddqd6];

% 第一个神经网络逼近f
Input = [e1;e2;e3;e4;e5;e6;de1;de2;de3;de4;de5;de6];
% --------------------------------------------- W权值的更新
hf1 = zeros(node , 1);   %7*1矩阵
hf2 = zeros(node , 1);   %7*1矩阵
hf3 = zeros(node , 1);   %7*1矩阵
hf4 = zeros(node , 1);   %7*1矩阵
hf5 = zeros(node , 1);   %7*1矩阵
hf6 = zeros(node , 1);   %7*1矩阵
for i =1:node
    hf1(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf2(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf3(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf4(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf5(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b2^2));
    hf6(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b3^2));
end
WW1 = x(1:node);         % 7*1矩阵
WW2 = x(node+1:2*node);  % 7*1矩阵
WW3 = x(node*2+1:3*node);         % 7*1矩阵
WW4 = x(node*3+1:4*node);  % 7*1矩阵
WW5 = x(node*4+1:5*node);         % 7*1矩阵
WW6 = x(node*5+1:6*node);  % 7*1矩阵

fx1 = WW1' *hf1;
fx2 = WW2' *hf2;
fx3 = WW3' *hf3;
fx4 = WW4' *hf4;
fx5 = WW5' *hf5;
fx6 = WW6' *hf6;

% fx1 = 0;
% fx2 = 0;
% fx3 = 0;
% fx4 = 0;
% fx5 = 0;
% fx6 = 0;

%
% 参数的定义 此处需要名义矩阵M0 C0 G0
%创建V矩阵，需要用到连杆长度a质心位置r
%连杆长度
a1 = [0; 0; 0];
a2 = [0.1949; -0.0951; 0.2850];
a3 = [0.6137; -1.0938e-4; -0.003];
a4 = [0.2; -0.275; -0.1105];
a5 = [0; -0.0720; 0.3650];
a6 = [4.7173e-4; 0.0981; -0.0540];

%各个连杆的相对前一个关节坐标系的重心即质心
r1 = [0.1268; -0.0105; 0.2020];
r2 = [0.3482; -0.1597; 0.3146];
r3 = [0.4589; -0.1586; -0.1020];
r4 = [0.2721; -0.1454; -0.0036];
r5 = [-0.05; -0.0318; 0.2224];
r6 = [0.003; 0.0851; 0.0711];

%连杆质量
%六个连杆的质量
m1 = 17.9513;
m2 = 3.5484;
m3 = 7.3201;
m4 = 3.8682;
m5 = 0.7287;
m6 = 1;
m=[m1;m2;m3;m4;m5;m6];

%各个连杆的转动惯量
J1 = [ 0.5354  0.0131  -0.2059;
       0.0131  0.7118   0.0404;
      -0.2059   0.0404  0.5010];
        
J2 = [1.4282   0.0769  -0.3651;
    0.0769, 1.4960  0.2513;
    -0.3651, 0.2513  0.1778];
        
J3 = [7.9418    0.0198   -2.5248;
    0.0198    8.7609    0.0794;
   -2.5248    0.0794    0.8661];
        
J4 = [    4.9904   -0.0308   -2.7019;
   -0.0308    6.5104   -0.0619;
   -2.7019   -0.0619    1.5455];

J5 = [    0.8243   -0.0213   -0.6458;
   -0.0213    1.3309   -0.0270;
   -0.6458   -0.0270    0.5106];
        
J6 = [    0.9066   -0.0304   -0.7942;
   -0.0304    1.6033   -0.0346;
   -0.7942   -0.0346    0.7010];

%连杆扭转
alpha = [0, pi/2, 0, pi/2, pi/2, -pi/2];
theta = [q1, q2+pi/2, q3, q4, q5+pi/2, q6];

%变换矩阵A的定义 先绕x旋转再绕z轴旋转
A01 = [1 0 0;0 cos(alpha(1)) -sin(alpha(1));0 sin(alpha(1)) cos(alpha(1))]* [cos(theta(1)) -sin(theta(1)) 0;sin(theta(1))  cos(theta(1)) 0;0 0 1];
A12 = [1 0 0;0 cos(alpha(2)) -sin(alpha(2));0 sin(alpha(2)) cos(alpha(2))]* [cos(theta(2)) -sin(theta(2)) 0;sin(theta(2))  cos(theta(2)) 0;0 0 1];
A23 = [1 0 0;0 cos(alpha(3)) -sin(alpha(3));0 sin(alpha(3)) cos(alpha(3))]* [cos(theta(3)) -sin(theta(3)) 0;sin(theta(3))  cos(theta(3)) 0;0 0 1];
A34 = [1 0 0;0 cos(alpha(4)) -sin(alpha(4));0 sin(alpha(4)) cos(alpha(4))]* [cos(theta(4)) -sin(theta(4)) 0;sin(theta(4))  cos(theta(4)) 0;0 0 1];
A45 = [1 0 0;0 cos(alpha(5)) -sin(alpha(5));0 sin(alpha(5)) cos(alpha(5))]* [cos(theta(5)) -sin(theta(5)) 0;sin(theta(5))  cos(theta(5)) 0;0 0 1];
A56 = [1 0 0;0 cos(alpha(6)) -sin(alpha(6));0 sin(alpha(6)) cos(alpha(6))]* [cos(theta(6)) -sin(theta(6)) 0;sin(theta(6))  cos(theta(6)) 0;0 0 1];

A1=A01;
A2=A12*A01;
A3=A23*A12*A01;
A4=A34*A23*A12*A01;
A5=A45*A34*A23*A12*A01;
A6=A56*A45*A34*A23*A12*A01;

%定义VT中的叉乘矩阵
xA1r1=A1*r1;
chachengxA1r1=[0 -xA1r1(3) xA1r1(2);xA1r1(3) 0 -xA1r1(1);-xA1r1(2) xA1r1(1) 0];
xA2r2=A2*r2;
chachengxA2r2=[0 -xA2r2(3) xA2r2(2);xA2r2(3) 0 -xA2r2(1);-xA2r2(2) xA2r2(1) 0];
xA3r3=A3*r3;
chachengxA3r3=[0 -xA3r3(3) xA3r3(2);xA3r3(3) 0 -xA3r3(1);-xA3r3(2) xA3r3(1) 0];
xA4r4=A4*r4;
chachengxA4r4=[0 -xA4r4(3) xA4r4(2);xA4r4(3) 0 -xA4r4(1);-xA4r4(2) xA4r4(1) 0];
xA5r5=A5*r5;
chachengxA5r5=[0 -xA5r5(3) xA5r5(2);xA5r5(3) 0 -xA5r5(1);-xA5r5(2) xA5r5(1) 0];
xA6r6=A6*r6;
chachengxA6r6=[0 -xA6r6(3) xA6r6(2);xA6r6(3) 0 -xA6r6(1);-xA6r6(2) xA6r6(1) 0];

%定义叉乘矩阵A1*qx
xA1a2=A1*a2;  %3*1
chachengxA1a2=[0 -xA1a2(3) xA1a2(2);xA1a2(3) 0 -xA1a2(1);-xA1a2(2) xA1a2(1) 0];  %3*3矩阵
xA2a3=A2*a3;
chachengxA2a3=[0 -xA2a3(3) xA2a3(2);xA2a3(3) 0 -xA2a3(1);-xA2a3(2) xA2a3(1) 0];
xA3a4=A3*a4;
chachengxA3a4=[0 -xA3a4(3) xA3a4(2);xA3a4(3) 0 -xA3a4(1);-xA3a4(2) xA3a4(1) 0];
xA4a5=A4*a5;
chachengxA4a5=[0 -xA4a5(3) xA4a5(2);xA4a5(3) 0 -xA4a5(1);-xA4a5(2) xA4a5(1) 0];
xA5a6=A5*a6;
chachengxA5a6=[0 -xA5a6(3) xA5a6(2);xA5a6(3) 0 -xA5a6(1);-xA5a6(2) xA5a6(1) 0];

%定义VkT  3*18矩阵
V1T=-[chachengxA1r1 [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0] ];
V2T=-[chachengxA1a2 chachengxA2r2 [0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0]];
V3T=-[chachengxA1a2 chachengxA2a3 chachengxA3r3 [0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0]];
V4T=-[chachengxA1a2 chachengxA2a3 chachengxA3a4 chachengxA4r4 [0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0]];
V5T=-[chachengxA1a2 chachengxA2a3 chachengxA3a4 chachengxA4a5 chachengxA5r5 [0 0 0;0 0 0;0 0 0]];
V6T=-[chachengxA1a2 chachengxA2a3 chachengxA3a4 chachengxA4a5 chachengxA5a6 chachengxA6r6];
V1=V1T';  %18*3矩阵
V2=V2T';
V3=V3T';
V4=V4T';
V5=V5T';
V6=V6T';

%定义W和WT
E3=[0;0;1];
W1=[E3'*A1';0 0 0;0 0 0;0 0 0;0 0 0;0 0 0];   %6*3矩阵
W2=[E3'*A1';E3'*A2';0 0 0;0 0 0;0 0 0;0 0 0];
W3=[E3'*A1';E3'*A2';E3'*A3';0 0 0;0 0 0;0 0 0];
W4=[E3'*A1';E3'*A2';E3'*A3';E3'*A4';0 0 0;0 0 0];
W5=[E3'*A1';E3'*A2';E3'*A3';E3'*A4';E3'*A5';0 0 0];
W6=[E3'*A1';E3'*A2';E3'*A3';E3'*A4';E3'*A5';E3'*A6'];
W1T=W1';    %3*6矩阵
W2T=W2';
W3T=W3';
W4T=W4';
W5T=W5';
W6T=W6';

%定义W
W=[W1 W2 W3 W4 W5 W6];          %6*18矩阵
WT=[W1T;W2T;W3T;W4T;W5T;W6T];   %18*6矩阵


%定义角速度
w1=A1*E3*dq(1)     ;              %3*1 矩阵
w2=A1*E3*dq(1)+A2*E3*dq(2);
w3=A1*E3*dq(1)+A2*E3*dq(2)+A3*E3*dq(3);
w4=A1*E3*dq(1)+A2*E3*dq(2)+A3*E3*dq(3)+A4*E3*dq(4);
w5=A1*E3*dq(1)+A2*E3*dq(2)+A3*E3*dq(3)+A4*E3*dq(4)+A5*E3*dq(5);
w6=A1*E3*dq(1)+A2*E3*dq(2)+A3*E3*dq(3)+A4*E3*dq(4)+A5*E3*dq(5)+A6*E3*dq(6);

%定义角速度的叉乘矩阵
xw1=[0 -w1(3) w1(2);w1(3) 0 -w1(1);-w1(2) w1(1) 0];   %3*3矩阵
xw2=[0 -w2(3) w2(2);w2(3) 0 -w2(1);-w2(2) w2(1) 0];
xw3=[0 -w3(3) w3(2);w3(3) 0 -w3(1);-w3(2) w3(1) 0];
xw4=[0 -w4(3) w4(2);w4(3) 0 -w4(1);-w4(2) w4(1) 0];
xw5=[0 -w5(3) w5(2);w5(3) 0 -w5(1);-w5(2) w5(1) 0];
xw6=[0 -w6(3) w6(2);w6(3) 0 -w6(1);-w6(2) w6(1) 0];

%定义dVkT中的叉乘矩阵
xxw1A1r1=xw1*A1*r1;   %3*1矩阵
chachengxxw1A1r1=[0 -xxw1A1r1(3) xxw1A1r1(2);xxw1A1r1(3) 0 -xxw1A1r1(1);-xxw1A1r1(2) xxw1A1r1(1) 0];
xxw1A1a2=xw1*A1*a2;
chachengxxw1A1a2=[0 -xxw1A1a2(3) xxw1A1a2(2);xxw1A1a2(3) 0 -xxw1A1a2(1);-xxw1A1a2(2) xxw1A1a2(1) 0];
xxw2A2r2=xw2*A2*r2;
chachengxxw2A2r2=[0 -xxw2A2r2(3) xxw2A2r2(2);xxw2A2r2(3) 0 -xxw2A2r2(1);-xxw2A2r2(2) xxw2A2r2(1) 0];
xxw2A2a3=xw2*A2*a3;
chachengxxw2A2a3=[0 -xxw2A2a3(3) xxw2A2a3(2);xxw2A2a3(3) 0 -xxw2A2a3(1);-xxw2A2a3(2) xxw2A2a3(1) 0];
xxw3A3r3=xw3*A3*r3;
chachengxxw3A3r3=[0 -xxw3A3r3(3) xxw3A3r3(2);xxw3A3r3(3) 0 -xxw3A3r3(1);-xxw3A3r3(2) xxw3A3r3(1) 0];
xxw3A3a4=xw3*A3*a4;
chachengxxw3A3a4=[0 -xxw3A3a4(3) xxw3A3a4(2);xxw3A3a4(3) 0 -xxw3A3a4(1);-xxw3A3a4(2) xxw3A3a4(1) 0];
xxw4A4r4=xw4*A4*r4;
chachengxxw4A4r4=[0 -xxw4A4r4(3) xxw4A4r4(2);xxw4A4r4(3) 0 -xxw4A4r4(1);-xxw4A4r4(2) xxw4A4r4(1) 0];
xxw4A4a5=xw4*A4*a5;
chachengxxw4A4a5=[0 -xxw4A4a5(3) xxw4A4a5(2);xxw4A4a5(3) 0 -xxw4A4a5(1);-xxw4A4a5(2) xxw4A4a5(1) 0];
xxw5A5r5=xw5*A5*r5;
chachengxxw5A5r5=[0 -xxw5A5r5(3) xxw5A5r5(2);xxw5A5r5(3) 0 -xxw5A5r5(1);-xxw5A5r5(2) xxw5A5r5(1) 0];
xxw5A5a6=xw5*A5*a6;
chachengxxw5A5a6=[0 -xxw5A5a6(3) xxw5A5a6(2);xxw5A5a6(3) 0 -xxw5A5a6(1);-xxw5A5a6(2) xxw5A5a6(1) 0];
xxw6A6r6=xw6*A6*r6;
chachengxxw6A6r6=[0 -xxw6A6r6(3) xxw6A6r6(2);xxw6A6r6(3) 0 -xxw6A6r6(1);-xxw6A6r6(2) xxw6A6r6(1) 0];

%定义dVkT ------------------------3*18矩阵
dV1T=-[chachengxxw1A1r1 [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]];
dV2T=-[chachengxxw1A1a2 chachengxxw2A2r2 [0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0]];
dV3T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3r3 [0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0]];
dV4T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3a4 chachengxxw4A4r4 [0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0]];
dV5T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3a4 chachengxxw4A4a5 chachengxxw5A5r5 [0 0 0;0 0 0;0 0 0]];
dV6T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3a4 chachengxxw4A4a5 chachengxxw5A5a6 chachengxxw6A6r6];
dV1=dV1T';     %18*3矩阵
dV2=dV2T';
dV3=dV3T';
dV4=dV4T';
dV5=dV5T';
dV6=dV6T';

%定义A1e3的叉乘矩阵
xA1e3=A1*E3;
chachengxA1e3=[0 -xA1e3(3) xA1e3(2);xA1e3(3) 0 -xA1e3(1);-xA1e3(2) xA1e3(1) 0];
xA2e3=A2*E3;
chachengxA2e3=[0 -xA2e3(3) xA2e3(2);xA2e3(3) 0 -xA2e3(1);-xA2e3(2) xA2e3(1) 0];
xA3e3=A3*E3;
chachengxA3e3=[0 -xA3e3(3) xA3e3(2);xA3e3(3) 0 -xA3e3(1);-xA3e3(2) xA3e3(1) 0];
xA4e3=A4*E3;
chachengxA4e3=[0 -xA4e3(3) xA4e3(2);xA4e3(3) 0 -xA4e3(1);-xA4e3(2) xA4e3(1) 0];
xA5e3=A5*E3;
chachengxA5e3=[0 -xA5e3(3) xA5e3(2);xA5e3(3) 0 -xA5e3(1);-xA5e3(2) xA5e3(1) 0];
xA6e3=A6*E3;
chachengxA6e3=[0 -xA6e3(3) xA6e3(2);xA6e3(3) 0 -xA6e3(1);-xA6e3(2) xA6e3(1) 0];

%定义dW和dWT
dW1=[dq'*W1*chachengxA1e3;[0 0 0;0 0 0;0 0 0;0 0 0;0 0 0]];  %6*3矩阵
dW2=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;[0 0 0;0 0 0;0 0 0;0 0 0]];
dW3=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;[0 0 0;0 0 0;0 0 0]];
dW4=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;dq'*W4*chachengxA4e3;[0 0 0;0 0 0]];
dW5=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;dq'*W4*chachengxA4e3;dq'*W5*chachengxA5e3;[0 0 0]];
dW6=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;dq'*W4*chachengxA4e3;dq'*W5*chachengxA5e3;dq'*W6*chachengxA6e3];
dW=[dW1 dW2 dW3 dW4 dW5 dW6];  %6*18矩阵
dW1T=dW1';  %3*6矩阵
dW2T=dW2';
dW3T=dW3';
dW4T=dW4';
dW5T=dW5';
dW6T=dW6';
dWT=[dW1T;dW2T;dW3T;dW4T;dW5T;dW6T];          %18*6矩阵

%定义M矩阵
M1=m1*W*V1*V1T*WT+W1*J1*W1T;   %6*6矩阵
M2=m2*W*V2*V2T*WT+W2*J2*W2T;
M3=m3*W*V3*V3T*WT+W3*J3*W3T;
M4=m4*W*V4*V4T*WT+W4*J4*W4T;
M5=m5*W*V5*V5T*WT+W5*J5*W5T;
M6=m6*W*V6*V6T*WT+W6*J6*W6T;
M=M1+M2+M3+M4+M5+M6;   %6*6矩阵
MT=M';

%定义WkT和dq的叉乘矩阵
xW1Tdq=W1T*dq;
chachengxW1Tdq=[0 -xW1Tdq(3) xW1Tdq(2);xW1Tdq(3) 0 -xW1Tdq(1);-xW1Tdq(2) xW1Tdq(1) 0];
xW2Tdq=W2T*dq;
chachengxW2Tdq=[0 -xW2Tdq(3) xW2Tdq(2);xW2Tdq(3) 0 -xW2Tdq(1);-xW2Tdq(2) xW2Tdq(1) 0];
xW3Tdq=W3T*dq;
chachengxW3Tdq=[0 -xW3Tdq(3) xW3Tdq(2);xW3Tdq(3) 0 -xW3Tdq(1);-xW3Tdq(2) xW3Tdq(1) 0];
xW4Tdq=W4T*dq;
chachengxW4Tdq=[0 -xW4Tdq(3) xW4Tdq(2);xW4Tdq(3) 0 -xW4Tdq(1);-xW4Tdq(2) xW4Tdq(1) 0];
xW5Tdq=W5T*dq;
chachengxW5Tdq=[0 -xW5Tdq(3) xW5Tdq(2);xW5Tdq(3) 0 -xW5Tdq(1);-xW5Tdq(2) xW5Tdq(1) 0];
xW6Tdq=W6T*dq;
chachengxW6Tdq=[0 -xW6Tdq(3) xW6Tdq(2);xW6Tdq(3) 0 -xW6Tdq(1);-xW6Tdq(2) xW6Tdq(1) 0];

%定义N矩阵
N1=(m1*W*V1*(dV1T*WT+V1T*dWT)+W1*J1*dW1T+W1*chachengxW1Tdq*J1*W1T);  %6*6矩阵
N2=(m2*W*V2*(dV2T*WT+V2T*dWT)+W2*J2*dW2T+W2*chachengxW2Tdq*J2*W2T);
N3=(m3*W*V3*(dV3T*WT+V3T*dWT)+W3*J3*dW3T+W3*chachengxW3Tdq*J3*W3T);
N4=(m4*W*V4*(dV4T*WT+V4T*dWT)+W4*J4*dW4T+W4*chachengxW4Tdq*J4*W4T);
N5=(m5*W*V5*(dV5T*WT+V5T*dWT)+W5*J5*dW5T+W5*chachengxW5Tdq*J5*W5T);
N6=(m6*W*V6*(dV6T*WT+V6T*dWT)+W6*J6*dW6T+W6*chachengxW6Tdq*J6*W6T);
N=N1+N2+N3+N4+N5+N6;   %6*6矩阵

% 名义模型M0 N0 
M0 =  1.2*M;
N0 =  1.2*N;
g = inv(M0);
H = -inv(M0)*N0*dq;

% e的指数项
q1_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e1)-1);
q2_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e2)-1);
q3_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e3)-1);
q4_index = 1/2+M11/(2*n11)+(M11/(2*n11)-1/2)*sign(abs(e4)-1);
q5_index = 1/2+q5_M11/(2*q5_n11)+(q5_M11/(2*q5_n11)-1/2)*sign(abs(e5)-1);
q6_index = 1/2+q6_M11/(2*q6_n11)+(q6_M11/(2*q6_n11)-1/2)*sign(abs(e6)-1);

% 滑模面的定义
s1 = de1 + alpha11*abs(e1)^q1_index*sign(e1) + belta11*abs(e1)^E2*sign(e1);
s2 = de2 + alpha11*abs(e2)^q2_index*sign(e2) + belta11*abs(e2)^E2*sign(e2);
s3 = de3 + alpha11*abs(e3)^q3_index*sign(e3) + belta11*abs(e3)^E2*sign(e3);
s4 = de4 + alpha11*abs(e4)^q4_index*sign(e4) + belta11*abs(e4)^E2*sign(e4);
s5 = de5 + q5_alpha11*abs(e5)^q5_index*sign(e5) + q5_belta11*abs(e5)^q5_E2*sign(e5);
s6 = de6 + q6_alpha11*abs(e6)^q6_index*sign(e6) + q6_belta11*abs(e6)^q6_E2*sign(e6);

% s的指数项
s1_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s1)-1);
s2_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s2)-1);
s3_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s3)-1);
s4_index = 1/2+(M21/(2*n21))+(M21/(2*n21)-1/2)*sign(abs(s4)-1);
s5_index = 1/2+(q5_M21/(2*q5_n21))+(q5_M21/(2*q5_n21)-1/2)*sign(abs(s5)-1);
s6_index = 1/2+(q6_M21/(2*q6_n21))+(q6_M21/(2*q6_n21)-1/2)*sign(abs(s6)-1);

% 趋近律 滑模面的导数ds
ds1 = -alpha12*abs(s1)^s1_index*sign(s1) - belta12*abs(s1)^(E4)*sign(s1);
ds2 = -alpha12*abs(s2)^s2_index*sign(s2) - belta12*abs(s2)^(E4)*sign(s2);
ds3 = -alpha12*abs(s3)^s3_index*sign(s3) - belta12*abs(s3)^(E4)*sign(s3);
ds4 = -alpha12*abs(s4)^s4_index*sign(s4) - belta12*abs(s4)^(E4)*sign(s4);
ds5 = -q5_alpha12*abs(s5)^s5_index*sign(s5) - q5_belta12*abs(s5)^(q5_E4)*sign(s5);
ds6 = -q6_alpha12*abs(s6)^s6_index*sign(s6) - q6_belta12*abs(s6)^(q6_E4)*sign(s6);

% % % 饱和函数的判断  为切换项  可通过调整切换项的增益h来减小抖振
temp1 = belta11*E2*abs(e1)^(E2-1)*sign(e1)*de1;
temp2 = belta11*E2*abs(e2)^(E2-1)*sign(e2)*de2;
temp3 = belta11*E2*abs(e3)^(E2-1)*sign(e3)*de3;
temp4 = belta11*E2*abs(e4)^(E2-1)*sign(e4)*de4;
temp5 = q5_belta11*q5_E2*abs(e5)^(q5_E2-1)*sign(e5)*de5;
temp6 = q6_belta11*q6_E2*abs(e6)^(q6_E2-1)*sign(e6)*de6;

temp = [temp1;temp2;temp3;temp4;temp5;temp6];

% 固定切换增益h
h1 = 100; h2 = 100; h3 = 100; h4 = 100; h5 = 100; h6 = 100;

if abs(temp1)<h1
    sat1 = temp1;
else 
    sat1 = h1*sign(temp1);
end
if abs(temp2)<h2
    sat2 = temp2;
else 
    sat2 = h2*sign(temp2);
end
if abs(temp3)<h3
    sat3 = temp3;
else 
    sat3 = h3*sign(temp3);
end
if abs(temp4)<h4
    sat4 = temp4;
else 
    sat4 = h4*sign(temp4);
end
if abs(temp5)<h5
    sat5 = temp5;
else 
    sat5 = h5*sign(temp5);
end
if abs(temp6)<h6
    sat6 = temp6;
else 
    sat6 = h6*sign(temp6);
end

T11 = fx1 + alpha11*q1_index* (abs(e1)^(q1_index-1)*sign(e1))*de1;      % 第一关节
T21 = fx2 + alpha11*q2_index* (abs(e2)^(q2_index-1)*sign(e2))*de2;      % 第二关节
T31 = fx3 + alpha11*q3_index* (abs(e3)^(q3_index-1)*sign(e3))*de3;      % 第三关节
T41 = fx4 + alpha11*q4_index* (abs(e4)^(q4_index-1)*sign(e4))*de4;      % 第四关节
T51 = fx5 + q5_alpha11*q5_index* (abs(e5)^(q5_index-1)*sign(e5))*de5;      % 第五关节
T61 = fx6 + q6_alpha11*q6_index* (abs(e6)^(q6_index-1)*sign(e6))*de6;      % 第六关节

T2 = [sat1;sat2;sat3;sat4;sat5;sat6];

T13 = alpha12*abs(s1)^s1_index*sign(s1) + belta12*abs(s1)^E4*sign(s1);
T23 = alpha12*abs(s2)^s2_index*sign(s2) + belta12*abs(s2)^E4*sign(s2);
T33 = alpha12*abs(s3)^s3_index*sign(s3) + belta12*abs(s3)^E4*sign(s3);
T43 = alpha12*abs(s4)^s4_index*sign(s4) + belta12*abs(s4)^E4*sign(s4);
T53 = q5_alpha12*abs(s5)^s5_index*sign(s5) + q5_belta12*abs(s5)^q5_E4*sign(s5);
T63 = q6_alpha12*abs(s6)^s6_index*sign(s6) + q6_belta12*abs(s6)^q6_E4*sign(s6);

T_1 = [T11;T21;T31;T41;T51;T61];
T_3 = [T13;T23;T33;T43;T53;T63];

tau = -inv(g) * (T_1 + T2 + T_3 + H - ddqd);

d = 0.5*[2*sin(t);2*sin(t);2*sin(t);2*sin(t);2*sin(t);2*sin(t)];

% 输出真正的f(x)
trueFx = inv(M0) * (0.2*M0 * ddq + 0.2*N0 * dq + d);

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = tau(3);
sys(4) = tau(4);
sys(5) = tau(5);
sys(6) = tau(6);
sys(7) = s1;
sys(8) = s2;
sys(9) = s3;
sys(10) = s4;
sys(11) = s5;
sys(12) = s6;
sys(13) = fx1;
sys(14) = fx2;
sys(15) = fx3;
sys(16) = fx4;
sys(17) = fx5;
sys(18) = fx6;
sys(19) = temp1;
sys(20) = temp2;
sys(21) = temp3;
sys(22) = temp4;
sys(23) = temp5;
sys(24) = temp6;

% 真正的fx输出
sys(25) = trueFx(1);
sys(26) = trueFx(2);
sys(27) = trueFx(3);
sys(28) = trueFx(4);
sys(29) = trueFx(5);
sys(30) = trueFx(6);

W_past1 = W_new1;
W_past2 = W_new2;
W_past3 = W_new3;
W_past4 = W_new4;
W_past5 = W_new5;
W_past6 = W_new6;











