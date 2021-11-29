function [sys,x0,str,ts] = YHXDynamaic(t,x,u,flag)
% 以下程序为凯恩方法的六关节动力学建模
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
sizes.NumContStates  = 12;   %变量个数
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 12;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   
sys = simsizes(sizes);
x0  = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];
str = [];
ts  = [0 0];

function sys=mdlDerivatives(t,x,u)
%定义关节角和关节角速度x(1)~x(12)的代表含义
% 角度输入
q1=x(1);
q2=x(2);
q3=x(3);
q4=x(4);
q5=x(5);
q6=x(6);

% 角速度输入
dq1=x(7);
dq2=x(8);
dq3=x(9);
dq4=x(10);
dq5=x(11);
dq6=x(12);

th = [q1;q2;q3;q4;q5;q6]';
dth = [dq1;dq2;dq3;dq4;dq5;dq6]';
%----定义旋转矩阵A并取绕Z轴旋转分量----
%部分DH参数
alpha_i_1 = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
theta = [th(1), th(2), th(3)-pi/2, th(4), th(5), th(6)];

A0i = 1;  %循环叠加求第i个连杆相对与0坐标系的旋转矩阵
e3 = [0; 0; 1];  %取Z轴分量
for j = 1 : 6
    A0i = A0i * [cos(theta(j)) -sin(theta(j)) 0;
                        cos(alpha_i_1(j))*sin(theta(j)) cos(alpha_i_1(j))*cos(theta(j)) -sin(alpha_i_1(j));
                        sin(alpha_i_1(j))*sin(theta(j)) sin(alpha_i_1(j))*cos(theta(j)) cos(alpha_i_1(j))];
    A0{j} = A0i;
end

m = [17.9513, 3.5484, 7.3201, 3.8682, 0.7287, 1]; %六个连杆的质量
%细胞数组J为各个连杆的转动惯量
J{1} = [ 0.5354           0.0131           -0.2059;
            0.0131           0.7118        0.0404;
            -0.2059           0.0404           0.5010];
        
J{2} = [0.5044, -0.0164, -0.0021;
    -0.0164, 0.0144, -0.0304;
    -0.0021, -0.0304, 0.5091];
        
J{3} = [0.2601, -0.1844, -0.0883;
    -0.1844, 0.2780, -0.0850;
    -0.0883, -0.0850, 0.3979];
        
J{4} = [0.1544, -0.0001, -0.0143;
    -0.0001, 0.1527, -0.0051;
    -0.0143, -0.0051, 0.0224];
        
J{5} = [0.0055, 0, 0;
    0, 0.0040, -0.0015;
    0, -0.0015, 0.0028];
        
J{6} = [0.0042, 0, 0;
    0, 0.0042, 0;
    0, 0, 0.0017];
%创建W矩阵
W{1} = [e3'*A0{1}'; 
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
        
W{2} = [e3'*A0{1}'; 
            e3'*A0{2}';
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
  
W{3} = [e3'*A0{1}'; 
            e3'*A0{2}';
            e3'*A0{3}';
            0 0 0;
            0 0 0;
            0 0 0];
        
W{4} = [e3'*A0{1}'; 
            e3'*A0{2}';
            e3'*A0{3}';
            e3'*A0{4}';
            0 0 0;
            0 0 0];
        
W{5} = [e3'*A0{1}'; 
            e3'*A0{2}';
            e3'*A0{3}';
            e3'*A0{4}';
            e3'*A0{5}';
            0 0 0];
        
W{6} = [e3'*A0{1}'; 
            e3'*A0{2}';
            e3'*A0{3}';
            e3'*A0{4}';
            e3'*A0{5}';
            e3'*A0{6}'];
        
WT{1} = W{1}';
WT{2} = W{2}';
WT{3} = W{3}';
WT{4} = W{4}';
WT{5} = W{5}';
WT{6} = W{6}';

%创建V矩阵，需要用到连杆长度q质心位置r
a{1} = [0; 0; 0];
a{2} = [0.1949; -0.0951; 0.2850];
a{3} = [1.0938e-04; -0.6137; 0.0030];
a{4} = [0.2000; 0.2750; 0.1105];
a{5} = [0; 0.0320; 0.3650];
a{6} = [-4.7162e-04; 0.0981; 0.0540];

r{1} = [0.0467; -0.0104; 0.1100];
r{2} = [-0.0155; -0.2895; -0.0268];
r{3} = [0.1366; 0.1311; 0.0806];
r{4} = [0.0350; 0.0044; 0.1474];
r{5} = [3.9663e-06; -0.0019; 0.0540];
r{6} = [0; 0; 0.05];

%求叉乘矩阵前，先求积，再转为叉乘矩阵
%A0q的叉乘矩阵
tmp1 = A0{1}*a{2};
tmp2 = A0{2}*a{3};
tmp3 = A0{3}*a{4};
tmp4 = A0{4}*a{5};
tmp5 = A0{5}*a{6};
A01q2x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
A02q3x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
A03q4x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
A04q5x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
A05q6x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];

%A0r的叉乘矩阵
tmp1 = A0{1}*r{1};
tmp2 = A0{2}*r{2};
tmp3 = A0{3}*r{3};
tmp4 = A0{4}*r{4};
tmp5 = A0{5}*r{5};
tmp6 = A0{6}*r{6};
A01r1x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
A02r2x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
A03r3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
A04r4x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
A05r5x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
A06r6x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];
              
%创建ViT矩阵
VT{1} = -[A01r1x , [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{2} = -[A01q2x , A02r2x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{3} = -[A01q2x , A02q3x, A03r3x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{4} = -[A01q2x , A02q3x, A03q4x, A04r4x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{5} = -[A01q2x , A02q3x, A03q4x, A04q5x, A05r5x, [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{6} = -[A01q2x , A02q3x, A03q4x, A04q5x, A05q6x, A06r6x];

%A0e3的叉乘矩阵
tmp1 = A0{1}*e3;
tmp2 = A0{2}*e3;
tmp3 = A0{3}*e3;
tmp4 = A0{4}*e3;
tmp5 = A0{5}*e3;
tmp6 = A0{6}*e3;
A01e3x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
A02e3x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
A03e3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
A04e3x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
A05e3x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
A06e3x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];

%创建W的导数矩阵dW
dW{1} = [dth*W{1}*A01e3x ;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
        
dW{2} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
        
dW{3} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            0 0 0;
            0 0 0;
            0 0 0];
        
dW{4} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            dth*W{4}*A04e3x;
            0 0 0;
            0 0 0];
        
dW{5} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            dth*W{4}*A04e3x;
            dth*W{5}*A05e3x;
            0 0 0];
        
dW{6} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            dth*W{4}*A04e3x;
            dth*W{5}*A05e3x;
            dth*W{6}*A06e3x];
        
dWT{1} = dW{1}';
dWT{2} = dW{2}';
dWT{3} = dW{3}';
dWT{4} = dW{4}';
dWT{5} = dW{5}';
dWT{6} = dW{6}';

%omega_i的叉乘矩阵
tmp1 = [0, 0, dth(1)];
tmp2 = [0, 0, dth(2)];
tmp3 = [0, 0, dth(3)];
tmp4 = [0, 0, dth(4)];
tmp5 = [0, 0, dth(5)];
tmp6 = [0, 0, dth(6)];
omega1x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
omega2x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
omega3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
omega4x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
omega5x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
omega6x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];
              
%omega_ixA0i-1qi的叉乘矩阵
tmp1 = omega1x*A0{1}*a{2};
tmp2 = omega2x*A0{2}*a{3};
tmp3 = omega3x*A0{3}*a{4};
tmp4 = omega4x*A0{4}*a{5};
tmp5 = omega5x*A0{5}*a{6};
omega1xA01q2x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
omega2xA02q3x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
omega3xA03q4x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
omega4xA04q5x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
omega5xA05q6x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];       
              
%omega_ixA0iri的叉乘矩阵
tmp1 = omega1x*A0{1}*r{1};
tmp2 = omega2x*A0{2}*r{2};
tmp3 = omega3x*A0{3}*r{3};
tmp4 = omega4x*A0{4}*r{4};
tmp5 = omega5x*A0{5}*r{5};
tmp6 = omega6x*A0{6}*r{6};
omega1xA01r1x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
omega2xA02r2x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
omega3xA03r3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
omega4xA04r4x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
omega5xA05r5x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
omega6xA06r6x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];     
              
%创建dViT矩阵
dVT{1} = -[omega1xA01r1x , [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{2} = -[omega1xA01q2x , omega2xA02r2x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{3} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03r3x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{4} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04r4x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{5} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04q5x, omega5xA05r5x, [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{6} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04q5x, omega5xA05q6x, omega6xA06r6x];

%WkTdth的叉乘矩阵
tmp1 = WT{1}*dth';
tmp2 = WT{2}*dth';
tmp3 = WT{3}*dth';
tmp4 = WT{4}*dth';
tmp5 = WT{5}*dth';
tmp6 = WT{6}*dth';
WTdth{1} = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
WTdth{2} = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
WTdth{3} = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
WTdth{4} = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
WTdth{5} = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
WTdth{6} = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];     
%参数M
M = 0;
for i = 1 : 6
    M = M + m(i)*(cell2mat(W)*VT{i}'*VT{i}*cell2mat(WT')) + W{i}*J{i}*W{i}';
end

%参数N
N = 0;
for i = 1 : 6
    N = N + m(i)*(cell2mat(W)*VT{i}'*(dVT{i}*cell2mat(WT')+VT{i}*cell2mat(dWT')))+W{i}*J{i}*dWT{i}+W{i}*WTdth{i}*J{i}*WT{i};
end

%六个力矩分量
tao(1) = u(1);   
tao(2) = u(2);   
tao(3) = u(3);   
tao(4) = u(4);   
tao(5) = u(5);   
tao(6) = u(6);   
tao=[tao(1);tao(2);tao(3);tao(4);tao(5);tao(6)];    %6*1矩阵  

% f = [u(7);u(8);u(9);u(10);u(11);u(12)];
% 外部扰动
dt = 0.1*[2*sin(t);2*sin(t);2*sin(t);2*sin(t);2*sin(t);2*sin(t)];
% 库伦摩擦力
F = 0.02*sign(dth);

%动力学方程
ddth=inv(M) * (tao - N * dth' - dt - F); %6*1矩阵

%x1=q1    x2=q2   ....  x7=dq1  ...x12=dq6
sys(1)=x(7);   %q1
sys(2)=x(8);   %q2
sys(3)=x(9);   %q3
sys(4)=x(10);  %q4
sys(5)=x(11);  %q5
sys(6)=x(12);  %q6
sys(7)=ddth(1);    
sys(8)=ddth(2);    
sys(9)=ddth(3);     
sys(10)=ddth(4);    
sys(11)=ddth(5);    
sys(12)=ddth(6);    

function sys=mdlOutputs(t,x,u)
sys(1)=x(1);   %q1
sys(2)=x(2);   %q2
sys(3)=x(3) ;  %q3
sys(4)=x(4) ;  %q4
sys(5)=x(5) ;  %q5
sys(6)=x(6) ;  %q6
sys(7)=x(7) ;  %dq1
sys(8)=x(8) ;  %dq2
sys(9)=x(9) ;  %dq3
sys(10)=x(10); %dq4
sys(11)=x(11); %dq5
sys(12)=x(12); %dq6

