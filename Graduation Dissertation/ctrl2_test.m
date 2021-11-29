function [sys,x0,str,ts] = ctrl2_test(t,x,u,flag)
% �˳���Ϊ���͹̶�ʱ������� by nijunkang
switch flag
      case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
      case 1 %����״̬����
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
% ���������2-5-1�ṹ
c = 2*[-1.5 -1 -0.5  -0 0.5 1 1.5;
       -1.5 -1 -0.5  -0 0.5 1 1.5];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*7
b = 10;  % ��˹�����Ļ���  ά��MID * 1  1*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
node = 7;
gama = 20;
sizes = simsizes;
sizes.NumContStates  = node;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 3;   %����ϵͳ����ı���
sizes.NumInputs      = 2;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = zeros(1,node);        % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b gama  node
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
th = u(1);
dth = u(2);

% �����Ķ���
alpha1 = 3;
belta1 = 3;
alpha2 = 5;
belta2 = 5;
m1 = 9;
n1 = 5;
m2 = 7;
n2 = 3;
p1 = 7;
q1 = 11;
p2 = 5;
q2 = 9;

% ��ģ��
E2 = p1/q1;
E4 = p2/q2;
h = 100;        % h����0
p = 1/2+m1/(2*n1)+(m1/(2*n1)-1/2)*sign(abs(th)-1);
s = dth + alpha1*abs(th)^p*sign(th) + belta1*abs(th)^E2*sign(th);

Input = [th; dth];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = x(1:node);
w_updata = gama * s  * h;

for i = 1:node
    sys(i) = w_updata(i);
end



function sys=mdlOutputs(t,x,u)
global c b gama node
th = u(1);
dth = u(2);

Input = [th; dth];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = x(1:node);
fx = W'*h;      % fx�Ĺ���ֵ
    
% �����Ķ���
alpha1 = 3;
belta1 = 3;
alpha2 = 5;
belta2 = 5;
m1 = 9;
n1 = 5;
m2 = 7;
n2 = 3;
p1 = 7;
q1 = 11;
p2 = 5;
q2 = 9;

% ϵͳ״̬���̵Ķ���
f = -25*dth;
g = 133;

% ��ģ��Ķ���
E2 = p1/q1;
E4 = p2/q2;
h = 100;        % h����0
p = 1/2+m1/(2*n1)+(m1/(2*n1)-1/2)*sign(abs(th)-1);
s = dth + alpha1*abs(th)^p*sign(th) + belta1*abs(th)^E2*sign(th);
q = 1/2+(m2/(2*n2))+(m2/(2*n2)-1/2)*sign(abs(s)-1);

% ���ͺ������ж�
temp = belta1*p1/q1*abs(th)^(E2-1)*sign(th)*dth;
if abs(temp)<h
    sat = temp;
else 
    sat = h*sign(temp);
end

T1 = fx + alpha1*p*abs(th)^(p-1)*sign(th)*dth;
T2 = sat;
T3 = alpha2*abs(s)^q*sign(s) + belta2*abs(s)^E4*sign(s);
ut = -1/g * (T1+T2+T3) ;
sys(1) = ut;
sys(2) = p;
sys(3) = (p-1)*sign(th)*dth;

