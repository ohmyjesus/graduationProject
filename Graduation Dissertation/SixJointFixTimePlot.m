% 系统初始状态为 x0  = [0.05 0.05 0.05 0.05 0.05 0.05 0.1 0.1 0.1 0.1 0.1 0.1];
% 角度跟踪轨迹和角度跟踪误差轨迹
figure
% 第一关节角
subplot(1,2,1)
plot(qd1.Time,qd1.Data,'--',q1.Time,q1.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.125 0.125])
axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定轨迹','实际轨迹')

% 第二关节角
subplot(1,2,2)
plot(qd2.Time,qd2.Data,'--',q2.Time,q2.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.125 0.125])
axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定轨迹','实际轨迹')

figure
% 第三关节角
subplot(1,2,1)
plot(qd3.Time,qd3.Data,'--',q3.Time,q3.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.125 0.125])
axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定轨迹','实际轨迹')

% 第四关节角
subplot(1,2,2)
plot(qd4.Time,qd4.Data,'--',q4.Time,q4.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.125 0.125])
axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定轨迹','实际轨迹')

figure
% 第五关节角
subplot(1,2,1)
plot(qd5.Time,qd5.Data,'--',q5.Time,q5.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.125 0.125])
axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定轨迹','实际轨迹')

% 第六关节角
subplot(1,2,2)
plot(qd6.Time,qd6.Data,'--',q6.Time,q6.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.125 0.125])
axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定轨迹','实际轨迹')

figure
% 第一关节角误差
subplot(1,2,1)
plot(e1.Time,e1.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.05625 0.00625])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度误差/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);

% 第二关节角误差
subplot(1,2,2)
plot(e2.Time,e2.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.05625 0.00625])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度误差/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);

figure
% 第三关节角误差
subplot(1,2,1)
plot(e1.Time,e1.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.05625 0.00625])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度误差/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);

% 第四关节角误差
subplot(1,2,2)
plot(e2.Time,e2.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.05625 0.00625])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度误差/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);

figure
% 第五关节角误差
subplot(1,2,1)
plot(e1.Time,e1.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.05625 0.00625])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度误差/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);

% 第六关节角误差
subplot(1,2,2)
plot(e2.Time,e2.Data,'LineWidth',3)
grid on
xlim([0 2])
ylim([-0.05625 0.00625])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度误差/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);