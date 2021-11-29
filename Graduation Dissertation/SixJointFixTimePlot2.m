% 系统初始状态为 x0  = [0.3 0.3 0.3 0.3 0.3 0.3 0.1 0.1 0.1 0.1 0.1 0.1];
figure
% 第一关节角
subplot(1,2,1)
plot(qd1.Time,qd1.Data,q1.Time,q1.Data,'--',q11.Time,q11.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.15001 0.35009])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定曲线','本文方法','对比方法')

% 第二关节角
subplot(1,2,2)
plot(qd2.Time,qd2.Data,q2.Time,q2.Data,'--',q21.Time,q21.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.15001 0.35009])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定曲线','本文方法','对比方法')

% 第三关节角
figure
subplot(1,2,1)
plot(qd3.Time,qd3.Data,q3.Time,q3.Data,'--',q31.Time,q31.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.15001 0.35009])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定曲线','本文方法','对比方法')

% 第四关节角
subplot(1,2,2)
plot(qd4.Time,qd4.Data,q4.Time,q4.Data,'--',q41.Time,q41.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.15001 0.35009])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定曲线','本文方法','对比方法')

% 第五关节角
figure
subplot(1,2,1)
plot(qd5.Time,qd5.Data,q5.Time,q5.Data,'--',q51.Time,q51.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.15001 0.35009])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定曲线','本文方法','对比方法')

% 第六关节角
subplot(1,2,2)
plot(qd6.Time,qd6.Data,q6.Time,q6.Data,'--',q61.Time,q61.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.15001 0.35009])
% axis square
xlabel('时间/s','FontSize',40)
ylabel('角度/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('给定曲线','本文方法','对比方法')


% figure
% % 第一关节角误差
% subplot(3,2,1)
% plot(e1.Time,e1.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.01252 0.11273])
% axis square
% title('第一关节角度跟踪误差曲线','FontSize',11,'FontName','Adobe 黑体 Std R','FontWeight','bold')
% xlabel('时间(s)','FontSize',11)
% ylabel('角度误差(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % 第二关节角误差
% subplot(3,2,2)
% plot(e2.Time,e2.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.01252 0.11273])
% axis square
% title('第二关节角度跟踪误差曲线','FontSize',11,'FontName','Adobe 黑体 Std R','FontWeight','bold')
% xlabel('时间(s)','FontSize',11)
% ylabel('角度误差(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % 第三关节角误差
% subplot(3,2,3)
% plot(e3.Time,e3.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.01252 0.11273])
% axis square
% title('第三关节角度跟踪误差曲线','FontSize',11,'FontName','Adobe 黑体 Std R','FontWeight','bold')
% xlabel('时间(s)','FontSize',11)
% ylabel('角度误差(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % 第四关节角误差
% subplot(3,2,4)
% plot(e4.Time,e4.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.01252 0.11273])
% axis square
% title('第四关节角度跟踪误差曲线','FontSize',11,'FontName','Adobe 黑体 Std R','FontWeight','bold')
% xlabel('时间(s)','FontSize',11)
% ylabel('角度误差(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % 第五关节角误差
% subplot(3,2,5)
% plot(e5.Time,e5.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.01252 0.11273])
% axis square
% title('第五关节角度跟踪误差曲线','FontSize',11,'FontName','Adobe 黑体 Std R','FontWeight','bold')
% xlabel('时间(s)','FontSize',11)
% ylabel('角度误差(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % 第六关节角误差
% subplot(3,2,6)
% plot(e6.Time,e6.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.01252 0.11273])
% axis square
% title('第六关节角度跟踪误差曲线','FontSize',11,'FontName','Adobe 黑体 Std R','FontWeight','bold')
% xlabel('时间(s)','FontSize',11)
% ylabel('角度误差(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
