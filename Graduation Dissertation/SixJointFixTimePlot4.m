% 有无RBF补偿的绘图程序  e1是有RBF  e11是无RBF
figure 
plot(e1.Time,e1.Data,'--','LineWidth',4)
hold on
plot(e11.Time,e11.Data,'LineWidth',4)
grid on
axis square
xlabel('t/s','FontSize',40)
ylabel('e1/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('无补偿','有补偿')

figure
plot(e2.Time,e2.Data,'--','LineWidth',4)
hold on
plot(e21.Time,e21.Data,'LineWidth',4)
grid on
axis square
xlabel('t/s','FontSize',40)
ylabel('e2/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('无补偿','有补偿')

figure
plot(e3.Time,e3.Data,'--','LineWidth',4)
hold on
plot(e31.Time,e31.Data,'LineWidth',4)
grid on
axis square
xlabel('t/s','FontSize',40)
ylabel('e3/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('无补偿','有补偿')

figure
plot(e4.Time,e4.Data,'--','LineWidth',4)
hold on
plot(e41.Time,e41.Data,'LineWidth',4)
grid on
axis square
xlabel('t/s','FontSize',40)
ylabel('e4/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('无补偿','有补偿')

figure
plot(e5.Time,e5.Data,'--','LineWidth',4)
hold on
plot(e51.Time,e51.Data,'LineWidth',4)
grid on
axis square
xlabel('t/s','FontSize',40)
ylabel('e5/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('无补偿','有补偿')

figure
plot(e6.Time,e6.Data,'--','LineWidth',4)
hold on
plot(e61.Time,e61.Data,'LineWidth',4)
grid on
axis square
xlabel('t/s','FontSize',40)
ylabel('e6rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('无补偿','有补偿')