% ϵͳ��ʼ״̬Ϊ x0  = [0.09 0.09 0.09 0.09 0.09 0.09 0.1 0.1 0.1 0.1 0.1 0.1];
figure
% ��һ�ؽڽ�
subplot(1,2,1)
plot(qd1.Time,qd1.Data,q1.Time,q1.Data,'--',q11.Time,q11.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.125 0.125])
% axis square
xlabel('ʱ��/s','FontSize',40)
ylabel('�Ƕ�/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('��������','���ķ���','�Աȷ���')

% �ڶ��ؽڽ�
subplot(1,2,2)
plot(qd2.Time,qd2.Data,q2.Time,q2.Data,'--',q21.Time,q21.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.125 0.125])
% axis square
xlabel('ʱ��/s','FontSize',40)
ylabel('�Ƕ�/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('��������','���ķ���','�Աȷ���')

% �����ؽڽ�
figure
subplot(1,2,1)
plot(qd3.Time,qd3.Data,q3.Time,q3.Data,'--',q31.Time,q31.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.125 0.125])
% axis square
xlabel('ʱ��/s','FontSize',40)
ylabel('�Ƕ�/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('��������','���ķ���','�Աȷ���')

% ���Ĺؽڽ�
subplot(1,2,2)
plot(qd4.Time,qd4.Data,q4.Time,q4.Data,'--',q41.Time,q41.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.125 0.125])
% axis square
xlabel('ʱ��/s','FontSize',40)
ylabel('�Ƕ�/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('��������','���ķ���','�Աȷ���')

% ����ؽڽ�
figure
subplot(1,2,1)
plot(qd5.Time,qd5.Data,q5.Time,q5.Data,'--',q51.Time,q51.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.125 0.125])
% axis square
xlabel('ʱ��/s','FontSize',40)
ylabel('�Ƕ�/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('��������','���ķ���','�Աȷ���')

% �����ؽڽ�
subplot(1,2,2)
plot(qd6.Time,qd6.Data,q6.Time,q6.Data,'--',q61.Time,q61.Data,':','LineWidth',4)
grid on
xlim([0 3])
ylim([-0.125 0.125])
% axis square
xlabel('ʱ��/s','FontSize',40)
ylabel('�Ƕ�/rad','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('��������','���ķ���','�Աȷ���')


% figure
% % ��һ�ؽڽ����
% subplot(3,2,1)
% plot(e1.Time,e1.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.1125 0.0125])
% axis square
% title('��һ�ؽڽǶȸ����������','FontSize',11,'FontName','Adobe ���� Std R','FontWeight','bold')
% xlabel('ʱ��(s)','FontSize',11)
% ylabel('�Ƕ����(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % �ڶ��ؽڽ����
% subplot(3,2,2)
% plot(e2.Time,e2.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.1125 0.0125])
% axis square
% title('�ڶ��ؽڽǶȸ����������','FontSize',11,'FontName','Adobe ���� Std R','FontWeight','bold')
% xlabel('ʱ��(s)','FontSize',11)
% ylabel('�Ƕ����(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % �����ؽڽ����
% subplot(3,2,3)
% plot(e3.Time,e3.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.1125 0.0125])
% axis square
% title('�����ؽڽǶȸ����������','FontSize',11,'FontName','Adobe ���� Std R','FontWeight','bold')
% xlabel('ʱ��(s)','FontSize',11)
% ylabel('�Ƕ����(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % ���Ĺؽڽ����
% subplot(3,2,4)
% plot(e4.Time,e4.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.1125 0.0125])
% axis square
% title('���ĹؽڽǶȸ����������','FontSize',11,'FontName','Adobe ���� Std R','FontWeight','bold')
% xlabel('ʱ��(s)','FontSize',11)
% ylabel('�Ƕ����(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % ����ؽڽ����
% subplot(3,2,5)
% plot(e5.Time,e5.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.1125 0.0125])
% axis square
% title('����ؽڽǶȸ����������','FontSize',11,'FontName','Adobe ���� Std R','FontWeight','bold')
% xlabel('ʱ��(s)','FontSize',11)
% ylabel('�Ƕ����(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');
% 
% % �����ؽڽ����
% subplot(3,5,6)
% plot(e6.Time,e6.Data,'LineWidth',2)
% grid on
% xlim([0 3])
% ylim([-0.1125 0.0125])
% axis square
% title('�����ؽڽǶȸ����������','FontSize',11,'FontName','Adobe ���� Std R','FontWeight','bold')
% xlabel('ʱ��(s)','FontSize',11)
% ylabel('�Ƕ����(rad)','FontSize',14)
% set(gca,'FontSize',11);
% set(get(gca,'XLabel'),'FontSize',11,'FontWeight','bold');
% set(get(gca,'YLabel'),'FontSize',11,'FontWeight','bold');