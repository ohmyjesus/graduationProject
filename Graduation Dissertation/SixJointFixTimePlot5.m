% 真实的fx和逼近的fx比较
figure 
plot(fx1.Time,result,'LineWidth',4)
hold on
plot(fjx1.Time,fjx1.Data,'--','LineWidth',4)
grid on
% axis square
xlabel('t/s','FontSize',40)
ylabel('f(x) and estimate f(x)','FontSize',40)
set(gca,'FontSize',40);
set(get(gca,'XLabel'),'FontSize',50);
set(get(gca,'YLabel'),'FontSize',50);
legend('True f','Estimate f')
% $\mathrm{f}_1\left( \mathrm{x} \right) \,\,\mathrm{and}  \mathrm{estimate\,\,\,} \mathrm{f}_1\left( \mathrm{x} \right) $
