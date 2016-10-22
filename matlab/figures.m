load('d_pos.mat');
load('state.mat');

%
load('control.mat');

[m1,n1] = size(state);

xwidth =1200;
ywidth = 600;

figure(1) %tracking error
hold on; grid on;box on;
plot(d_pos(2,:)-state(5,:),'--r','LineWidth',3);
plot(d_pos(3,:)-state(6,:),'-.b','LineWidth',3);
plot(d_pos(4,:)-state(7,:),'g','LineWidth',3);
xlim([0 30000])
set(gca,'XTick',0:5000:30000);
set(gca,'XTickLabel',{'0','5','10','15','20','25','30'})
xlabel('Time(s)','fontsize',20)
ylabel('Tracking error (m)','fontsize',20)
leg = legend('y_1-y_1^d','y_2-y_2^d','y_3-y_3^d');
set(leg,'Location','NorthEast','fontsize',20);

hFig = figure(1);
set(gcf,'PaperPositionMode','auto')
set(hFig, 'Position', [0 0 xwidth ywidth])
print -depsc2 results/error.eps;      



figure(2) %thrust
hold on; grid on;
box on;
plot(control(2,:),'r','LineWidth',3);
ylim([0 15]);
xlim([0 30000]);
set(gca,'YTick',0:5:15);
set(gca,'XTick',0:5000:30000);
set(gca,'XTickLabel',{'0','5','10','15','20','25','30'})
xlabel('Time (s)','fontsize',20)
ylabel('Thrust (N)','fontsize',20)
hFig = figure(2);
set(gcf,'PaperPositionMode','auto')
set(hFig, 'Position', [0 0 xwidth ywidth])
print -depsc2 results/lambda.eps; 

figure(3) %tau
hold on; grid on;
box on;
plot(control(3,:),'--r','LineWidth',3);
plot(control(4,:),'-.b','LineWidth',3);
plot(control(5,:),'g','LineWidth',3);
%ylim([-25 15]);
xlim([0 30000]);
%set(gca,'YTick',-25:5:15);
set(gca,'XTick',0:5000:30000);
set(gca,'XTickLabel',{'0','5','10','15','20','25','30'})
xlabel('Time (s)','fontsize',20)
ylabel('Tau (Nm)','fontsize',20)
leg = legend('\tau_1','\tau_2','\tau_3');
set(leg,'Location','NorthEast','fontsize',20);
hFig = figure(3);
set(gcf,'PaperPositionMode','auto')
set(hFig, 'Position', [0 0 xwidth ywidth])
print -depsc2 results/tau.eps; 
