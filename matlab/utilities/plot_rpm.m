function plot_rpm(innerLoopData, rpm) 
% Plot RPM
time = innerLoopData(:,1);
figure;
plot(time, rpm(:,1), 'r','LineWidth',3)
hold on; plot(time, rpm(:,2), 'g','LineWidth',3)
hold on; plot(time, rpm(:,3), 'b','LineWidth',3)
hold on; plot(time, rpm(:,4), 'm','LineWidth',3)
hold on;
grid on;
set(gca,'fontsize',9,'FontWeight','bold');
title('RPM values vs Time','FontSize',11,'FontWeight','bold');
ylabel('RPM values','FontSize',9,'FontWeight','bold');
xlabel('Time (in seconds)','FontSize',9,'FontWeight','bold');
legend('motor 1', 'motor 2', 'motor 3', 'motor 4');
hold on;
end