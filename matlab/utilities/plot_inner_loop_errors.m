function plot_inner_loop_errors(Y, states)
    r2d = 180/pi;
    % Plot desired and actual euler angles
    grid on;
    figure(1);
    subplot(3,1,1);plot(states(:,1),states(:,11)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);

    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),Y(:,8)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,2);plot(states(:,1),states(:,12)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),Y(:,9)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,3);plot(states(:,1),states(:,13)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1),Y(:,10)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    
    % Plot desired and actual angular velocities
    figure(2);
    subplot(3,1,1);plot(states(:,1),states(:,5)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('p', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),Y(:,11)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('p', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,2);plot(states(:,1),states(:,6)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('q', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),Y(:,12)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('q', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,3);plot(states(:,1),states(:,7)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('r', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1),Y(:,13)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('r', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
end
