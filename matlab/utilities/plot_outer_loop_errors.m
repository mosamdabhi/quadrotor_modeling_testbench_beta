function plot_outer_loop_errors(Y, states)
    
    % Plot desired and actual position
    grid on;
    figure;
    subplot(3,1,1);plot(states(:,1),states(:,8), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),Y(:,8), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,2);plot(states(:,1),states(:,9) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),Y(:,9) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,3);plot(states(:,1),states(:,10) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1),Y(:,10) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    
    x_pos = states(:,8);
    y_pos = states(:,9);
    z_pos = states(:,10);
    save ('../matlab/sensor_models/DataLog/x_pos.mat', 'x_pos'); 
    save ('../matlab/sensor_models/DataLog/y_pos.mat', 'y_pos');
    save ('../matlab/sensor_models/DataLog/z_pos.mat', 'z_pos');
    
    % Plot desired and actual linear velocities
    figure;
    subplot(3,1,1);plot(states(:,1),states(:,2), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{x}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),Y(:,11), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{x}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,2);plot(states(:,1),states(:,3), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{y}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),Y(:,12), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{y}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,3);plot(states(:,1),states(:,4), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{z}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1),Y(:,13), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{z}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
end
