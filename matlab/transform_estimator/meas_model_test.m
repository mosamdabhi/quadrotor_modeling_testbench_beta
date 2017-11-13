function meas_model_test(quat_, pos_, zEst, mRes, t)

q_ = zeros(4, length(quat_.x));
euler_ = zeros(3, length(quat_.x));
eulerEst = zeros(3, length(quat_.x));

for i = 1:1:length(quat_.x)
    q_(:, i) = [quat_.x(i); quat_.y(i); quat_.z(i); quat_.w(i)];
    euler_(:, i) = QuatToZYX(q_(:, i));
    eulerEst(:, i) = QuatToZYX(zEst(1:4, i));
end

    % plot angular information
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(pos_)),eulerEst(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(t(1:length(pos_)), euler_(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    subplot(3,1,1);plot(t(1:length(pos_)), mRes(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Measurement Model', 'Vicon', 'Measurement residual');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(pos_)),eulerEst(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(t(1:length(pos_)),euler_(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    subplot(3,1,2);plot(t(1:length(pos_)), mRes(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Measurement Model', 'Vicon', 'Measurement residual');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(pos_)),eulerEst(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(t(1:length(pos_)),euler_(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    subplot(3,1,3);plot(t(1:length(pos_)), mRes(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Measurement Model', 'Vicon', 'Measurement residual');
    
    % plot position information
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(zEst(5,:))), zEst(5,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(t(1:length(zEst(5,:))), pos_(:,1), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    subplot(3,1,1);plot(t(1:length(pos_)), mRes(4,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Measurement Model', 'Vicon', 'Measurement residual');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(zEst(5,:))), zEst(6,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(t(1:length(pos_)), pos_(:,2), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    subplot(3,1,2);plot(t(1:length(pos_)), mRes(5,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold'); 
    legend('Measurement Model', 'Vicon', 'Measurement residual');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(zEst(5,:))), zEst(7,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(t(1:length(pos_)), pos_(:,3), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    subplot(3,1,3);plot(t(1:length(pos_)), mRes(6,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Measurement Model', 'Vicon', 'Measurement residual');
end