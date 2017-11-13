function plot_state(states, quat_, pos_, zEst, mRes, mRes_, QuadModel, t)
% Generate actual euler angles from saved data
q_ = zeros(4, length(quat_.x));
euler_ = zeros(3, length(quat_.x));
eulerEst = zeros(3, length(quat_.x));
IMUVicOrient = zeros(3, length(quat_.x));
IMUVicOrientDes = zeros(3, length(quat_.x));
IMUVicPos = zeros(3, length(quat_.x));
IMUVicPosDes = zeros(3, length(quat_.x));

for i = 1:1:length(quat_.x)
    q_(:, i) = [quat_.x(i); quat_.y(i); quat_.z(i); quat_.w(i)];
    euler_(:, i) = QuatToZYX(q_(:, i));
    eulerEst(:, i) = QuatToZYX(zEst(1:4, i));
    IMUVicOrient(:,i) = QuatToZYX(states(i, 12:15));
    IMUVicOrientDes(:,i) = QuatToZYX(quat_destruct(QuadModel.IMUViconTransform.quat));
    IMUVicPos(:,i) = states(i, 16:18);
    IMUVicPosDes(:,i) = QuadModel.IMUViconTransform.pos;
end
% 
% % Plot desired and actual position
%     figure;
%     grid on;
%     subplot(3,1,1);plot(states(:,1),states(:,9), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     hold on;
%     subplot(3,1,1);plot(t(1:length(pos_)), pos_(:,1), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     subplot(3,1,1);plot(t(1:length(pos_)), mRes(4,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     legend('Process Model', 'Vicon', 'Measurement residual');
%     
%     grid on;
%     subplot(3,1,2);plot(states(:,1),states(:,10) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     hold on;
%     subplot(3,1,2);plot(t(1:length(pos_)), pos_(:,2), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     subplot(3,1,2);plot(t(1:length(pos_)), mRes(5,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold'); 
%     legend('Process Model', 'Vicon', 'Measurement residual');
%     
%     grid on;
%     subplot(3,1,3);plot(states(:,1),states(:,11) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     hold on;
%     subplot(3,1,3);plot(t(1:length(pos_)), pos_(:,3), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     subplot(3,1,3);plot(t(1:length(pos_)), mRes(6,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     legend('Process Model', 'Vicon', 'Measurement residual');
%     
%     figure;
%     grid on;
%     plot3(states(:,9), states(:,10), states(:,11), 'LineWidth', 1.5);
%     axis([-4 4 -4 4 0 5]);
%     grid on;
%     xlabel('X');ylabel('Y');zlabel('Z');
%     title('Trajectory followed by quadrotor', 'FontSize', 18);
%     grid on; 
%     set(gca,'fontsize',12,'FontWeight','bold');
%     view(3);    
%     
%     % plot angular information
%     figure;
%     grid on;
%     subplot(3,1,1);plot(t(1:length(pos_)),eulerEst(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     hold on;
%     subplot(3,1,1);plot(t(1:length(pos_)), euler_(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     subplot(3,1,1);plot(t(1:length(pos_)), mRes(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     legend('Measurement Model', 'Vicon', 'Measurement residual');
%     
%     grid on;
%     subplot(3,1,2);plot(t(1:length(pos_)),eulerEst(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     hold on;
%     subplot(3,1,2);plot(t(1:length(pos_)),euler_(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     subplot(3,1,2);plot(t(1:length(pos_)), mRes(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     legend('Measurement Model', 'Vicon', 'Measurement residual');
%     
%     grid on;
%     subplot(3,1,3);plot(t(1:length(pos_)),eulerEst(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     hold on;
%     subplot(3,1,3);plot(t(1:length(pos_)),euler_(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     subplot(3,1,3);plot(t(1:length(pos_)), mRes(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
%     set(gca,'fontsize',12,'FontWeight','bold');
%     legend('Measurement Model', 'Vicon', 'Measurement residual');
%     
%     % Check H matrix
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(pos_)), mRes(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(t(1:length(pos_)), mRes_(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('r', 'r from H matrix');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(pos_)),mRes(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(t(1:length(pos_)),mRes_(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('r', 'r from H matrix');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(pos_)),mRes(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(t(1:length(pos_)),mRes_(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('r', 'r from H matrix');
%     
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(pos_)), mRes(4,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(t(1:length(pos_)), mRes_(4,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('r', 'r from H matrix');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(pos_)), mRes(5,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(t(1:length(pos_)), mRes_(5,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold'); 
    legend('r', 'r from H matrix');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(pos_)), mRes(6,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(t(1:length(pos_)), mRes_(6,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('r', 'r from H matrix');
    
    
%     % Computed orientation transform
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(pos_)), IMUVicOrient(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(t(1:length(pos_)), IMUVicOrientDes(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('EKF', 'Actual');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(pos_)),IMUVicOrient(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(t(1:length(pos_)),IMUVicOrientDes(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('EKF', 'Actual');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(pos_)),IMUVicOrient(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(t(1:length(pos_)),IMUVicOrientDes(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('EKF', 'Actual');
    
    % Computed position transform
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(pos_)), IMUVicPos(1,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(t(1:length(pos_)), IMUVicPosDes(1,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('EKF', 'Actual');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(pos_)),IMUVicPos(2,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(t(1:length(pos_)), IMUVicPosDes(2,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('EKF', 'Actual');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(pos_)),IMUVicPos(3,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(t(1:length(pos_)),IMUVicPosDes(3,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('EKF', 'Actual');
end