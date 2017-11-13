function IMU_test(states, quat_, pos_, t, hulk1c)

q_ = zeros(4, length(quat_.x));
q = zeros(4, length(states(:,1)));
euler_ = zeros(3, length(quat_.x));
euler = zeros(3, length(states(:,1)));

for i = 1:1:length(quat_.x)
    q_(:, i) = [quat_.x(i); quat_.y(i); quat_.z(i); quat_.w(i)];
    
    R = QuatToR(quat_struct(q_(:, i)));
    
    qIV(1,1) = hulk1c.IMUViconTransform.quat.x;
    qIV(2,1) = hulk1c.IMUViconTransform.quat.y;
    qIV(3,1) = hulk1c.IMUViconTransform.quat.z;
    qIV(4,1) = hulk1c.IMUViconTransform.quat.w;
 
    q_(:,i) = MultiplyQuat(qIV, q_(:,i));
    
    PosIV = hulk1c.IMUViconTransform.pos;
    pos_(:,i) = pos_(:,i) + R'*PosIV;
    euler_(:, i) = QuatToZYX(q_(:, i));
end

for l = 1:1:length(states(:,1))
    q(:,l) = states(l, 2:5)';
    euler(:, l) = QuatToZYX(q(:, l));
end
    pos_ = pos_';
    figure;
    grid on;
    subplot(3,1,1);plot(states(:,1),states(:,9), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    grid on;
    subplot(3,1,1);plot(t(1:length(pos_)), pos_(:,1), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Process Model', 'Vicon');
    
    grid on;
    subplot(3,1,2);plot(states(:,1),states(:,10) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    grid on;
    subplot(3,1,2);plot(t(1:length(pos_)), pos_(:,2), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold'); 
    legend('Process Model', 'Vicon');
    
    grid on;
    subplot(3,1,3);plot(states(:,1),states(:,11) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    grid on;
    subplot(3,1,3);plot(t(1:length(pos_)), pos_(:,3), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Process Model', 'Vicon');
    
    figure;
    grid on;
    plot3(states(:,9), states(:,10), states(:,11), 'LineWidth', 1.5);
    hold on;
    grid on;
    plot3(pos_(:,1), pos_(:,2), pos_(:,3), 'LineWidth', 1.5);
    axis([-4 4 -4 4 0 5]);
    grid on;
    xlabel('X');ylabel('Y');zlabel('Z');
    title('Trajectory followed by quadrotor', 'FontSize', 18);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    view(3);    

    % plot angular information
    figure;
    grid on;
    subplot(3,1,1);plot(states(:,1),euler(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    grid on;
    subplot(3,1,1);plot(t(1:length(pos_)),euler_(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Process Model', 'Vicon');
    
    grid on;
    subplot(3,1,2);plot(states(:,1),euler(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    grid on;
    subplot(3,1,2);plot(t(1:length(pos_)),euler_(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Process Model', 'Vicon');
    
    grid on;
    subplot(3,1,3);plot(states(:,1) ,euler(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    grid on;
    subplot(3,1,3);plot(t(1:length(pos_)),euler_(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('Process Model', 'Vicon');
    
end