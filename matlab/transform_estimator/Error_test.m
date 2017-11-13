function Error_test(err_, cov_, t)
    sigma_ = zeros(15, length(err_(1,:)));
    for i = 1:1:length(err_(1,:))
        sigma_(:,i) = sqrt(diag(cov_(:,:,i)));
    end
    
    % Computed orientation errors in IG
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(1,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(1,:)*180/pi + sigma_(1,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(1,:)*180/pi - sigma_(1,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '- \sigma');
    title('Computed orientation errors in IG');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(2,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(2,:)*180/pi + sigma_(2,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(2,:)*180/pi - sigma_(2,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '- \sigma');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(3,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(3,:)*180/pi + sigma_(3,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(3,:)*180/pi - sigma_(3,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    
    
    % Computed position errors in IG
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(7,:) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(7,:) + sigma_(7,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(7,:) - sigma_(7,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    title('Computed position errors in IG');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(8,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(8,:) + sigma_(8,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(8,:) - sigma_(8,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(9,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(9,:) + sigma_(9,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(9,:) - sigma_(9,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    
    
    % Computed orientation errors in IV
    figure;
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(10,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(10,:)*180/pi + sigma_(10,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(10,:)*180/pi - sigma_(10,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '- \sigma');
    title('Computed orientation errors in IV');
    
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(11,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(11,:)*180/pi + sigma_(11,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(11,:)*180/pi - sigma_(11,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(12,:)*180/pi, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(12,:)*180/pi + sigma_(12,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(12,:)*180/pi - sigma_(12,:)*180/pi, 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    
    
    % Computed position errors in IV
    figure;
    
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(13,:) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(13,:) + sigma_(13,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    subplot(3,1,1);plot(t(1:length(err_(1,:))), err_(13,:) - sigma_(13,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    title('Computed position errors in IV');
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(14,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(14,:) + sigma_(14,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    subplot(3,1,2);plot(t(1:length(err_(1,:))), err_(14,:) - sigma_(14,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(15,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    hold on;
    grid on;
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(15,:) + sigma_(15,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    subplot(3,1,3);plot(t(1:length(err_(1,:))), err_(15,:) - sigma_(15,:), 'r--', 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('mean', '\sigma', '-\sigma');
    
end