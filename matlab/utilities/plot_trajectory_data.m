function plot_trajectory_data(des, states)
    figure;
    grid on;
    plot3(states(:,2), states(:,3), states(:,4), 'LineWidth', 1.5);
    hold on;
    plot3(des(:,8), des(:,9), des(:,10), 'LineWidth', 1.5);
    axis([-4 4 -4 4 0 10]);
    grid on;
    xlabel('X');ylabel('Y');zlabel('Z');
    title('Trajectory followed by quadrotor', 'FontSize', 18);
    legend('actual', 'desired');
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    view(3);
end