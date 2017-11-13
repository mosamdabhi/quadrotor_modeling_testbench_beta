function plot_kalman(t, X, XEst, sigma, z)

figure;
plot(t, X(1,:), '-b');
hold on;
plot(t, z(1,:), '-g');
plot(t, XEst(1,:), '-m');
plot(t, XEst(1,:) + 1*sigma(1,:),'-r');
plot(t, XEst(1,:) - 1*sigma(1,:),'-r');
set(gca,'fontsize',12,'FontWeight','bold');
title('Position estimation results');
xlabel('time (s)');
ylabel('X coordinate (in m)');
legend('Truth', 'Measurement', 'Kalman estimates', '+ \sigma', '- \sigma');
end