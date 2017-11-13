function FILTER_OUTPUT = estimate_transforms(endtime, plot_flag)
close all;
load('time.mat');
load('lin_vel.mat');

%% Initialize Quad
hulk1c = QuadrotorModel;

%% Simulate IMU
[a, av, omega, hulk1c] = simulate_IMU(hulk1c);

%% Simulate Vicon
[quatViconWorld, posViconWorld] = simulate_Vicon();

%% Initialization for transform estimation
k = find(t(:,1)==endtime);
covEst = zeros(15, 15, k);
mRes = zeros(6, k);
zEst = zeros(7, k);
dt = t(2) - t(1);
WActual = zeros(17,k);
WNominal = zeros(17, k);
WEst = zeros(17, k);
% ErrW = zeros(15, k);
TransformError = zeros(15, k);
%cost = zeros(1, k);

%% Actual W initialization
    qVG_Vicon(1,1) = quatViconWorld.x(1);
    qVG_Vicon(2,1) = quatViconWorld.y(1);
    qVG_Vicon(3,1) = quatViconWorld.z(1);
    qVG_Vicon(4,1) = quatViconWorld.w(1);
    
    R = QuatToR(quat_struct(qVG_Vicon));
    
    qIV(1,1) = hulk1c.IMUViconTransform.quat.x;
    qIV(2,1) = hulk1c.IMUViconTransform.quat.y;
    qIV(3,1) = hulk1c.IMUViconTransform.quat.z;
    qIV(4,1) = hulk1c.IMUViconTransform.quat.w;
    
    qIG_Vicon = MultiplyQuat(qIV, qVG_Vicon);
    
    VelIG = lin_vel_store(:, 1);
    PosIV = hulk1c.IMUViconTransform.pos;
    PosIG = posViconWorld(:,1) + R'*PosIV;
    
    WActual(:,1) = [qIG_Vicon; VelIG; PosIG; qIV; PosIV];
    
for i = 1:1:(k-1)
   % Populate actual vector
    qVG_Vicon(1,1) = quatViconWorld.x(i+1);
    qVG_Vicon(2,1) = quatViconWorld.y(i+1);
    qVG_Vicon(3,1) = quatViconWorld.z(i+1);
    qVG_Vicon(4,1) = quatViconWorld.w(i+1);
    
    R = QuatToR(quat_struct(qVG_Vicon));
    
    qIV(1,1) = hulk1c.IMUViconTransform.quat.x;
    qIV(2,1) = hulk1c.IMUViconTransform.quat.y;
    qIV(3,1) = hulk1c.IMUViconTransform.quat.z;
    qIV(4,1) = hulk1c.IMUViconTransform.quat.w;
    
    qIG_Vicon = MultiplyQuat(qIV, qVG_Vicon);
    
    VelIG = lin_vel_store(:, i+1);
    PosIV = hulk1c.IMUViconTransform.pos;
    PosIG = posViconWorld(:,i+1) + R'*PosIV;
    
    WActual(:,i+1) = [qIG_Vicon; VelIG; PosIG; qIV; PosIV];
        
   if i == 1
        covEst(:,:,i) = 0.1.*eye(15, 15);
        %W(1:4,1) = qVG_Vicon;
        WNominal(1,i) = quatViconWorld.x(1);
        WNominal(2,i) = quatViconWorld.y(1);
        WNominal(3,i) = quatViconWorld.z(1);
        WNominal(4,i) = quatViconWorld.w(1);
        WNominal(8:10,i) = posViconWorld(:,1);
        WNominal(14,i) = 1;
        WNominal(15:17,i) = [0.0;0;0];
        %ErrW(:,i) = zeros(15,1);
        WEst(:,i) = WNominal(:,i);
        TransformError(:,i) = errstate_compute(WActual(:,i), WEst(:,i));
        costPrime(:,i) = TransformError(:,i)'*TransformError(:,i);
   end

    A0 = eye(3,3);
    A0 = reshape(A0, [9,1]);
    
    augW = vertcat(WEst(:,i), A0);
    
    tspan = [t(i) t(i+1)];
    [t_soln, W_soln] = ode45(@(t, augW) ProcessModel(t, augW, a(i,:)', omega(i,:)', hulk1c), tspan, augW);
    WNominal(:,i+1) =  W_soln(end,1:17)';
    F21 = W_soln(end, 18:end)';

    F21 = reshape(F21, [3,3]);

    sys.F = [-skewmat(omega(i,:)'), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);
        zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);
        zeros(3,3), eye([3,3]), zeros(3,3), zeros(3,3), zeros(3,3);
        zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);
        zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3)];
     
     dis(:,:,i) = distrans(sys.F, F21, dt); 
    
     meanTemp = zeros(15,1);%dis(:,:,i)*ErrW(:,i); % Prediction update of err state
     covTemp = dis(:,:,i)*covEst(:,:,i)*dis(:,:,i)'; % Prediction update of covariance 
   
     %sys.H = [QuatToR(quat_struct(WEst(11:14, i)))', zeros(3,3), zeros(3,3), -QuatToR(quat_struct(WEst(1:4, i)))', zeros(3,3);
     %         zeros(3,3), zeros(3,3), eye(3,3), zeros(3,3), -QuatToR(quat_struct(WEst(1:4, i)))*QuatToR(quat_struct(WEst(11:14, i)))'];
     for ii = 1:1:10
     if ii == 1    
     sys.H = [eye(3,3), zeros(3,3), zeros(3,3), -eye(3,3), zeros(3,3);
             zeros(3,3), zeros(3,3), eye(3,3), zeros(3,3), -QuatToR(quat_struct(WNominal(1:4, i+1)))*QuatToR(quat_struct(WNominal(11:14, i+1)))'];
     else
     sys.H = [eye(3,3), zeros(3,3), zeros(3,3), -eye(3,3), zeros(3,3);
             zeros(3,3), zeros(3,3), eye(3,3), zeros(3,3), -QuatToR(quat_struct(WEst(1:4, i+1)))*QuatToR(quat_struct(WEst(11:14, i+1)))'];    
     end
     sys.R = 0.05*eye(6,6);
     
     zEst(:,i) = NonlinearMeasurementModel(WNominal(:, i+1));
     mRestemp(1:4, 1) = MultiplyQuat(quatViconWorld, InverseQuat(quat_struct(zEst(1:4, i))), i+1);
     mRestemp(5:7,1) = posViconWorld(:, i+1) - zEst(5:7, i); 
    
     mRes(1:3, i) = 2.*mRestemp(1:3 ,1);
     mRes(4:6, i) = mRestemp(5:7 ,1);
    
     K = covTemp*sys.H'*inv(sys.H*covTemp*sys.H' + sys.R);
     
     meanTemp = K*(mRes(:,i) + sys.H*meanTemp);
     
     covEst(:,:,i+1) = covTemp - K*sys.H*covTemp;
     
     WEst(:, i+1) = propagate_state(WNominal(:,i+1), meanTemp);
     
     TransformError(:,i+1) = errstate_compute(WActual(:,i+1), WEst(:,i+1));
     
     xTilde(:,i) = errstate_compute(WNominal(:,i+1), WEst(:,i+1));
     
     %cost(:,i+1) = TransformError(:,i+1)'*TransformError(:,i+1);
     
     cost(:,ii) = xTilde(:,i)'*inv(covEst(:,:,i+1))*xTilde(:,i) + mRes(:,i)'*inv(sys.R)*mRes(:,i);
     
     if cost(:,ii) < cost(:,1)/10  && cost(:,i+1) > cost(:,i)
         break;
     end
     
     end 
     costPrime(:,i+1) = TransformError(:,i+1)'*TransformError(:,i+1);
     disp(t(i));
end 

if plot_flag
states = vertcat(t(1:length(WNominal(1,:)))', WNominal);

Error_test(TransformError, covEst, t);
IMU_test(states', quatViconWorld, posViconWorld, t, hulk1c);

figure;
plot(t(1:length(costPrime)), costPrime, 'LineWidth', 1.5); xlabel('time'); ylabel('Cost');
grid on
set(gca,'fontsize',12,'FontWeight','bold');
end

FILTER_OUTPUT.estimate = WEst;
FILTER_OUTPUT.actual = WActual;
FILTER_OUTPUT.error = TransformError;
FILTER_OUTPUT.cost = cost;
end
