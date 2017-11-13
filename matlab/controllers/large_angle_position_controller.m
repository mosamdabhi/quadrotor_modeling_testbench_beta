function [angCommands, u1Des] = large_angle_position_controller(error, currentQuaternion, controller, QuadModel)
 
 kp = controller.kp;
 kv = controller.kv;
 m = QuadModel.mass;
 g = QuadModel.gravity;
 
 errorPosition = error.p;
 errorVelocity = error.v;
 commandAcc = error.acc;
 commandHeading = error.heading;
 
 % Desired force in world frame.
 FDes = m*(kp.*errorPosition + kv.*errorVelocity + [0; 0; g] + commandAcc);
 
 R = QuatToR(currentQuaternion); % Compute rotation matrix body to world
 z_b = R(:,3); % Body Z
 u1Des = dot(FDes, z_b);
 
 zbDes = FDes/norm(FDes); % Direction of desired body Z axis w.r.t world frame
 
 % Desired body orientation when desired heading is kept zero.
 xcDes = [cos(commandHeading(1)), sin(commandHeading(1)), 0]';
 ybDes = (cross(zbDes, xcDes))/(norm(cross(zbDes, xcDes)));
 xbDes = cross(ybDes, zbDes);
 
 % Define desired rotation matrix and convert to quaternion for inner loop
 % input
 RDes = horzcat(xbDes, ybDes, zbDes);
 commandQuat = RToQuat(RDes);
 
 % Compute desired body frame angular velocity
 errorAcc = [0; 0; 0];
 jerk = (-kp.*errorVelocity - kv.*errorAcc)*m; 
 b3 = zbDes/norm(zbDes);
 desThrust = abs(u1Des);
 hw = (m/desThrust)*(-(dot(b3, jerk))*b3);
 commandAngVel(1) = dot(hw,xbDes);
 commandAngVel(2) = dot(hw, -ybDes);

 tempmat = zeros(3,3);
 tempmat(:,1) = xcDes;
 tempmat(:,2) = ybDes;
 tempmat(3,3) = 1.0;
 
 midmat = R'*tempmat;

 rhs(1) = commandAngVel(1) - commandHeading(2)*midmat(1,3);
 rhs(2) = commandAngVel(2) - commandHeading(2)*midmat(2,3);
 
 rollpitchdot = midmat(1:2, 1:2)\rhs';
 commandAngVel(3) = midmat(3,1)*rollpitchdot(1) + midmat(3,2)*rollpitchdot(2) + midmat(3,3)*commandHeading(2);
 
 %% Return structure
 angCommands.commandQuat = commandQuat;
 angCommands.commandAngVel = commandAngVel;
end

