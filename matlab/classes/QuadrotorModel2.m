classdef QuadrotorModel2
%%
    properties
        % Inertial and motor properties
        mass = 1.251;
        
        I = 0.0001 + [0.0058508, -6.5e-5, -0.00029204;
             -6.5e-5,   0.0081457, 3.528e-5;
             -0.00029204, 3.528e-5, 0.0126617]; % Moment of Inertia
        cT = 1.7081e-5 + 0.01e-5; % Coefficient of Thrust
        cTorque = 0.011651904893177 + 0.001;
        gravity = 9.80665;
        rOffset = [0.01; 0.01; 0.01];

        kMotor = 36.5;
        rpmPerPWM = 20.844;
        rpmVZero = -22688.774; %b
        motorModel = [1.288e-5; -0.02863; 15.93];  
        rpmMin = 410;
        rpmMax = 16081.066;
        armLength = 0.15839;
        motorSpreadAngle = 0.7854;
        dsm = 0.15839*sin(0.7854);
        dcm = 0.15839*cos(0.7854);
        motorMixingModel = [-1, -1, -1, -1;
                            -0.15839*sin(0.7854), 0.15839*sin(0.7854), 0.15839*sin(0.7854), -0.15839*sin(0.7854);
                            0.15839*cos(0.7854), -0.15839*cos(0.7854), 0.15839*cos(0.7854), -0.15839*cos(0.7854);
                            0.011651904893177, 0.011651904893177, -0.011651904893177, -0.011651904893177];
        zOffset = 0.0;

        % State
        IMUViconTransform;
        IMUWorldTransform;

        currentQuaternion;
        currentPosition;
        currentAngVel;
        currentLinAcc;
        currentAngAcc;
    end
%%
    methods
        %%
        function obj = setIMUViconTransform(obj, q, p)
        % Set IMU Vicon transform actual value
            obj.IMUViconTransform.quat = q;
            obj.IMUViconTransform.pos = p;
        end
        %%
        function obj = setIMUWorldTransform(obj, q, p)
        % Set IMU Vicon transform actual value
            obj.IMUWorldTransform.quat = q;
            obj.IMUWorldTransform.pos = p;
        end
        %%
        function obj = setcurrentQuaternion(obj, q)
            obj.currentQuaternion = q;
        end
        %%
        function obj = setcurrentPosition(obj, p)
            obj.currentPosition = p;
        end
        %%
        function obj = setcurrentAngVel(obj, omega)
            obj.currentAngVel = omega;
        end
        %%
        function obj = setcurrentLinAcc(obj, a)
            qx = obj.currentQuaternion.x;
            qy = obj.currentQuaternion.y;
            qz = obj.currentQuaternion.z;
            qw = obj.currentQuaternion.w;
            
            R = zeros(3, 3);
            R(1, 1) = qw*qw + qx*qx - qy*qy - qz*qz;
            R(1, 2) = 2*qx*qy - 2*qw*qz;
            R(1, 3) = 2*qx*qz + 2*qw*qy;
            R(2, 1) = 2*qx*qy + 2*qw*qz;
            R(2, 2) = qw*qw - qx*qx + qy*qy - qz*qz;
            R(2, 3) = 2*qy*qz - 2*qw*qx;
            R(3, 1) = 2*qx*qz - 2*qw*qy;
            R(3, 2) = 2*qy*qz + 2*qw*qx;
            R(3, 3) = qw*qw - qx*qx - qy*qy + qz*qz;
            
            obj.currentLinAcc = R*a;
        end
        %%
        function obj = setcurrentAngAcc(obj, alpha)
            obj.currentAngAcc = alpha;
        end
        %%
        function [a, omega] = genIMUData(obj)
            qx = obj.IMUViconTransform.quat.x;
            qy = obj.IMUViconTransform.quat.y;
            qz = obj.IMUViconTransform.quat.z;
            qw = obj.IMUViconTransform.quat.w;
            
            R = zeros(3, 3);
            R(1, 1) = qw*qw + qx*qx - qy*qy - qz*qz;
            R(1, 2) = 2*qx*qy - 2*qw*qz;
            R(1, 3) = 2*qx*qz + 2*qw*qy;
            R(2, 1) = 2*qx*qy + 2*qw*qz;
            R(2, 2) = qw*qw - qx*qx + qy*qy - qz*qz;
            R(2, 3) = 2*qy*qz - 2*qw*qx;
            R(3, 1) = 2*qx*qz - 2*qw*qy;
            R(3, 2) = 2*qy*qz + 2*qw*qx;
            R(3, 3) = qw*qw - qx*qx - qy*qy + qz*qz;
            
            omega = R*obj.currentAngVel;
            a = R*obj.currentLinAcc + R*(cross(obj.currentAngAcc , obj.IMUViconTransform.pos)) ...
                + R*(cross(obj.currentAngVel, cross(obj.currentAngVel, obj.IMUViconTransform.pos)));
        end
        %%
        function [force] = convert_RPM_toForce(obj, rpm)
        % Assumes:
        % Linear motor: RPM = m*PWM + b;
        % Quadratic thrust model: Thrust = p1*PWM^2 + p2*PWM + p3;
        % Thrust = p1*(RPM-b)^2/m^2 + p2*(RPM-b)/m + p3;
            u = (max(rpm, 0) - obj.rpmVZero)/obj.rpmPerPWM;
            force = obj.motorModel(1)*u*u + obj.motorModel(2)*u + obj.motorModel(3);
        end
        %%
        function [rpm] = convert_force_toRPM(obj, force)
            % Assumes:
            % Linear motor: RPM = m*PWM + b;
            % Quadratic thrust model: Thrust = p1*PWM^2 + p2*PWM + p3;
            f = max(0, force);
            p1 = obj.motorModel(1);
            p2 = obj.motorModel(2);
            p3 = obj.motorModel(3);

            % Solve the quadratic equation below (selecting the appropriate solution)
            % Confirm that it is not imaginary with requirements:
            % If f < p3 (most likely case), imaginary when p1 > p2^2/(4*p3-4*f)
            % If f > p3 (less likely case), imaginary when p1 < p2^2/(4*p3-4*f)
            imaginary = 0;
            if (((f < p3) && (p1 > 0.25*p2*p2/(p3-f))) || ((f > p3) && (p1 < 0.25*p2*p2/(p3-f))))
                 imaginary = 1;
            end
            sq = 0;
            if (imaginary == 0)
               sq = sqrt(p2*p2 - 4*p1*p3 + 4*p1*f);
            end
            pwm = 0.5*(-p2 + sq)/p1;
            rpm = obj.rpmPerPWM*pwm + obj.rpmVZero;
        end       
        %%
        function [rpm] = convert_bodyForces_toRPM(obj, U)
            mixerInv = pinv(obj.motorMixingModel);
            
            % Ensuring the forces are at least at the level of the min command
            fm_base = obj.convert_RPM_toForce(obj.rpmMin);

            % Lower bound thrust (NED frame)
            fbz = min(-4*fm_base, -U(1));

            % Force at each of the motors (NED frame!)
            ub = [fbz; -U(2); -U(3); -U(4)];
            fm = mixerInv*ub;
                
            % Converting force to RPM and ensuring it is in bounds
            rpm = zeros(4,1);
            for i=1:4
                rpm(i) = min(max(obj.convert_force_toRPM(fm(i)),obj.rpmMin), obj.rpmMax);
            end    
       end        
       %%
       function [U] = constrained_control_inputs(obj, omega)
            % Compute individual rotor forces
            force(1,1) = obj.convert_RPM_toForce(omega(1));
            force(2,1) = obj.convert_RPM_toForce(omega(2));
            force(3,1) = obj.convert_RPM_toForce(omega(3));
            force(4,1) = obj.convert_RPM_toForce(omega(4));
    
            % Convert to wrench
            U = (obj.motorMixingModel*force) * (-1);
       end
       %%
       function angAcc = AngularDynamics(obj, tau, angVel, U)
           angAcc = obj.I\(tau - cross(angVel, obj.I*angVel) - cross(obj.rOffset, [0; 0; U(1)]));
       end
       
       %%
       function linAcc = TranslationalDynamics(obj, W, U, F)
           phi = W(10);
           theta = W(11);
           psi = W(12);
           R = zeros(3, 3);

           R(1, 1) = cos(theta)*cos(psi);
           R(1, 2) = cos(psi)*sin(theta)*sin(phi) - cos(phi)*sin(psi);
           R(1, 3) = cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi);
           
           R(2, 1) = cos(theta)*sin(psi);
           R(2, 2) = cos(phi)*cos(psi) + sin(theta)*sin(phi)*sin(psi);
           R(2, 3) = -cos(psi)*sin(phi) + cos(phi)*sin(theta)*sin(psi);
           
           R(3, 1) = -sin(theta);
           R(3, 2) = cos(theta)*sin(phi);
           R(3, 3) = cos(theta)*cos(phi);
           
           linAcc = R*[0; 0; U(1)/obj.mass] - [0; 0; obj.gravity] - cross(W(4:6), cross(W(4:6), obj.rOffset)) - cross(F, obj.rOffset);
       end
    end
end
