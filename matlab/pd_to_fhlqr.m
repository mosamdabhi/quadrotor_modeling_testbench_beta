function pd_to_fhlqr()

% subsys = 'att'; 
subsys = 'pos';

%veh = 'crazyflie40';
veh = 'danaus05';
% veh = 'CMUQuad10';

if strcmp(subsys,'pos')
    
    if strcmp(veh,'danaus05')
        mass = 1.251;
        N = 1;
%         N = 10;
        rate = 1000;
        
        K = [4 0 0 5.5 0 0
            0 10 0 0 6.5 0
            0 0 16 0 0 10]*mass;
        
    elseif strcmp(veh,'CMUQuad10')
        mass = 1.855;
        N = 15;
        rate = 100;
        
        K = [6.8 0 0 3 0 0
            0 6.8 0 0 3 0
            0 0 7.099 0 0 3.9572]*mass;

    elseif strcmp(veh,'crazyflie40')
        mass = 0.032;
        N = 10;
        rate = 40;
        
        % Crazyflie PD gains appear to include mass
        K = [0.5 0 0 0.4 0 0
            0 0.5 0 0 0.4 0
            0 0 0.8 0 0 0.5];
    end
    
    Ac = zeros(6,6);
    Ac(1,4) = 1;
    Ac(2,5) = 1;
    Ac(3,6) = 1;
    
    Bc = zeros(6,3);
    Bc(4,1) = 1.0/mass;
    Bc(5,2) = 1.0/mass;
    Bc(6,3) = 1.0/mass;
    
    
    sysd = c2d(ss(Ac,Bc,eye(6),0),1/rate);
    A = sysd.A;
    B = sysd.B;
        
    Aeq = [1 -1 zeros(1,7)
        zeros(1,3) 1 -1 zeros(1,4)
        zeros(1,6) 1 -1 0];
    
    beq = zeros(3,1);
    
    outstr = 'Q:\n  x:     %f\n  y:     %f\n  z:     %f\n  xdot:  %f\n  ydot:  %f\n  zdot:  %f\nR:\n  fx:    %f\n  fy:    %f\n  fz:    %f\n';
    
elseif strcmp(subsys,'att')
    
    if strcmp(veh,'danaus05')
        inertia = diag([0.003 0.003 0.005]);
%         N = 25;
        N = 10;
        rate = 200;
        
        K = inertia*...
            [190 0 0 30 0 0
            0 198 0 0 30 0
            0 0 80 0 0 17.88];
%         K = inertia*...
%             [160 0 0 38 0 0
%             0 160 0 0 38 0
%             0 0 64 0 0 16];
        
    elseif strcmp(veh,'CMUQuad10')
        inertia = [0.0210468 -0.000106389 4.85029e-6
                   -0.000106389 0.021109 0.0000195723
                   4.85029e-6 0.0000195723 0.0240574];
        N = 10;
        rate = 200;
        
        K = inertia*...
            [169 0 0 15.6 0 0
            0 169 0 0 15.6 0
            0 0 56.2 0 0 4];
    end
    
    Ac = zeros(6,6);
    Ac(1,4) = 1;
    Ac(2,5) = 1;
    Ac(3,6) = 1;
    
    Bc = zeros(6,3);
    Bc(4:6,:) = inv(inertia);

    
    sysd = c2d(ss(Ac,Bc,eye(6),0),1/rate);
    A = sysd.A;
    B = sysd.B;
    
    Aeq = [1 -1 zeros(1,7)
        zeros(1,3) 1 -1 zeros(1,4)];
    beq = [0;0];

    outstr = 'Q:\n  phi:     %f\n  theta:   %f\n  psi:     %f\n  p:       %f\n  q:       %f\n  r:       %f\nR:\n  Mphi:    %f\n  Mtheta:  %f\n  Mpsi:    %f\n';
    
end

    function K = fhlqr(Q,R)
        P=Q;
        for ii=1:N,
            F = (R + B'*P*B)\(B'*P*A);
            Pnew = Q + F'*R*F + (A-B*F)'*P*(A-B*F);
            P = Pnew;
        end
        K = F;
    end

% costfn = @(w) sum(sum(abs(fhlqr(diag(w(1:6)),diag(w(7:end)))-K))) + 2e-2*sum(abs(log(w))); %0.0001*sum((w-1).^2);
costfn = @(w) 100*sum(sum(abs(fhlqr(diag(w(1:6)),diag(w(7:end)))-K))) + 1e-9*sum(w.^2);

w = fmincon(costfn,[10 10 10 1 1 1 .1 .1 .1],[],[],Aeq,beq,zeros(9,1)+1e-4,1000*ones(9,1));
disp('Original weights')
disp(w)

scalefactor = exp(-mean(log(w)));
w = w*scalefactor;
fprintf('Scale Factor: %f\n',scalefactor)

fprintf(['-------\nSubsystem: %s\nVehicle: %s\n\n',outstr,'-------\n'],...
    subsys,veh,w)

disp('Reference K:')
disp(K)
disp('Finite Horizon LQR K:')
disp(fhlqr(diag(w(1:6)),diag(w(7:end))))

dtpoles = eig(A-B*K);
disp('DT Poles:');
for ii=1:numel(dtpoles)
    if isreal(dtpoles(ii))
        fprintf('   %4.4f\n',dtpoles(ii));    
    else
        impart = imag(dtpoles(ii));
        if impart < 0
            sgn = '-';
        else
            sgn = '+';
        end
        fprintf('   %4.4f %c %4.4fj\t(Mag: %4.4f)\n', real(dtpoles(ii)), sgn, abs(impart), abs(dtpoles(ii)));
    end
end

end
