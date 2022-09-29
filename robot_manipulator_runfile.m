%%% *Joint Reference tracking for robot manipulator with time delay estimation* 
%%% the idea of this code is the implementation of model predictive
%%% controller with time delaye estimated system *3 joints robot manipulator* in joint space to minimized the joints error. 

%% clear and close all the warning
clc;
clear;
close all;
warning off;

%% Definition of variables

End_time = 40;                  % total time intervel
Ts = 0.01;                      % sampling time  
N1 = 10;                        % prediction horizon

%% initialization of states and input variables
% state variables q = [thita1,thita2,thita2]
q = [0,0.5,0]';
qd = [0,0,0]';
qdd = [0,0,0]';
T = [0,0,0]';

% Define the actual system states and inputs for the plots for the whole simulation range

q_t = zeros(3,End_time/Ts+1);
qd_t = zeros(3,End_time/Ts+1);
qdd_t = zeros(3,End_time/Ts+1);
q_t(:,1) = q;
qd_t(:,1) = qd;
qdd_t(:,1) = qdd;

T_t = zeros(3,End_time/Ts+1);
T_t(:,1) = T;

k = 2;
for t = 0:Ts:End_time-Ts

    
    
%  trajectory

q_ref_1 = 0.5*sin(pi/5*(t));
qd_ref_1 = 0.5*pi/5*cos(pi/5*(t));
qdd_ref_1 = -0.5*pi*pi/25*sin(pi/5*(t));
q_ref_2 = 0.5*cos(pi/5*(t));
qd_ref_2 = -0.5*pi/5*sin(pi/5*(t));
qdd_ref_2 = -0.5*pi*pi/25*cos(pi/5*(t));
q_ref_3 = sin(0.5*t);
qd_ref_3 = 0.5*cos(0.5*t);
qdd_ref_3 = -0.5*0.5*sin(0.5*t);

%% System dynamic
% cos and sine
% Mass-Inertia matrix
c2 = cos(q_t(2,k-1));
c3 = cos(q_t(3,k-1));
c23 = cos(q_t(2,k-1)+q_t(3,k-1));
s2 = sin(q_t(2,k-1));
s3 = sin(q_t(3,k-1));
s23 = sin(q_t(2,k-1)+q_t(3,k-1));
% M-components
m11 = 1.0425+0.08094*c23+0.3484*c2+.0561*c3;
m12 = 0.4398+.04047*c23+.1742*c2+.0561*c3;
m13 = .1788+.04047*c23+.02809*c3;
m22 = .4398+.0519*c3;
m23 = .1788+.02809*c3;
m33 = .1788;
M = [m11 m12 m13;
     m12 m22 m23;
     m13 m23 m33];
% N-components
n11 = -.1742*s2*qd(2)^2-0.04047*s23*qd(2)^2-.02809*s3*qd(3)^2-.04047*s23*qd(3)^2-0.3484*s2*qd(1)*qd(2)-.08094*s23*qd(1)*qd(2);
n12 = -0.05619*s3*qd(1)*qd(3)-.08094*s23*qd(1)*qd(3)-.05619*s3*qd(2)*qd(3)-.08094*s23*qd(2)*qd(3);
n2 = .1742*s2*qd(1)^2+.04047*s23*qd(1)^2-.02809*s3*qd(3)^2-0.0561*s3*qd(1)*qd(3)-0.05619*s3*qd(2)*qd(3);
n3 = 0.02809*s3*qd(1)^2+.02809*s3*qd(2)^2+.04047*s23*qd(1)^2+.05619*s3*qd(1)*qd(2);
% N matrix
N = [n11+n12;
     n2;
     n3];
% Friction coefficients
F = [2.6e-4 0 0;
     0 2.6e-4 0; 
     0 0 2.6e-4];

%% Parameters for MPC
g_bar = [20,20,20];
N1 = 10;

%% Inputs and  References for IMPC control for 3 joints robotic arm

u_1 = zeros(1,N1)';
u_2 = zeros(1,N1)';
u_3 = zeros(1,N1)';

X_ref_1 = zeros(40,1);
X_ref_2 = zeros(40,1);
X_ref_3 = zeros(40,1);

    % every simulation, update only 10 values to perform IMPC for length 10
    
    for i = 1:N1
        
        q_ref_1_1 = 0.5*sin(pi/5*((k+i)*Ts));
        qd_ref_1_1 = 0.5*pi/5*cos(pi/5*((k+i)*Ts));
        q_ref_1_2 = 0.5*sin(pi/5*((k+i-1)*Ts));
        qd_ref_1_2 = 0.5*pi/5*cos(pi/5*((k+i-1)*Ts));
        
        q_ref_2_1 = 0.5*cos(pi/5*((k+i)*Ts));
        qd_ref_2_1 = -0.5*pi/5*sin(pi/5*((k+i)*Ts));
        q_ref_2_2 = 0.5*cos(pi/5*((k+i-1)*Ts));
        qd_ref_2_2 = -0.5*pi/5*sin(pi/5*((k+i-1)*Ts));
        
        q_ref_3_1 = sin(0.5*((k+i)*Ts));
        qd_ref_3_1 = 0.5*cos(0.5*((k+i)*Ts));
        q_ref_3_2 = sin(0.5*((k+i-1)*Ts));
        qd_ref_3_2 = 0.5*cos(0.5*((k+i-1)*Ts));
        
        X_ref_1((1+((i-1)*4)):(4+((i-1)*4)),1) = [q_ref_1_1, qd_ref_1_1, q_ref_1_2, qd_ref_1_2]';
        X_ref_2((1+((i-1)*4)):(4+((i-1)*4)),1) = [q_ref_2_1, qd_ref_2_1, q_ref_2_2, qd_ref_2_2]';
        X_ref_3((1+((i-1)*4)):(4+((i-1)*4)),1) = [q_ref_3_1, qd_ref_3_1, q_ref_3_2, qd_ref_3_2]';

        i = i + 1;
    end
    
        %updated references joint angles
     
        X_ref_1;
        X_ref_2;
        X_ref_3;
    
        %current states, joint angles, x(k) for mpc controller
        
        x_k_1 =[q_t(1,k); qd_t(1,k); q_t(1,k-1); qd_t(1,k-1)];
        x_k_2 =[q_t(2,k); qd_t(2,k); q_t(2,k-1); qd_t(2,k-1)];
        x_k_3 =[q_t(3,k); qd_t(3,k); q_t(3,k-1); qd_t(3,k-1)];
    
                %% optimization:- the optimization is perform based on optimization tool fmincon 
 
                deltau_1 = fmincon(@(u1)costfunction1(X_ref_1,x_k_1,u1),u_1,[],[]);
                deltau_2 = fmincon(@(u2)costfunction2(X_ref_2,x_k_2,u2),u_2,[],[]);
                deltau_3 = fmincon(@(u3)costfunction3(X_ref_3,x_k_3,u3),u_3,[],[]);
     
        % update inputs for the next optimization 
     
        u_1 = [deltau_1(2:end);0];
        u_2 = [deltau_2(2:end);0];
        u_3 = [deltau_3(2:end);0];

        % optimized input for robotic arm 
         
        T_t(1,k) = T_t(1,k-1) + deltau_1(1,1);
        T_t(2,k) = T_t(2,k-1) + deltau_2(1,1);
        T_t(3,k) = T_t(3,k-1) + deltau_3(1,1);

%% updated dynamics for robotic arm for next iterations
 
    qdd=M\(T_t(:,k)-N-F*qd);
    qd = qd + qdd*Ts;
    q = q + qd*Ts;

    q_t(:,k+1) = q;
    qd_t(:,k+1) = qd;
    qdd_t(:,k+1) = qdd;
 
    k = k + 1
    
end

figure
%plot(thita1, thita2, thita3)
plot3(q_t(1,:),q_t(2,:),q_t(3,:),'--');
time_serial = 0:Ts:End_time;
hold on
%plot(thita1ref,thita2ref,thita3ref)
plot3(0.5*sin(pi/5*(time_serial)),0.5*cos(pi/5*(time_serial)),sin(0.5*(time_serial)));
legend('Actual trajectory','Reference trajectory');
xlabel('x');
ylabel('y');
zlabel('z');
title('IMPC Tracking Performance of Robot manipulator')
set(gcf,'color','w');


%%
figure
subplot(3,1,1)
%plot thita1 error over time horizon
time_serial = 0:Ts:End_time+0.01;
plot( 0:Ts:End_time+0.01 , q_t(1,:) - 0.5*sin(pi/5*(time_serial)) );
title('Joint1')
xlabel('time')
ylabel('amplitude(Grad)')
legend('error e1');

subplot(3,1,2)
%plot thita2 error over time horizon
time_serial = 0:Ts:End_time+0.01;
plot( 0:Ts:End_time+0.01 , q_t(2,:) - 0.5*cos(pi/5*(time_serial)) );
title('Joint2')
xlabel('time')
ylabel('amplitude(Grad)')
legend('error e2');

subplot(3,1,3)
%plot thita3 error over time horizon
time_serial = 0:Ts:End_time+0.01;
plot( 0:Ts:End_time+0.01 , q_t(3,:) - sin(0.5*(time_serial)));
title('Joint3')
xlabel('time')
ylabel('amplitude(Grad)')
legend('error e3');

set(gcf,'color','w');