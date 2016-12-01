function QuadrotorIBVS
sim_time = 50;
addpath('..');
iniCon = zeros(18,1);
iniCon(1)=-0.5;
iniCon(2)=-0.5;
iniCon(3)=4;
iniCon(4)= 1;
iniCon(8)= -1;
iniCon(12)= -1;

% Setup Pointmass CLF
global env;
env = PointmassEnvironment;
env = env.SetCirObs([0.1,-0.05,2,0.1]');   
controller = CLFController(env);
env = env.SetController(controller);


% iniYaw = 0;
% iniPitch = -pi/2;
% iniRoll = pi;
% dcm = angle2dcm( iniYaw, iniPitch, iniRoll );
% iniCon(4:6) = dcm(1,1:3);
% iniCon(7:9)= dcm(2,1:3);
% iniCon(10:12)= dcm(3,1:3);

% HB Pysical properties source: http://wiki.asctec.de/display/AR/CAD+Models
g = 9.81;
% m = 0.53057; % HB
m = 1;

% ( kilograms * square meters )
%J = [0.00367556974  0.00000426367  -0.00005668247; % HB
%     0.00000426367  0.00703095144  -0.00001409010
%    -0.00005668247 -0.00001409010   0.00365031574];
J = diag([0.0820,0.0820,0.1377]);   
J_inv = inv(J);
dt = 0.001;
tspan = 0:dt:sim_time;

[t, y] = ode45(@Nonlinear_Dynamics, tspan, iniCon);
plot_graphs(t,y);

    % state x = [x,y,z,R11,R12,R13,R21,R22,R23,R31,R32,R33,x_dot,y_dot,z_dot,w_x,w_y,w_z]'
    %           18x1
    %           [Position = (x,y,z); Rotation Matrix of Body Frame represented in World Frame => Their derivatives; Omega => angular velocity in body frame]
    % input u = [F_total, phi_dot_dot, theta_dot_dot, psi_dot_dot]
    % Units: S.I.
    function dx = Nonlinear_Dynamics(t,x)
    dx = zeros(18,1);

    % velocity
    dx(1) = x(13); dx(2) = x(14); dx(3) = x(15);

    % R_dot
    omega = [x(16) x(17) x(18)]';
    omega_hat = [0        -omega(3)  omega(2);
                 omega(3)     0     -omega(1);
                -omega(2)  omega(1)    0];
    R = [x(4:6)'; x(7:9)'; x(10:12)'];      
    R_dot = R*omega_hat;
    dx(4:6) = R_dot(1,1:3); dx(7:9) = R_dot(2,1:3); dx(10:12) = R_dot(3,1:3);

    % compute u
    [y_d, y_dot_d, y_dot_dot_d] =  trajGenerator_fixedPoint();
%     [y_d, y_dot_d, y_dot_dot_d] =  trajGenerator_sinePath(t);
    [u] = geometric_controller_CLF(y_d, y_dot_d, y_dot_dot_d, x, m, J, g);
    %u=zeros(4,1);

    % Linear Acceleration
    a = g*[0 0 1]' - (1/m)*u(1)*R*[0 0 1]';
    dx(13:15) = a(1:3);

    % Angular Accerleration
    alpha = J_inv*(-omega_hat*J*omega + u(2:4));
    dx(16:18) = alpha(1:3);
    end

    function[y_d, y_dot_d, y_dot_dot_d] = trajGenerator_fixedPoint()
%     x_d = 1;  % position in m
%     y_d = 1;
%     z_d = 0.5;
    x_d = 0.5; y_d = 0.5; z_d = 5;
    yaw_d = 0;  
    y_d = [x_d; y_d; z_d; yaw_d];
    y_dot_d = zeros(size(y_d));
    y_dot_dot_d = zeros(size(y_d));
    end

    function[y_d, y_dot_d, y_dot_dot_d] = trajGenerator_sinePath(t)
        vel = 1.5; % velocity in m/s
        r = 1; % radius of circle in m/s

        Ax = r;   % amplitude in m
        Ay = r;   
        w = vel/r;      % freq in rad/s
        x_d = Ax*cos(w*t);
        y_d = Ay*sin(w*t);
        z_d = 0;
        yaw_d = 0;
        y_d = [x_d; y_d; z_d; yaw_d];
        y_dot_d = [-Ax*w*sin(w*t); Ay*w*cos(w*t); 0; 0];
        y_dot_dot_d = [-Ax*w*w*cos(w*t); -Ay*w*w*sin(w*t); 0; 0];
    end

    function plot_graphs(t,y)
        [y_d,~,~] = arrayfun(@trajGenerator_sinePath, t, 'UniformOutput', false);
        y_d = cell2mat(y_d);
        y_d = reshape(y_d,4,size(y_d,1)/4)';
        plot(t,y_d(:,1:3)-y(:,1:3));
        figure;
        plot(t,y_d(:,1:3),'--');
%         hold on;
%         plot(t,y(:,1:3));
%         hold off;
    end
end