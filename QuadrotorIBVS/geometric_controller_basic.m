% state x = [x,y,z,R11,R12,R13,R21,R22,R23,R31,R32,R33,x_dot,y_dot,z_dot,w_x,w_y,w_z]';
%           18x1
%           [Position = (x,y,z); Rotation Matrix of Body Frame represented in World Frame => Their derivatives; Omega => angular velocity in body frame]
% input u = [F_total, phi_dot_dot, theta_dot_dot, psi_dot_dot]'
% y_d = [x_d y_d z_d psi_d]'
% y_dot_d = [x_dot_d y_dot_d z_dot_d psi_dot_d]'
% y_dot_dot_d = [x_dot_dot_d y_dot_dot_d z_dot_dot_d psi_dot_dot_d]'
%       4x1
% m = mass (kg)
% J = Intertia Matrix (3x3) kg-m^2
% g = graviational acc m/s^2
function [u] = geometric_controller_basic(y_d, y_dot_d, y_dot_dot_d, x, m, J, g)
k_x = 15;
k_v = 15.6;
k_R = 8.81;
k_omega = 2.54;
% force in world frame
f_w = (k_x*(x(1:3)-y_d(1:3)) + k_v*(x(13:15)-y_dot_d(1:3)) + (m*g*[0 0 1]') - m*y_dot_dot_d(1:3)); % 
R = [x(4:6)';
     x(7:9)';
     x(10:12)'];
f = dot(f_w,R*[0 0 1]');
R_c = computeRc(f_w,R);
R_dot_c = zeros(3,3);   % for fixed reference position R_dot_c = 0
omega_c_hat = R_c'*R_dot_c;
omega_c = [omega_c_hat(3,2), omega_c_hat(1,3), omega_c_hat(2,1)]';

e_R_hat = 0.5*(R_c'*R - R'*R_c);
e_R = [e_R_hat(3,2), e_R_hat(1,3), e_R_hat(2,1)]';
omega = x(16:18);
e_omega = omega - R'*R_c*omega_c;
%M_t1 = -k_R*e_R - k_omega*e_omega + cross(omega_t1,J*omega_t1) - J*(cross(omega_t1,R_t1')*R_c(:,:,2)*omega_c_t1 - R_t1'*R_c(:,:,2)*Omega_c_dot_t1);
M = -k_R*e_R - k_omega*e_omega + cross(omega,J*omega) - 0;
u = [f; M];

% f_w =  Thrust vector of quadrotor in world frame
% R = Rotation Matrix of Body Frame represented in World Frame
% R_c = Computed rotation matrix corresponding to f_w (used to find sign of b1_c if b3_c is pointing upwards)
    function [R_c] = computeRc(f_w, R)
        b3_c = -f_w/norm(f_w);
        if(sum(abs(b3_c-[0 0 1]'))<0.01)        % if b3_c is pointing upwards then directly define b1_c
            b1_c = [1*sign(R(1,1)) 0 0]';
        else
            b1_c = -cross(b3_c,cross(b3_c,[1 0 0]'));      % we are interested in controlling position so choose random heading for quadrotor (like yaw)
            b1_c = b1_c/norm(b1_c);
        end
        R_c = [b1_c cross(b3_c, b1_c) b3_c];
    end
end