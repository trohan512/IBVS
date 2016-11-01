classdef PointmassEnvironment < Environment
    properties
        fx=350; fy=350; cx=0; cy=0; xf1=0.1; yf1=0; xf2=-0.1; yf2=-0.1; m=1;
    end
    methods
        function obj = PointmassEnvironment()
            obj.EnvironmentName = 'Pointmass';
        end

        function [X_d] = TrajGen(obj, t)
%             A = 0.5;
%             X_d = A*[cos(t) sin(t) 1+t/k -sin(t) cos(t) 1/k]';
            X_d = [5 5 10 0 0 0]';
        end        
        
        function X_dot = Dynamics(obj, t, X)
            X_d = obj.TrajGen(t);
            I_d = obj.World2Img(X_d);
            I = obj.World2Img(X);
            U = obj.Controller.GetU(I_d, I);
            %U = clf_cbf_img_controller(t,I_d,I);
            %U = linear_img_controller(I_d,I);
            X_dot = [X(4:6); U(1:3)/obj.m];
            %disp(t);
        end
        
        function [J,J_dot] = GetJacobianFromX(obj, X)
            x=X(1); y=X(2); z=X(3); x_dot=X(4); y_dot=X(5); z_dot=X(6);            
            J = [ -obj.fx/z,     0, obj.cx/z - (obj.cx*z - obj.fx*x + obj.fx*obj.xf1)/z^2;
                    0, -obj.fy/z, obj.cy/z - (obj.cy*z - obj.fy*y + obj.fy*obj.yf1)/z^2;
                  -obj.fx/z,     0, obj.cx/z - (obj.cx*z - obj.fx*x + obj.fx*obj.xf2)/z^2;
                    0, -obj.fy/z, obj.cy/z - (obj.cy*z - obj.fy*y + obj.fy*obj.yf2)/z^2];
            J_dot = [(obj.fx*z_dot)/z^2,              0, (obj.fx*(x_dot*z - 2*x*z_dot + 2*obj.xf1*z_dot))/z^3;
                                  0, (obj.fy*z_dot)/z^2, (obj.fy*(y_dot*z - 2*y*z_dot + 2*obj.yf1*z_dot))/z^3;
                     (obj.fx*z_dot)/z^2,              0, (obj.fx*(x_dot*z - 2*x*z_dot + 2*obj.xf2*z_dot))/z^3;
                                  0, (obj.fy*z_dot)/z^2, (obj.fy*(y_dot*z - 2*y*z_dot + 2*obj.yf2*z_dot))/z^3];
        end
        
        % F = [u1, v1, u2, u1_dot, v2_dot, u2_dot]
        function [J,J_dot] = GetJacobianFromF(obj, F)
            u1=F(1); v1=F(2); u2=F(3); u1_dot=F(4); v1_dot=F(5); u2_dot=F(6);
            J = [ (obj.cx - u1)/(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)),                                                                         0,                                             ((obj.cx - u1)^2*((obj.fx*(obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1))/(u1 - u2) - obj.fx*obj.xf1 + (obj.cx*obj.fx*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)))/(obj.cx - u1)))/(obj.fx^2*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2))^2) - (obj.cx*(obj.cx - u1))/(obj.fx*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)));
                                                                   0, (obj.fy*(obj.cx - u1))/(obj.fx*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2))), ((obj.cx - u1)^2*(obj.fy*(obj.yf1 - (obj.fx*(obj.cy - v1)*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)))/(obj.fy*(obj.cx - u1))) - obj.fy*obj.yf1 + (obj.cy*obj.fx*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)))/(obj.cx - u1)))/(obj.fx^2*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2))^2) - (obj.cy*(obj.cx - u1))/(obj.fx*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)));
                  (obj.cx - u1)/(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)),                                                                         0,                                             ((obj.cx - u1)^2*((obj.fx*(obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1))/(u1 - u2) - obj.fx*obj.xf2 + (obj.cx*obj.fx*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)))/(obj.cx - u1)))/(obj.fx^2*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2))^2) - (obj.cx*(obj.cx - u1))/(obj.fx*(obj.xf1 - (obj.cx*obj.xf1 - obj.cx*obj.xf2 + u1*obj.xf2 - u2*obj.xf1)/(u1 - u2)))];
            J_dot =[ -(u1_dot - u2_dot)/(obj.xf1 - obj.xf2),                                        0,           (obj.cx*u1_dot - obj.cx*u2_dot - 2*u1*u1_dot + u1*u2_dot + u2*u1_dot)/(obj.fx*(obj.xf1 - obj.xf2));
                                  0, -(obj.fy*(u1_dot - u2_dot))/(obj.fx*(obj.xf1 - obj.xf2)), (obj.cy*u1_dot - obj.cy*u2_dot - u1*v1_dot - u1_dot*v1 + u2*v1_dot + u2_dot*v1)/(obj.fx*(obj.xf1 - obj.xf2));
                     -(u1_dot - u2_dot)/(obj.xf1 - obj.xf2),                                        0,          -(obj.cx*u2_dot - obj.cx*u1_dot + u1*u2_dot + u2*u1_dot - 2*u2*u2_dot)/(obj.fx*(obj.xf1 - obj.xf2))];
        end
        
        function I_dot = ImgDynamics(t,I)        
        end
        
        function plot(obj, t, X)
            X_d = arrayfun(@obj.TrajGen, t, 'UniformOutput', false);
            I_d = cellfun(@obj.World2Img,X_d,'UniformOutput', false);
            I_d = cell2mat(I_d);
            X_d = cell2mat(X_d);
            X_d = reshape(X_d,6,size(X_d,1)/6)';
            I_d = reshape(I_d,8,numel(I_d)/8)';
            I = cellfun(@obj.World2Img, num2cell(X',1), 'UniformOutput', false);
            I = cell2mat(I)';
            subplot(2,2,1);
            plot(t, I_d(:,1:3),'--')
            hold on;
            plot(t, I(:,1:3))
            hold off;
            xlabel('Time');
            ylabel('I');
            legend('u1','v1','u2');
            subplot(2,2,2);
            plot(t, I_d(:,1:3)-I(:,1:3));
            xlabel('Time');
            ylabel('I_E_r_r_o_r');
            legend('u1','v1','u2');

            subplot(2,2,3);
            plot(t, X_d(:,1:3),'--')
            hold on;
            plot(t, X(:,1:3))
            hold off;
            xlabel('Time');
            ylabel('X');
            legend('x','y','z');
            subplot(2,2,4);
            plot(t, X_d(:,1:3)-X(:,1:3));
            xlabel('Time');
            ylabel('X_E_r_r_o_r');
            legend('x','y','z'); 
        end
    end
end