classdef PointmassEnvironment < Environment
    properties
        m=1;
    end
    methods
        function obj = PointmassEnvironment()
            obj.EnvironmentName = 'Pointmass';
            %TODO: Shift this to Environment
            obj.X_f = [obj.xf1, obj.xf2;
                   obj.yf1, obj.yf2;
                    0,   0];
        end

        function I_bar = World2Img(obj, X_bar_cam)
            num_f = size(obj.X_f,2);
            I_bar = zeros(2*num_f,1);
            for i = 1:num_f
                I_bar(2*i-1:2*i) = World2ImgCord(obj, X_bar_cam(1:3), obj.X_f(:,i));
                [J1, ~]= obj.GetJ1(X_bar_cam, obj.X_f(:,i));
                I_bar(2*num_f+2*i-1:2*num_f+2*i) = J1*X_bar_cam(4:6);
            end
        end
        
        function X_bar_dot_cam = Dynamics(obj, t, X_bar_cam)
            X_bar_d = obj.TrajGen(t);
            I_bar_d = obj.World2Img(X_bar_d);
            I_bar = obj.World2Img(X_bar_cam);
            U = obj.Controller.GetU(I_bar_d, I_bar, X_bar_cam);
            X_bar_dot_cam = [X_bar_cam(4:6); U(1:3)/obj.m];
            disp(t);
        end
        
        function I_bar_dot = ImgDynamics(t,I_bar)        
        end
        
        function [J1,J1_dot] = GetJ1(obj, X_bar_cam, X1_F)
            x=X_bar_cam(1); y=X_bar_cam(2); z=X_bar_cam(3); x_dot=X_bar_cam(4); y_dot=X_bar_cam(5); z_dot=X_bar_cam(6);            
            xf1 = X1_F(1); yf1 = X1_F(2);  
            J1 = [ -obj.fx/z,     0, obj.cx/z - (obj.cx*z - obj.fx*x + obj.fx*xf1)/z^2;
                    0, -obj.fy/z, obj.cy/z - (obj.cy*z - obj.fy*y + obj.fy*yf1)/z^2];
            J1_dot = [(obj.fx*z_dot)/z^2,              0, (obj.fx*(x_dot*z - 2*x*z_dot + 2*xf1*z_dot))/z^3;
                                  0, (obj.fy*z_dot)/z^2, (obj.fy*(y_dot*z - 2*y*z_dot + 2*yf1*z_dot))/z^3];
        end
% TODO: remove dependence on X_bar_cam   
        function [f,g,A] = Getfg(obj, I_bar, X_bar_cam)
            n_features = size(obj.X_f, 2);
            J = zeros(n_features, 3);
            J_dot = zeros(n_features, 3);
            for i=1:n_features
                [J1 , J1_dot]= obj.GetJ1(X_bar_cam, obj.X_f(:,i));
                J(2*i-1:2*i,:) = J1;
                J_dot(2*i-1:2*i,:) = J1_dot;
            end 
            A = [zeros(2*n_features,2*n_features), eye(2*n_features); zeros(2*n_features,2*n_features), J_dot*pinv(J)];
            f = A*I_bar;
            g = [zeros(2*n_features,3); J];
        end
        
        function X_bar_cam = GetXBarCam(I_bar_f)
            
        end
        
        function [X_bar_d] = TrajGen(obj, t)
%             A = 0.5;
%             X_d = A*[cos(t) sin(t) 1+t/k -sin(t) cos(t) 1/k]';
            X_bar_d = [0.5 0.5 5 0 0 0]';
        end    
        
        % animate speed = between [0,1]
        function plot(obj, t, X, animation_speed)
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
            
            figure;
            % Draw Obstacles
%             [xs,ys,zs] = sphere();
%             xs = xs*obj.X1_cir_obs(4) + obj.X1_cir_obs(1);
%             ys = ys*obj.X1_cir_obs(4) + obj.X1_cir_obs(2);
%             zs = zs*obj.X1_cir_obs(4) + obj.X1_cir_obs(3);
%             surf(xs,ys,zs,'EdgeColor','none');
%             hold on;
            % Draw Features
            for i=1:size(obj.X_f,2)
                plot3(obj.X_f(1,i),obj.X_f(2,i),obj.X_f(3,i),'r*');
                hold on;
            end
            plot3(X(:,1),X(:,2),X(:,3));
        end
    end
end