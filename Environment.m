classdef Environment
    properties
        EnvironmentName;
        Controller;
        X1_cir_obs; % [X0,Y0,Z0,R0]' in world
        fx=1045.83; fy=1045.83; cx=514.41; cy=357.50; 
        % X_f = [Xf1, Xf2, ...] 3-D cordinates of feature in the world
        X_f = [0.1, -0.1;
                 0, -0.1;
                 0,   0];
    end
    
    methods
        function obj = SetController(obj, Controller)
            obj.Controller = Controller;
        end
        
        % X1_cir_obs = [x1,y1,z1,r1]';
        % (x1,y1,z1) = center of sphere
        % r1 = radius of sphere
        function obj = SetCirObs(obj, X1_cir_obs)
            obj.X1_cir_obs = X1_cir_obs;
        end
        
        % Returns the image cordinate of a point in world for a given
        % pointmass position
        % X1_cam = [x; y; z] of pointmass/camera
        % X1_f = [xp; yp; zp] of point
        % I1_f = [u1; v1] of point P
        function I1_f = World2ImgCord(obj, X1_cam, X1_f)
            x=X1_cam(1); y=X1_cam(2); z=X1_cam(3);
            u1 = obj.cx - obj.fx*(X1_f(1)-x)/(X1_f(3)-z);
            v1 = obj.cy - obj.fy*(X1_f(2)-y)/(X1_f(3)-z);
            I1_f =  [u1; v1];
        end

        function I1_cir_obs_bar = GetCirObsInImg(obj, X1_bar_cam, X1_cir_obs)
            center = obj.World2ImgCord(X1_bar_cam(1:3),X1_cir_obs(1:3));
            border = obj.World2ImgCord(X1_bar_cam(1:3),[X1_cir_obs(1)+X1_cir_obs(4);
                                               X1_cir_obs(2);
                                               X1_cir_obs(3)]);
            r0 = norm(center-border);
            [J1, ~]= obj.GetJ1(X1_bar_cam, X1_cir_obs(1:3));
            I1_cir_obs_vel = J1*X1_bar_cam(4:6);
            I1_cir_obs_bar = [center; r0; I1_cir_obs_vel];
        end
        
        function X1_cam = Img2World(obj, I_f)
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
    methods (Abstract)

        X_bar_dot = Dynamics(t,X_bar);

        I_bar_dot = ImgDynamics(t,I_bar);
        
        [J1,J1_dot] = GetJ1(X_cam);
        
        X_bar_cam = GetXBarCam(I_bar_f);
        
        [X_bar_d] = TrajGen(t);
    end
end