classdef Environment
    properties
        EnvironmentName;
        Controller;
        X1_cir_obs;
        fx=350; fy=350; cx=0; cy=0; xf1=0.1; yf1=0; xf2=-0.1; yf2=-0.1; 
        X_f;
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

        function I1_cir_obs = GetCirObsInImg(obj, X1_cam, X1_cir_obs)
            center = obj.World2ImgCord(X1_cam,X1_cir_obs(1:3));
            border = obj.World2ImgCord(X1_cam,[X1_cir_obs(1)+X1_cir_obs(4);
                                               X1_cir_obs(2);
                                               X1_cir_obs(3)]);
            r0 = norm(center-border);
            I1_cir_obs = [center; r0];
        end
        
        function X1_cam = Img2World(obj, I_f)
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