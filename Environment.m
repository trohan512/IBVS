classdef Environment
    properties
        EnvironmentName;
        Controller;
    end
    methods
        function obj = SetController(obj, Controller)
            obj.Controller = Controller;
        end
        
        function X = Img2World(obj, I)
        end

        % X = [x; y; z; x_dot; y_dot; z_dot;]
        % I = [u1; v1; u2; v2; u1_dot; v1_dot; u2_dot; v2_dot];
        function I = World2Img(obj, X)
            x = X(1); y=X(2); z=X(3);
            u1 = obj.cx + obj.fx*(obj.xf1-x)/z;
            v1 = obj.cy + obj.fy*(obj.yf1-y)/z;
            u2 = obj.cx + obj.fx*(obj.xf2-x)/z;
            v2 = obj.cy + obj.fy*(obj.yf2-y)/z;
            [J,~] = obj.GetJacobianFromX(X);
            I =  [u1; v1; u2; v2; J*X(4:6)];
        end
    end
    methods (Abstract)

        X_dot = Dynamics(t,X);

        I_dot = ImgDynamics(t,I);
        
        [J,J_dot] = GetJacobianFromX(X);
        
        [J,J_dot] = GetJacobianFromF(F);
        
        [X_d] = TrajGen(t);
    end
end