classdef ImgCLFController < ImgController    
    methods
        function obj = ImgCLFController(Environment)
            obj.ControllerName = 'CLF';
            obj.Environment = Environment;
        end
        % [u] = Control Input
        % [I_d] = Desired Position in Img Space (u1,v1,u2)_d
        % [I] = Current Position in Img Space (u1,v1,u2)
        function u = GetU(obj, I_d, I)
            F_d = [I_d(1:3); I_d(5:7)];
            F = [I(1:3); I(5:7)];
            V_weight = [    1.0124   -0.0000   -0.0104    0.0126    0.0000   -0.0105
                -0.0000    1.0033   -0.0000   -0.0000    0.0033    0.0000
                -0.0104   -0.0000    1.0124   -0.0105   -0.0000    0.0126
                0.0126   -0.0000   -0.0105    0.0128    0.0000   -0.0108
                0.0000    0.0033   -0.0000    0.0000    0.0033   -0.0000
                -0.0105    0.0000    0.0126   -0.0108   -0.0000    0.0128];
            V = (F-F_d)'*V_weight*(F-F_d);
            dV_dx = 2*V_weight*(F-F_d);
            [J,J_dot]= obj.Environment.GetJacobianFromF(F);
            %x_dot = [x(4:6,1); 1/m]; % Missing u
            LfV = dV_dx'*[zeros(3,3),eye(3,3);
            zeros(3,3), J_dot*inv(J)]*F;
            LgV = dV_dx'*[zeros(3,3); J/obj.Environment.m];
            gamma = 1;
            b = -LfV - gamma*V;
            A = LgV;
            u = quadprog(eye(3),[0;0;0],A,b);
        end
    end
end