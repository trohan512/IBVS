classdef ImgCLFCBFController < ImgController    
    methods
        function obj = ImgCLFCBFController(Environment)
            obj.ControllerName = 'CLF-CBF';
            obj.Environment = Environment;
        end
        
        % [u] = Control Input
        % [I_d] = Desired Position in Img Space (u1,v1,u2)_d
        % [I] = Current Position in Img Space (u1,v1,u2)
        function u = GetU(obj, I_d, I)
            F_d = [I_d(1:3); I_d(5:7)];
            F = [I(1:3); I(5:7)];
            F1_bar = [I(1:2); I(5:6)];
            F2_bar = [I(3:4); I(7:8)];
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
            A = [LgV -1];
            I_obs = obj.Environment.getObstalceInImg([2,2,2]');
            [B1, dB1dx] = CBFImgPointMass(F1_bar,I_obs);
            A = [A; dB1dx()*F1_bar()];
            b = [b;]
            [B2, dB2dx] = CBFImgPointMass(F2_bar,I_obs);
            u = quadprog(diag(1,1,1,1000),[0;0;0;0],A,b);
        end
    end
end