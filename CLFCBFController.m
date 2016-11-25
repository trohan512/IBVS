classdef CLFCBFController < Controller    
    properties
        P
    end
    
    methods
        function obj = CLFCBFController(Environment)
            obj.ControllerName = 'CLF';
            obj.Environment = Environment;
        end
        
        function u = GetU(obj, I_bar_d, I_bar, X_bar_cam)
            obj.P = obj.GetP(X_bar_cam);
            n_features = size(I_bar,1)/4;
            [f,g, ~] = obj.Environment.Getfg(I_bar, X_bar_cam);
            
            % CLF
            I_sub_bar_d = [I_bar_d(1:3); I_bar_d(2*n_features+1:2*n_features+3)]; 
            I_sub_bar = [I_bar(1:3); I_bar(2*n_features+1:2*n_features+3)]; 
            f_sub = [f(1:3); f(2*n_features+1:2*n_features+3)];
            g_sub = [g(1:3,:); g(2*n_features+1:2*n_features+3,:)];
            V = (I_sub_bar-I_sub_bar_d)'*obj.P*(I_sub_bar-I_sub_bar_d);
            dV_dx = 2*obj.P*(I_sub_bar-I_sub_bar_d);
            LfV = dV_dx'*f_sub;
            LgV = dV_dx'*g_sub;
            gamma = 1;
            b = -LfV - gamma*V;
            A = [LgV -1];
            gamma = 0.1;
            % CBF
            I1_cir_obs = obj.Environment.GetCirObsInImg(X_bar_cam, obj.Environment.X1_cir_obs);
            
            for i = 1:n_features
%             for i = 2:2
                I1_bar = [I_bar(2*i-1:2*i); I_bar(2*n_features+(2*i-1):2*n_features+(2*i))];
                [B1, dB1dx] = CBFImgPointMass(I1_bar, I1_cir_obs);
                f_sub = [f(2*i-1:2*i); f(2*n_features+2*i-1:2*n_features+2*i)];
                g_sub = [g(2*i-1:2*i,:); g(2*n_features+2*i-1:2*n_features+2*i,:)];
                LfB = dB1dx*f_sub;
                LgB = dB1dx*g_sub;
                A = [A; LgB 0];
                b = [b; -LfB + gamma/B1];
            end
            umin=-1;umax=1;
%             A = [A;1 0 0 0; 0 1 0 0; 0 0 1 0];
%             A = [A;-1 0 0 0; 0 -1 0 0; 0 0 -1 0];
%             b = [b; umax; umax; umax];
%             b = [b; umin; umin; umin];
            u = quadprog(diag([1,1,1,100]),[0;0;0;0],A,b);
            disp(sum(abs(I_bar-I_bar_d)));
%             figure(9);
%             hold on;
%             plot(norm(X_bar_cam(1:3)-obj.Environment.X1_cir_obs(1:3))-obj.Environment.X1_cir_obs(4));
%             drawnow;
%             disp(norm(X_bar_cam(1:3)-obj.Environment.X1_cir_obs(1:3))-obj.Environment.X1_cir_obs(4))
        end
        % finds the Quadratic Lyapunov Function according to desired
        % trajectory
        function P = GetP(obj, X_bar)
            I_bar = obj.Environment.World2Img(X_bar);
            [~,g,A] = obj.Environment.Getfg(I_bar , X_bar);
            A= [A(1:3,1:3), A(1:3,5:7); A(5:7,1:3),A(5:7,5:7)];
            [K,P,e] = lqr(A,[g(1:3,:);g(5:7,:)],10*eye(size(A,1)),eye(size(g,2)));
        end
    end
end