clc;  clear variables;
env = PointmassEnvironment;
env = env.SetCirObs([1,1,1,0.1]');
%disp(env.getObstalceInImg([2,2,2]'));
controller = CLFController(env);
P = [    1.0124   -0.0000   -0.0104    0.0126    0.0000   -0.0105
                -0.0000    1.0033   -0.0000   -0.0000    0.0033    0.0000
                -0.0104   -0.0000    1.0124   -0.0105   -0.0000    0.0126
                0.0126   -0.0000   -0.0105    0.0128    0.0000   -0.0108
                0.0000    0.0033   -0.0000    0.0000    0.0033   -0.0000
                -0.0105    0.0000    0.0126   -0.0108   -0.0000    0.0128];
% controller = CLFCBFController(env);
env = env.SetController(controller);
X0 = [0.05,0.05,2,0,0,0]';
[t1, X1] = ode23(@(t, x) env.Dynamics(t,x), [0 10], X0);
env.plot(t1,X1);