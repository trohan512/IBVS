clc;  clear variables; close all;
env = PointmassEnvironment;
env = env.SetCirObs([0.1,-0.05,2,0.1]');   
%disp(env.getObstalceInImg([2,2,2]'));
P = [    1.0124   -0.0000   -0.0104    0.0126    0.0000   -0.0105
        -0.0000    1.0033   -0.0000   -0.0000    0.0033    0.0000
        -0.0104   -0.0000    1.0124   -0.0105   -0.0000    0.0126
        0.0126   -0.0000   -0.0105    0.0128    0.0000   -0.0108
        0.0000    0.0033   -0.0000    0.0000    0.0033   -0.0000
        -0.0105    0.0000    0.0126   -0.0108   -0.0000    0.0128];
controller = CLFController(env);
%controller = CLFCBFController(env);
env = env.SetController(controller);
X0 = [-0.5,-0.5,4,0,0,0]';
options = odeset('MaxStep',0.1);
[t1, X1] = ode45(@(t, x) env.Dynamics(t,x), [0 0.1], X0, options);
% dt=0.001; T=10;
% X1 = zeros(6,T/dt+1);
% X1(:,1) = X0;
% for i=2:T/dt+1
%     X1(:,i) = X1(:,i-1) + dt*env.Dynamics(dt*i,X1(:,i-1));
% end
% t1 = 0:dt:T;
env.plot(t1,X1,2);