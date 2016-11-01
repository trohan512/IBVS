clc;  clear variables;
env = PointmassEnvironment;
controller = ImgCLFController(env);
env = env.SetController(controller);
X0 = [0.05,0.05,2,0,0,0]';
[t1, X1] = ode23(@env.Dynamics, [0 20], X0);
env.plot(t1,X1);

