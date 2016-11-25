clear; clc;
syms u v u0 v0 r0 gamma u_dot v_dot u0_dot v0_dot; 
gamma=1;
h = (u-u0)^2 + (v-v0)^2 - r0^2;
h2 = simplify(jacobian(h,[u v u0 v0])*[u_dot; v_dot; u0_dot; v0_dot] + gamma*h)
B = 1/h2
dBdx = simplify(jacobian(B,[u v u_dot v_dot]))
m_list_F_bar = {'u', 'F_bar(1)'; 'v', 'F_bar(2)'; 'u_dot', 'F_bar(3)'; 'v_dot', 'F_bar(4)'};
m_list_params = {'u0', 'params(1)' ; ...
                 'v0', 'params(2)' ; ...
                 'r0', 'params(3)' ; ...
                 'u0_dot', 'params(4)'; ...
                 'v0_dot', 'params(5)'} ;
write_fcn_m(['..\CBFImgPointMass.m'],{'F_bar', 'params'},[m_list_F_bar; m_list_params],{B,'B'; dBdx,'dBdx'});