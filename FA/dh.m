%% ARMS

T_inter = trotx(0);
[t_inter, R_inter] = get_tR(T_inter);
q_inter = quaternion(R_inter,'rotmat','frame');

T_inter_0 = T_inter * trotx(pi/2)*trotz(pi/2);
[t_inter_0, R_inter_0] = get_tR(T_inter_0);
q_inter_0 = quaternion(R_inter_0,'rotmat','frame');

T_inter_1 = T_inter_0*trotx(pi/2)*trotz(pi/2);
[t_inter_1, R_inter_1] = get_tR(T_inter_1);
q_inter_1 = quaternion(R_inter_1,'rotmat','frame');

T_inter_2 = T_inter_1*transl(0,2,0) *trotx(pi/2)*trotz(pi);
[t_inter_2, R_inter_2] = get_tR(T_inter_2);
q_inter_2 = quaternion(R_inter_2,'rotmat','frame');

T_inter_3 = T_inter_2* trotx(pi/2)*trotz(pi);
[t_inter_3, R_inter_3] = get_tR(T_inter_3);
q_inter_3 = quaternion(R_inter_3,'rotmat','frame');

T_inter_4 = T_inter_3*transl(0,2,0) * trotx(pi/2)*trotz(pi/2);
[t_inter_4, R_inter_4] = get_tR(T_inter_4);
q_inter_4 = quaternion(R_inter_4,'rotmat','frame');

T_inter_5 = T_inter_4 * trotx(pi/2)*trotz(pi/2);
[t_inter_5, R_inter_5] = get_tR(T_inter_5);
q_inter_5 = quaternion(R_inter_4,'rotmat','frame');



figures(1)=figure;
plotTransforms(t_inter',q_inter)
hold on
plotTransforms(t_inter_0',q_inter_0)
plotTransforms(t_inter_1',q_inter_1)
plotTransforms(t_inter_2',q_inter_2)
plotTransforms(t_inter_3',q_inter_3)
plotTransforms(t_inter_4',q_inter_4)
plotTransforms(t_inter_5',q_inter_5)

grid on;

%%

T_inter = trotx(pi/2);
[t_inter, R_inter] = get_tR(T_inter);
q_inter = quaternion(R_inter,'rotmat','frame');

T_inter_0 = T_inter_0 * trotx(pi/2);
[t_inter_0, R_inter_0] = get_tR(T_inter_0);
q_inter_0 = quaternion(R_inter_0,'rotmat','frame');

T_inter_1 = T_inter_0*trotx(pi/2)*trotz(pi/2);
[t_inter_1, R_inter_1] = get_tR(T_inter_1);
q_inter_1 = quaternion(R_inter_1,'rotmat','frame');

T_inter_2 = T_inter_1*transl(0,2,0);
[t_inter_2, R_inter_2] = get_tR(T_inter_2);
q_inter_2 = quaternion(R_inter_2,'rotmat','frame');

T_inter_3 = T_inter_2*transl(0,2,0);
[t_inter_3, R_inter_3] = get_tR(T_inter_3);
q_inter_3 = quaternion(R_inter_3,'rotmat','frame');

T_inter_4 = T_inter_3 * trotx(-pi/2)*trotz(pi/2);
[t_inter_4, R_inter_4] = get_tR(T_inter_4);
q_inter_4 = quaternion(R_inter_4,'rotmat','frame');



figures(1)=figure;
plotTransforms(t_inter',q_inter)
hold on
plotTransforms(t_inter_0',q_inter_0)
plotTransforms(t_inter_1',q_inter_1)
plotTransforms(t_inter_2',q_inter_2)
plotTransforms(t_inter_3',q_inter_3)
plotTransforms(t_inter_4',q_inter_4)
% 


grid on
%%
function t = get_t(T)
    t = T(1:3,4);
end
function R = get_R(T)
    R = T(1:3,1:3);
end
function [t,R] = get_tR(T)
    t = get_t(T);
    R = get_R(T);
end
