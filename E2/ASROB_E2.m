% ----------------------------------------------------------------
% Lorenzo Cano
% Carlos Tierno
% ----------------------------------------------------------------



% Observe the reference frames A and B in Figure 1:
% a) Compute ATB y BTA in a graphical way, expressing the base of the
% orientation A RB and the position vector (without inverting the matrix).

R_ab_a = [-1,0,0;0,0,1;0,1,0];
t_ab_a = [2;7;3];

T_ab_a  = [R_ab_a,t_ab_a;0,0,0,1];


R_ba_b = [-1,0,0;0,0,1;0,1,0];
t_ba_b = [2;-3;-7];
T_ba_b_1 = [R_ba_b,t_ba_b;0,0,0,1];

% b) Compute BTA numerically, from ATB , using Matlab.

T_B_A_b = inv(T_ab_a);


% c) Imagine a concatenation of two rotations and a translation (in the order
% you prefer) that allow transforming from reference A into reference
% frame B. Plot the intermediary reference frames and express the
% associated matrix operation.



T_ab_a_1 = T_ab_a;
T_ab_a = transl(2,7,3)*trotz(pi)*trotx(pi/2);
if all(T_ab_a_1 == T_ab_a)
    clear T_ab_a_1
else
    error("They are different")
end
T_inter_0 = transl(0,0,0);
[t_inter_0, R_inter_0] = get_tR(T_inter_0);
q_inter_0 = quaternion(R_inter_0,'rotmat','frame');
T_inter_1 = transl(2,7,3);
[t_inter_1, R_inter_1] = get_tR(T_inter_1);
q_inter_1 = quaternion(R_inter_1,'rotmat','frame');
T_inter_2 = T_inter_1 * trotz(pi);
[t_inter_2, R_inter_2] = get_tR(T_inter_2);
q_inter_2 = quaternion(R_inter_2,'rotmat','frame');
T_inter_3 = T_inter_2 * trotx(pi/2);
[t_inter_3, R_inter_3] = get_tR(T_inter_3);
q_inter_3 = quaternion(R_inter_3,'rotmat','frame');

figures(1)=figure;
plotTransforms(t_inter_0',q_inter_0)
hold on
plotTransforms(t_inter_1',q_inter_1)
plotTransforms(t_inter_2',q_inter_2)
plotTransforms(t_inter_3',q_inter_3)



%% 2 


% I need t_ba_b (done), q_ba_b, t_bc_b and q_bc_b
q_ba_b = quaternion(R_ba_b,'rotmat','frame');
T_bc_b = [0,0,-1,-1;0,1,0,0;1,0,0,2;0,0,0,1];
[t_bc_b, R_bc_b] = get_tR(T_bc_b);
q_bc_b = quaternion(R_bc_b,'rotmat','frame');
figures = [figures,figure];
plotTransforms(t_inter_0',q_inter_0)
hold on
plotTransforms(t_ba_b',q_ba_b)
plotTransforms(t_bc_b',q_bc_b)

% Graph redone on paper for clarity, graphical transformation is:
t_ac_a = [3;9;3];
R_ac_a = [0,0,1;1,0,0;0,1,0];
T_ac_a = [R_ac_a,t_ac_a;0,0,0,1];

% B)

T_ac_a_1 = T_ab_a * T_bc_b;
if all(T_ac_a_1 == T_ac_a)
    clear T_ac_a_1
else
    error("They are not equal")
end

% C) 

e_ac_a = rotm2eul(get_R(T_ac_a));
x_ac_a = [t_ac_a',e_ac_a];
disp(x_ac_a)



%% Functions

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

