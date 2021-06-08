
% Observe the reference frames A and B in Figure 1:
% a) Compute ATB y BTA in a graphical way, expressing the base of the
% orientation A RB and the position vector (without inverting the matrix).

aR_ab = [-1,0,0;0,0,1;0,1,0];
at_ab = [2;7;3];

aT_ab  = [aR_ab,at_ab;0,0,0,1];


bR_ba = [-1,0,0;0,0,1;0,1,0];
bt_ba = [2;-3;-7];
bT_ba_1 = [bR_ba,bt_ba;0,0,0,1];

% b) Compute BTA numerically, from ATB , using Matlab.

bT_BA = inv(aT_ab);


% c) Imagine a concatenation of two rotations and a translation (in the order
% you prefer) that allow transforming from reference A into reference
% frame B. Plot the intermediary reference frames and express the
% associated matrix operation.



aT_ab_1 = aT_ab;
aT_ab = transl(2,7,3)*trotz(pi)*trotx(pi/2);

if all(aT_ab_1 == aT_ab)
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
q_ba_b = quaternion(bR_ba,'rotmat','frame');
T_bc_b = [0,0,-1,-1;0,1,0,0;1,0,0,2;0,0,0,1];

[t_bc_b, R_bc_b] = get_tR(T_bc_b);
q_bc_b = quaternion(R_bc_b,'rotmat','frame');

figures = [figures,figure];
plotTransforms(t_inter_0',q_inter_0)
hold on
plotTransforms(bt_ba',q_ba_b)
plotTransforms(t_bc_b',q_bc_b)

% Graph redone on paper for clarity, graphical transformation is:

t_ac_a = [3;9;3];
R_ac_a = [0,0,1;1,0,0;0,1,0];
T_ac_a = [R_ac_a,t_ac_a;0,0,0,1];

% B)

T_ac_a_1 = aT_ab * T_bc_b;
if all(T_ac_a_1 == T_ac_a)
    clear T_ac_a_1
else
    error("They are not equal")
end

% C) What  is  the  location  vector  AxC=(x,y,z,φ,θ,ψ),  that  expresses  the  orientation in Euler angles

e_ac_a = rotm2eul(get_R(T_ac_a));
x_ac_a = [t_ac_a',e_ac_a];
disp(x_ac_a)

%% 3rd Exercise

%%  Structure with 2 joints:

% Elements that define a link: 
% DH parameters: 
%   sigma (==0 rotat., == 1 transl.),
%   offset: if we want to include an initial offset for the joint 
%           [theta,D,A,alpha,sigma,offset]


%% A  
l1 = 2;
l2 = 4;

L1 = Revolute('a',0,'alpha',pi/2,'offset',30*pi/180); % DH parameters for 0T1
L2 = Prismatic(); %  DH parameters for 1T2


% Set limits for the joint
L2.qlim = [2 15];
% Construction of an object belonging to the class robot
my_arm=SerialLink([L1,L2],'name', 'robot1');
% Location of the base reference frame
my_arm.base=transl(0,2,0)*trotx(-pi/2);
% Visualization of the robot in the joint position given by qr
qr=[0, 2];
plot(my_arm,qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(my_arm,qr)

%% B

L1 = Revolute('a',1,'alpha',0,'offset',-20*pi/180); % DH parameters for 0T1
L2 = Revolute('a',2,'alpha',0,'offset',220*pi/180); %  DH parameters for 1T2


% Set limits for the joint
L2.qlim = [0 15];
% Construction of an object belonging to the class robot
my_arm=SerialLink([L1,L2],'name', 'robot1');
% Location of the base reference frame
my_arm.base=transl(0,2,0)*trotx(-pi/2);
% Visualization of the robot in the joint position given by qr
qr=[0, 2];
plot(my_arm,qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(my_arm,qr)

%% C
L1 = Revolute('d',2,'a',0,'alpha',-pi/2); % DH parameters for 0T1
L2 = Revolute('a',4,'alpha',0,'offset',pi+pi/4); %  DH parameters for 1T2


% Set limits for the joint
L2.qlim = [pi/4 pi-pi/4];
% Construction of an object belonging to the class robot
my_arm=SerialLink([L1,L2],'name', 'robot1');
% Location of the base reference frame
my_arm.base=transl(0,2,0);
% Visualization of the robot in the joint position given by qr
qr=[0, 2];
plot(my_arm,qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(my_arm,qr)


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

