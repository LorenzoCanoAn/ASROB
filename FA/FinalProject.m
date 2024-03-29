
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   LEG MODEL AS AN ARTICULATED MECHANISM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%38.6%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
id_fig = 1;
%%


figure(id_fig);
id_fig = id_fig +1;

i_hip_rot = Revolute('a',0,'alpha',pi/2,'qlim',[0 pi/2]); % DH parameters for 0T1
i_hip_ad = Revolute('a',0,'alpha',pi/2,'qlim',[-pi/2,pi],'offset',pi/2);
i_hip_flx = Revolute('a',2,'alpha',0,'qlim',[-pi pi],'offset',-pi);

i_knee = Revolute('a',2,'alpha',0,'qlim',[pi 2*pi]);

i_ankle_inv = Revolute('a',0,'alpha',-pi/2,'qlim',[-pi/2 30*pi/180], 'offset',pi/2);
i_ankle_ext = Revolute('a',1,'alpha',0,'qlim',[-pi/2 pi/2],'offset',0);

izq_leg=SerialLink([i_hip_rot,i_hip_ad,i_hip_flx,i_knee,i_ankle_inv,i_ankle_ext],'name', 'izqda_leg');
il_qr=[0, 0, 0, 0, 0, 0];
izq_leg.base=transl(0,1,0)*trotz(-pi/2);

izq_leg.plot(il_qr)
teach(izq_leg,il_qr)

%%

hold on;
d_hip_rot = Revolute('a',0,'alpha',pi/2,'qlim',[-pi/2 0],'offset', 0); % DH parameters for 0T1
d_hip_ad = Revolute('a',0,'alpha',pi/2,'qlim',[-pi/2,pi],'offset',pi/2);
d_hip_flx = Revolute('a',2,'alpha',0,'qlim',[-pi pi],'offset',-pi);

d_knee = Revolute('a',2,'alpha',0,'qlim',[pi 2*pi]);

d_ankle_inv = Revolute('a',0,'alpha',-pi/2,'qlim',[-pi/2 30*pi/180], 'offset',pi/2);
d_ankle_ext = Revolute('a',1,'alpha',0,'qlim',[-pi/2 pi/2],'offset',0);

dcha_leg=SerialLink([d_hip_rot,d_hip_ad,d_hip_flx,d_knee,d_ankle_inv,d_ankle_ext],'name', 'dcha_leg');
dl_qr=[0, 0, 0, 0, 0, 0];
dcha_leg.base=transl(0,-1,0)*trotz(-pi/2);
dcha_leg.plot(dl_qr)
teach(dcha_leg,dl_qr)

%%

i_shoulder_flx = Revolute('a',0,'alpha',pi/2,'qlim',[0 pi],'offset',pi/2); % DH parameters for 0T1
i_shoulder_ad = Revolute('a',0,'alpha',pi/2,'qlim',[0 ,pi],'offset',pi/2);
i_shoulder_rot = Revolute('a',2,'alpha',pi/2,'qlim',[-pi pi/2],'offset',pi);

i_elbow_pron  = Revolute('a',0,'alpha',pi/2,'offset',pi,'qlim',[-pi/2 pi/2]);
i_elbow_flex  = Revolute('a',2,'alpha',pi/2,'offset',pi,'qlim',[-pi/2 pi/2]);

i_wrist_flex  = Revolute('a',0,'alpha',pi/2,'offset',pi/2,'qlim',[-pi/2 pi/2]);
i_wrist_abd   = Revolute('a',1,'alpha',0,'offset',0,'qlim',[-pi/2 pi/2]);


izq_arm=SerialLink([i_shoulder_flx,i_shoulder_ad,i_shoulder_rot,i_elbow_pron,i_elbow_flex,i_wrist_flex,i_wrist_abd],'name', 'izqda_arm');
ia_qr=[0,0,0,0,0,0,0];
izq_arm.base=transl(0,1.5,5)*trotz(-pi/2);
izq_arm.plot(ia_qr)
teach(izq_arm,ia_qr)


%%

d_shoulder_flx = Revolute('a',0,'alpha',pi/2,'qlim',[0 pi],'offset',pi/2); % DH parameters for 0T1
d_shoulder_ad = Revolute('a',0,'alpha',pi/2,'qlim',[0 ,pi],'offset',pi/2);
d_shoulder_rot = Revolute('a',2,'alpha',pi/2,'qlim',[-pi pi/2],'offset',pi);

d_elbow_pron  = Revolute('a',0,'alpha',pi/2,'offset',pi,'qlim',[-pi/2 pi/2]);
d_elbow_flex  = Revolute('a',2,'alpha',pi/2,'offset',pi,'qlim',[-pi/2 pi/2]);

d_wrist_flex  = Revolute('a',0,'alpha',pi/2,'offset',pi/2,'qlim',[-pi/2 pi/2]);
d_wrist_abd   = Revolute('a',1,'alpha',0,'offset',0,'qlim',[-pi/2 pi/2]);


dcha_arm=SerialLink([d_shoulder_flx,d_shoulder_ad,d_shoulder_rot,d_elbow_pron,d_elbow_flex,d_wrist_flex,d_wrist_abd],'name', 'dcha_arm');
da_qr=[0,0,0,0,0,0,0];
dcha_arm.base=transl(0,-1.5,5)*trotz(-pi/2);
dcha_arm.plot(da_qr)
teach(dcha_arm,da_qr)




%% ARMS Dynamics
vela_d = []; Ta_d = [fkine(dcha_leg,dl_qr)];
vela_i = []; Ta_i = [fkine(izq_leg,il_qr)];

cart_arm_l = [];
cart_arm_r = [];

%init   #arm forward
t = 20; ts = 30;
q_arm_ii = ia_qr;
q_arm_if = [0,pi/4,-pi/10,pi/4,0,-pi/2,0];
q_arm_i_e=jtraj(q_arm_ii,q_arm_if,t);  

vela_i = [vela_i; repmat((q_arm_ii - q_arm_if)/t*ts,t,1)];



Til = fkine(izq_arm,q_arm_ii);
Ttl = fkine(izq_arm,q_arm_if);
cart_arm_l = [cart_arm_l, ctraj(Til,Ttl,t)];
Ta_i = [Ta_i; Ttl];


%-------------------
t= 20;
q_arm_di = da_qr;
q_arm_df = [0,pi/4,pi/10,pi/4,0,-pi/2,0];
q_arm_d_e=jtraj(q_arm_di,q_arm_df,20);

vela_d = [vela_d;  repmat((q_arm_di - q_arm_df)/t*ts,t,1)];


Tir = fkine(dcha_arm,q_arm_di);
Ttr = fkine(dcha_arm,q_arm_df);
cart_arm_r = [cart_arm_r, ctraj(Tir,Ttr,t)];
Ta_d = [Ta_d; Ttr];


%  ---------------------     #arm backwards
t= 30;
q_arm_ii = q_arm_if;
q_arm_if = [0,-pi/4,pi/10,0,0,-pi/2,0];
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,t)];  
 

vela_i = [vela_i; repmat((q_arm_ii - q_arm_if)/t*ts,t,1)];


Til = Ttl;
Ttl = fkine(izq_arm,q_arm_if);
cart_arm_l = [cart_arm_l, ctraj(Til,Ttl,t)];
Ta_i = [Ta_i; Ttl];

%----------


q_arm_di = q_arm_df;
q_arm_df = [0,-pi/4,-pi/10,0,0,-pi/2,0];
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,t)];  


vela_d = [vela_d; repmat((q_arm_di - q_arm_df)/t*ts,t,1)];


Tir = Ttr;
Ttr = fkine(dcha_arm,q_arm_df);
cart_arm_r = [cart_arm_r, ctraj(Tir,Ttr,t)];
Ta_d = [Ta_d; Ttr];


%   ----------------------    #arm top
t=40;
q_arm_ii = q_arm_if;
q_arm_if = [pi/12,pi-pi/10,pi/4,0,-pi/2,-pi/2,0];
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,t)];  


vela_i = [vela_i;  repmat((q_arm_ii - q_arm_if)/t*ts,t,1)];


Til = Ttl;
Ttl = fkine(izq_arm,q_arm_if);
cart_arm_l = [cart_arm_l, ctraj(Til,Ttl,t)];
Ta_i = [Ta_i; Ttl];


%------------
q_arm_di = q_arm_df;
q_arm_df = [0,pi-pi/10,0,0,0,-pi/2,0];
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,t)];  


vela_d = [vela_d; repmat((q_arm_di - q_arm_df)/t*ts,t,1)];


Tir = Ttr;
Ttr = fkine(dcha_arm,q_arm_df);
cart_arm_r = [cart_arm_r, ctraj(Tir,Ttr,t)];
Ta_d = [Ta_d; Ttr];

%  --------------     #spike

t=9;
q_arm_ii = q_arm_if;
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,t)];

vela_i = [vela_i;  repmat((q_arm_ii - q_arm_if)/t*ts,t,1)];


Til = Ttl;
Ttl = fkine(izq_arm,q_arm_if);
cart_arm_l = [cart_arm_l, ctraj(Til,Ttl,t)];
Ta_i = [Ta_i; Ttl];


t=15;
q_arm_ii = q_arm_if;
q_arm_if = [0,0,0,0,0,-pi/2,0];
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,t)];  


vela_i = [vela_i;  repmat((q_arm_ii - q_arm_if)/t*ts,t,1)];


Til = Ttl;
Ttl = fkine(izq_arm,q_arm_if);
cart_arm_l = [cart_arm_l, ctraj(Til,Ttl,t)];
Ta_i = [Ta_i; Ttl];


%------------------------------------

t=15;
q_arm_di = q_arm_df;
q_arm_df = [0,0,0,0,0,-pi/2,0];
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,t)];  

vela_d = [vela_d;  repmat((q_arm_di - q_arm_df)/t*ts,t,1)];



Tir = Ttr;
Ttr = fkine(dcha_arm,q_arm_df);
cart_arm_r = [cart_arm_r, ctraj(Tir,Ttr,t)];
Ta_d = [Ta_d; Ttr];

t=9;
q_arm_di = q_arm_df;
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,t)];  


vela_d = [vela_d;  repmat((q_arm_di - q_arm_df)/t*ts,t,1)];

Tir = Ttr;
Ttr = fkine(dcha_arm,q_arm_df);
cart_arm_r = [cart_arm_r, ctraj(Tir,Ttr,t)];
Ta_d = [Ta_d; Ttr];



vela_d = [vela_d; [0,0,0,0,0,0,0]];
vela_i = [vela_i; [0,0,0,0,0,0,0]];



%% LEGS Dynamics
vell_i = []; Tl_i = [fkine(izq_leg,il_qr)];
vell_d = []; Tl_d = [fkine(dcha_leg,dl_qr)];

cart_leg_l = [];
cart_leg_r = [];

%-------------------------
t=30;
q_leg_ii = il_qr;
q_leg_if = [0,0,0,-pi/5,-pi/7,0];
q_leg_i_e=jtraj(q_leg_ii,q_leg_if,t);  


vell_i = [vell_i; repmat((q_leg_ii - q_leg_if)/t*ts,t,1)];

Ti = fkine(izq_leg,q_leg_ii);
Tt = fkine(izq_leg,q_leg_if);
cart_leg_l = [cart_leg_l, ctraj(Ti,Tt,t)];
Tl_i = [Tl_i; Tt];

%-----------------
t=10;
q_leg_ii = q_leg_if;
q_leg_if = [0,0,pi/4,0,0,0];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,t)];  


vell_i = [vell_i; repmat((q_leg_ii - q_leg_if)/t*ts,t,1)];


Ti = Tt;
Tt = fkine(izq_leg,q_leg_if);
cart_leg_l = [cart_leg_l, ctraj(Ti,Tt,t)];
Tl_i = [Tl_i;Tt];
%--------------------------------------

t=10;
q_leg_ii = q_leg_if;
q_leg_if = [0,0,0,0,0,0];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,t)];  

vell_i = [vell_i; repmat((q_leg_ii - q_leg_if)/t*ts,t,1)];


Ti = Tt;
Tt = fkine(izq_leg,q_leg_if);
cart_leg_l = [cart_leg_l , ctraj(Ti,Tt,t)];
Tl_i = [Tl_i; Tt];

%---------------------------------------

t=15;
q_leg_ii = q_leg_if;
q_leg_if =[0,0,pi/5,-pi/5,-pi/7,0];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,t)];  

vell_i = [vell_i; repmat((q_leg_ii - q_leg_if)/t*ts,t,1)];


Ti = Tt;
Tt = fkine(izq_leg,q_leg_if);
cart_leg_l = [cart_leg_l , ctraj(Ti,Tt,t)];
Tl_i = [Tl_i; Tt];

%----------------------
t=8;
q_leg_ii = q_leg_if;
q_leg_if =[0,0,0,0,-pi/2,0];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,8)];  


vell_i = [vell_i; repmat((q_leg_ii - q_leg_if)/t*ts,t,1)];


Ti = Tt;
Tt = fkine(izq_leg,q_leg_if);
cart_leg_l = [cart_leg_l , ctraj(Ti,Tt,t)];
Tl_i = [Tl_i; Tt];

%-------------------------

t=30;
q_leg_di = il_qr;
q_leg_df = [0,0,pi/4,0,0,0,];
q_leg_d_e=jtraj(q_leg_di,q_leg_df,30);  


vell_d = [vell_d; repmat((q_leg_di - q_leg_df)/t*ts,t,1)];


Ti = fkine(dcha_leg,q_leg_di);
Tt = fkine(dcha_leg,q_leg_df);
cart_leg_r = [cart_leg_r , ctraj(Ti,Tt,t)];
Tl_d = [Tl_d; Tt];

%-----------------------------------
t=20;
q_leg_di = q_leg_df;
q_leg_df = [0,0,0,0,0,0];
q_leg_d_e=[q_leg_d_e;jtraj(q_leg_di,q_leg_df,t)];  


vell_d = [vell_d; repmat((q_leg_di - q_leg_df)/t*ts,t,1)];

Ti = Tt;
Tt = fkine(dcha_leg,q_leg_df);
cart_leg_r = [cart_leg_r , ctraj(Ti,Tt,t)];
Tl_d = [Tl_d; Tt];

%-----------------------------------------


t=15;
q_leg_di = q_leg_df;
q_leg_df = [0,0,pi/5,-pi/5,-pi/7,0];
q_leg_d_e=[q_leg_d_e;jtraj(q_leg_di,q_leg_df,t)];


vell_d = [vell_d; repmat((q_leg_di - q_leg_df)/t*ts,t,1)];

Ti = Tt;
Tt = fkine(dcha_leg,q_leg_df);
cart_leg_r = [cart_leg_r , ctraj(Ti,Tt,t)];
Tl_d = [Tl_d; Tt];

%-----------------------------------------

t=8;
q_leg_di = q_leg_df;
q_leg_df = [0,0,0,0,-pi/2,0];
q_leg_d_e=[q_leg_d_e;jtraj(q_leg_di,q_leg_df,t)];


vell_d = [vell_d; repmat((q_leg_di - q_leg_df)/t*ts,t,1)];

Ti = Tt;
Tt = fkine(dcha_leg,q_leg_df);
cart_leg_r = [cart_leg_r,  ctraj(Ti,Tt,t)];
Tl_d = [Tl_d; Tt];

%-----------------------------------------



vell_d = [vell_d; [0,0,0,0,0,0]];
vell_i = [vell_i; [0,0,0,0,0,0]];



%%
len_l = size(q_leg_i_e);
len_a = size(q_arm_i_e);
anim1 = Animate('movie.mp4');
for i = 1:max(len_l(1),len_a(1))
    if i <= len_l(1)
        izq_leg.animate(q_leg_i_e(i,:));
        dcha_leg.animate(q_leg_d_e(i,:));
    end
    if i <= len_a(1)
        izq_arm.animate(q_arm_i_e(i,:));
        dcha_arm.animate(q_arm_d_e(i,:));
    end
    anim1.add()
end
anim1.close()



%%

figure(id_fig);
id_fig = id_fig +1;
plot(vela_d);
title('Radian/s vel right arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');

figure(id_fig);
id_fig = id_fig +1;
plot(vela_i);
title('Radian/s vel left arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');


%%
pos_l = [];
pos_r = [];
for i = 1:length(cart_arm_l)
    pos_l = [pos_l;cart_arm_l(i).t'];
    pos_r = [pos_r;cart_arm_r(i).t'];
end

pos_l = [pos_l;pos_l(end,:)];
pos_r = [pos_r;pos_r(end,:)];


figure(id_fig);
id_fig = id_fig +1;
plot(pos_r,'-');
title("Position Right Armg")
legend('X','Y','Z');

figure(id_fig);
id_fig = id_fig +1;
plot(pos_l,'-');
title("Position Left Arm")
legend('X','Y','Z');

%% 

figure(id_fig);
id_fig = id_fig +1;
plot(q_arm_i_e,'-');
title("Position Left Arm")
legend('q1','q2','q3','q4','q5','q6','q7');

figure(id_fig);
id_fig = id_fig +1;
plot(q_arm_d_e,'-');
title("Position Right Arm")
legend('q1','q2','q3','q4','q5','q6','q7');




%%


figure(id_fig);
id_fig = id_fig +1;
plot(vell_d);
title('Radian/s right leg joints');
legend('q1','q2','q3','q4','q5','q6');

figure(id_fig);
id_fig = id_fig +1;
plot(vell_i);
title('Radian/s left leg joints');
legend('q1','q2','q3','q4','q5','q6');

%% 

figure(id_fig);
id_fig = id_fig +1;
plot(q_leg_i_e,'-');
title("Position Left Leg")
legend('q1','q2','q3','q4','q5','q6');

figure(id_fig);
id_fig = id_fig +1;
plot(q_leg_d_e,'-');
title("Position Right Leg")
legend('q1','q2','q3','q4','q5','q6');


%%
pos_l = [];
pos_r = [];
for i = 1:length(cart_leg_l)
    pos_l = [pos_l;cart_leg_l(i).t'];
    pos_r = [pos_r;cart_leg_r(i).t'];
end

pos_l = [pos_l;pos_l(end,:)];
pos_r = [pos_r;pos_r(end,:)];


figure(id_fig);
id_fig = id_fig +1;
plot(pos_r,'-');
title("Position Right Leg")
legend('X','Y','Z');

figure(id_fig);
id_fig = id_fig +1;
plot(pos_l,'-');
title("Position Left Leg")
legend('X','Y','Z');


%%


%%
for i = 1:length(Ta_d)
    se3 = Ta_d(i);
    t_inter_0 = se3.t;
    R_inter_0 = se3.R;
    q_inter_0 = quaternion(R_inter_0,'rotmat','frame');
    plotTransforms(t_inter_0',q_inter_0);
    
    text (se3.t(1),se3.t(2),se3.t(3),int2str(i));
    hold on;
    

    
    
    se3 = Ta_i(i);
    t_inter_0 = se3.t;
    R_inter_0 = se3.R;
    q_inter_0 = quaternion(R_inter_0,'rotmat','frame');
    plotTransforms(t_inter_0',q_inter_0);
    
    text (se3.t(1),se3.t(2),se3.t(3),int2str(i));
    hold on;
    
end

%%

figure(id_fig);
id_fig = id_fig +1;
grid on;
title('Reference axis arm joints')
for i = 1:length(Ta_d)
    se3 = Ta_d(i);
    t_inter_0 = se3.t;
    R_inter_0 = se3.R;
    q_inter_0 = quaternion(R_inter_0,'rotmat','frame');
    plotTransforms(t_inter_0',q_inter_0);
    
    text (se3.t(1),se3.t(2),se3.t(3),int2str(i));
    hold on;
    
    
    
    se3 = Ta_i(i);
    t_inter_0 = se3.t;
    R_inter_0 = se3.R;
    q_inter_0 = quaternion(R_inter_0,'rotmat','frame');
    plotTransforms(t_inter_0',q_inter_0);
    
    text (se3.t(1),se3.t(2),se3.t(3),int2str(i));
    hold on;
    
end
grid on;

figure(id_fig);
id_fig = id_fig +1;

grid on;
for i = 1:length(Tl_i)-1
    se3 = Tl_i(i);
    t_inter_0 = se3.t;
    R_inter_0 = se3.R;
    q_inter_0 = quaternion(R_inter_0,'rotmat','frame');
    plotTransforms(t_inter_0',q_inter_0);
    
    text (se3.t(1),se3.t(2),se3.t(3),int2str(i));
    hold on;
  

    se3 = Tl_d(i);
    t_inter_0 = se3.t;
    R_inter_0 = se3.R;
    q_inter_0 = quaternion(R_inter_0,'rotmat','frame');
    plotTransforms(t_inter_0',q_inter_0);
    
    text (se3.t(1),se3.t(2),se3.t(3),int2str(i));
    hold on;
    
    
end 
grid on;
figure;

%% Cartesian movement
%% Cartesian

len_l = size(cart_leg_l);
len_a = size(cart_arm_l);
anim = Animate('movie_c.mp4');
qil = il_qr;
qdl = dl_qr;
qia = ia_qr;
qda = da_qr;

for i = 1:max(len_l(2),len_a(2))
    if i <= len_l(2)
        
        qil_2 = ikine(izq_leg,cart_leg_l(i),qil);
        if ~isempty(qil_2)
            izq_leg.animate(qil_2);
            qil = qil_2;
        end

        qdl_2 = ikine(dcha_leg,cart_leg_r(i),qdl);
        if ~isempty(qdl)
            dcha_leg.animate(qdl_2);
            qdl = qdl_2;
        end
        
    end
    if i <= len_a(2)
        qia_2 = ikine(izq_arm,cart_arm_l(i),qia);
        if ~isempty(qia_2)
        izq_arm.animate(qia_2);
        qia = qia_2;
        end
        
        qda_2 = ikine(dcha_arm,cart_arm_r(i),qda);
        if ~isempty(qda_2)
            dcha_arm.animate(qda_2);
            qda = qda_2;
        end
    end
    anim.add()
end
anim.close()


