
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   LEG MODEL AS AN ARTICULATED MECHANISM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%38.6%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;

%%



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

%init   #arm forward
q_arm_ii = ia_qr;
q_arm_if = [0,pi/4,-pi/10,pi/4,0,-pi/2,0];
q_arm_i_e=jtraj(q_arm_ii,q_arm_if,20);  



q_arm_di = da_qr;
q_arm_df = [0,pi/4,pi/10,pi/4,0,-pi/2,0];
q_arm_d_e=jtraj(q_arm_di,q_arm_df,20);


%       #arm backwards
q_arm_ii = q_arm_if;
q_arm_if = [0,-pi/4,pi/10,0,0,-pi/2,0];
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,30)];  
 


q_arm_di = q_arm_df;
q_arm_df = [0,-pi/4,-pi/10,0,0,-pi/2,0];
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,30)];  



%       #arm top
q_arm_ii = q_arm_if;
q_arm_if = [pi/12,pi-pi/10,pi/4,0,-pi/2,-pi/2,0];
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,40)];  


q_arm_di = q_arm_df;
q_arm_df = [0,pi-pi/10,0,0,0,-pi/2,0];
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,40)];  

%       #spike

q_arm_ii = q_arm_if;
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,9)];

q_arm_ii = q_arm_if;
q_arm_if = [0,0,0,0,0,-pi/2,0];
q_arm_i_e=[q_arm_i_e;jtraj(q_arm_ii,q_arm_if,15)];  



q_arm_di = q_arm_df;
q_arm_df = [0,0,0,0,0,-pi/2,0];
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,15)];  
q_arm_di = q_arm_df;
q_arm_d_e=[q_arm_d_e;jtraj(q_arm_di,q_arm_df,9)];  


%% LEGS Dynamics
 
q_leg_ii = il_qr;
q_leg_if = [0,0,0,-pi/5,-pi/7,0];
q_leg_i_e=jtraj(q_leg_ii,q_leg_if,30);  

q_leg_ii = q_leg_if;
q_leg_if = [0,0,pi/4,0,0,0,];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,10)];  


q_leg_ii = q_leg_if;
q_leg_if = [0,0,0,0,0,0,];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,10)];  

q_leg_ii = q_leg_if;
q_leg_if =[0,0,pi/5,-pi/5,-pi/7,0];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,15)];  

q_leg_ii = q_leg_if;
q_leg_if =[0,0,0,0,-pi/2,0];
q_leg_i_e=[q_leg_i_e;jtraj(q_leg_ii,q_leg_if,8)];  
 

q_leg_di = il_qr;
q_leg_df = [0,0,pi/4,0,0,0,];
q_leg_d_e=jtraj(q_leg_di,q_leg_df,30);  

q_leg_di = q_leg_df;

q_leg_df = [0,0,0,0,0,0];
q_leg_d_e=[q_leg_d_e;jtraj(q_leg_di,q_leg_df,20)];  

q_leg_di = q_leg_df;
q_leg_df = [0,0,pi/5,-pi/5,-pi/7,0];
q_leg_d_e=[q_leg_d_e;jtraj(q_leg_di,q_leg_df,15)];

q_leg_di = q_leg_df;
q_leg_df = [0,0,0,0,-pi/2,0];
q_leg_d_e=[q_leg_d_e;jtraj(q_leg_di,q_leg_df,8)];


%%
len_l = size(q_leg_i_e);
len_a = size(q_arm_i_e);
anim = Animate('movie.mp4');
for i = 1:max(len_l(1),len_a(1))
    if i <= len_l(1)
        izq_leg.animate(q_leg_i_e(i,:));
        dcha_leg.animate(q_leg_d_e(i,:));
    end
    if i <= len_a(1)
        izq_arm.animate(q_arm_i_e(i,:));
        dcha_arm.animate(q_arm_d_e(i,:));
    end
    anim.add()
end
anim.close()


