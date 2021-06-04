
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   LEG MODEL AS AN ARTICULATED MECHANISM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%38.6%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;

hip_rot = Revolute('a',0,'alpha',pi/2,'qlim',[0 pi/2]); % DH parameters for 0T1
hip_ad = Revolute('a',0,'alpha',pi/2,'qlim',[-pi/2,pi],'offset',pi/2);
hip_flx = Revolute('a',2,'alpha',pi/2,'qlim',[-pi 0],'offset',-pi);

knee = Revolute('a',2,'alpha',0,'qlim',[pi 2*pi]);

ankle_inv = Revolute('a',0,'alpha',-pi/2,'qlim',[-pi/2 30*pi/180], 'offset',pi/2);
ankle_ext = Revolute('a',1,'alpha',0,'qlim',[-pi/2 pi/2],'offset',pi/2);

izq_leg=SerialLink([hip_rot,hip_ad,hip_flx,knee,ankle_inv,ankle_ext],'name', 'izqda_leg');
qr=[0, 0, 0, 0, 0, 0];
izq_leg.base=transl(0,1,0)*trotz(-pi/2);

izq_leg.plot(qr)
teach(izq_leg,qr)

%%

hold on;
hip_rot = Revolute('a',0,'alpha',pi/2,'qlim',[-pi/2 0],'offset', 0); % DH parameters for 0T1
hip_ad = Revolute('a',0,'alpha',pi/2,'qlim',[-pi/2,pi],'offset',pi/2);
hip_flx = Revolute('a',2,'alpha',pi/2,'qlim',[-pi 0],'offset',-pi);

ankle_inv = Revolute('a',0,'alpha',-pi/2,'qlim',[-pi/2 30*pi/180], 'offset',pi/2);
ankle_ext = Revolute('a',1,'alpha',0,'qlim',[-pi/2 pi/2],'offset',pi/2);

dcha_leg=SerialLink([hip_rot,hip_ad,hip_flx,knee,ankle_inv,ankle_ext],'name', 'dcha_leg');
qr=[0, 0, 0, 0, 0, 0];
dcha_leg.base=transl(0,-1,0)*trotz(-pi/2);
dcha_leg.plot(qr)
teach(dcha_leg,qr)

%%

shoulder_rot = Revolute('a',0,'alpha',-pi/2,'qlim',[-pi/2 pi/2],'offset',0); % DH parameters for 0T1
shoulder_flx = Revolute('a',0,'alpha',pi/2,'qlim',[0 ,pi],'offset',pi/2);
shoulder_ad = Revolute('a',2,'alpha',pi/2,'qlim',[-pi 0],'offset',0);

elbow_pron  = Revolute('a',0,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);
elbow_flex  = Revolute('a',2,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);

wrist_flex  = Revolute('a',0,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);
wrist_abd   = Revolute('a',1,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);



izq_arm=SerialLink([shoulder_rot,shoulder_flx,shoulder_ad,elbow_pron,elbow_flex,wrist_flex,wrist_abd],'name', 'izqda_arm');
qr=[0,0,0,0,0,0,0];
izq_arm.base=transl(0,1.5,5)*trotz(pi/2);
izq_arm.plot(qr)
teach(izq_arm,qr)

%%
shoulder_rot = Revolute('a',0,'alpha',-pi/2,'qlim',[-pi/2 pi/2],'offset',0); % DH parameters for 0T1
shoulder_flx = Revolute('a',0,'alpha',pi/2,'qlim',[0 ,pi],'offset',pi/2);
shoulder_ad = Revolute('a',2,'alpha',pi/2,'qlim',[-pi 0],'offset',0);

elbow_pron  = Revolute('a',0,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);
elbow_flex  = Revolute('a',2,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);

wrist_flex  = Revolute('a',0,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);
wrist_abd   = Revolute('a',1,'alpha',pi/2,'offset',0,'qlim',[-pi/2 pi/2]);


dcha_arm=SerialLink([shoulder_rot,shoulder_flx,shoulder_ad,elbow_pron,elbow_flex,wrist_flex,wrist_abd],'name', 'dcha_arm');
qr=[0,0,0,0,0,0,0];
dcha_arm.base=transl(0,-1.5,5)*trotz(pi/2);
dcha_arm.plot(qr)
teach(dcha_arm,qr)

