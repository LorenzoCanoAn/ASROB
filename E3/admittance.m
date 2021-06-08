% script: admittance.m
% Admittance Control for a leg 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  CONTROL PARAMETERS: SELECT THE PARAMETERS FRO YOUR EXERCISE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  EXTERNAL INPUTS
help_op=-1;     % helping (+1) force/ oppositng force (-1)

% WALKING AMPLITUDE
walk=1;       % walking amplitude

% ADMITTANCE
h_stiff=5;      % low admittance
l_stiff=2;      % high admittance

% ASSISTANCE
assis=0.5;      % total assistance [0..1]
emg_ratio=0;  % proportion [0..1] of EMG force (latent force) with respect to the pattern

%EXTERNAL FORCE
ext=0.2;       % external force/torque applied
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   LEG MODEL AS AN ARTICULATED MECHANISM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L1 = Revolute('a',0,'alpha',pi/2,'qlim',[0 pi/2]); % DH parameters for 0T1
L2 = Revolute('a',2,'alpha',0,'offset',300*pi/180);
L3 = Revolute('a',2,'alpha',0,'qlim',[pi 2*pi]);
L4 = Revolute('a',1,'alpha',0,'qlim',[0 pi],'offset',pi/2);
my_leg=SerialLink([L1,L2,L3,L4],'name', 'robot1');
qr=[0, 2];
my_leg.plot([0,0,0,0])
teach(my_leg,[0,0,0,0])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  SIMULINK ADMITTANCE CONTROL MODEL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
open('admittance_control.slx')
disp('INIT CONTROL: SIMULATE THE MOTION (Run in Simulink window)')
disp('WHEN CONTROL SIMULATION FINISH, PRESS ENTER')
pause
disp('MOTION STARTED')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  SIMULINK ADMITTANCE CONTROL MODEL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:(max(size(out.hip))-1)
    my_leg.animate([0,out.hip(i),out.knee(i),0]);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



