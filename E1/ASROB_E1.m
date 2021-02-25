% ----------------------------------------------------------------
% Lorenzo Cano
% Carlos Tierno
% ----------------------------------------------------------------

%% Exercise 1: 
% Obtain the homogeneous matrices that transform a reference system
% according to the following basic movements (all expressed in the original
% reference system).

    % Rotation of 90º on the Y axis, followed by a translation of 10 units on the X
    % axis.

T = troty(pi/2) * transl(10,0,0);
msg = "90º rotation along Y followed by tranlsation of 10 along X:\n";
disp(msg);disp(T);

    % Translation of 10 on the X axis, followed by a 90º rotation on the Y axis

T =  transl(10,0,0) * troty(pi/2);
msg = "Tranlsation of 10 along X followed by 90º rotation along Y :\n";
disp(msg);disp(T);

    % Compare both transformations. Interpret the differences considering both the
    % translation and the rotation parts.

%% Exercise 2:
% Compute and compare the following two transformations. Compare the
% results as well with the results of 1):

    % Rotation of 90º on the Y axis, followed by a translation of 10 on the X axis of
    % the original reference system

    
    % Translation of 10 on the X axis, followed by a 90º rotation on the new Y axis.

    
    % Interpret the result

    
%% Exercise 3:

    % a) Compute the location of the object relative to the robot base, BTO
    
    
    % b) After the work station setup, someone rotated the camera by 180º on the X
    % axis of its own reference frame. After that, they translated the camera
    % relative to the new frame axis, by 11 units following the direction of the
    % new X axis, and -10 units on the new Y axis. Compute the new location of
    % the camera relative to the robot base.
    
    
    % c) After performing these modifications, the object has been rotated 90º on its
    % Z axis, and it has been translated 10 units on the X axis of the robot base.
    % Compute the new location of the object relative to the camera frame.
    
%% Exercise 4:
% Obtain the rotation matrix associated with:

    % a) Three Euler angles (45º, 0, 15º). Compare it against the rotation matrix
    % associated with Euler angles (60º, 0, 0). What is happening? How do
    % you explain this?
    
    
    % b) Compute the RPY angles associated with the previous cases. 

%% Exercise 5:
% The center of the robot's gripper (G) is initially located at the same height
% from the ground as the base reference (B), but with an orientation of 45º
% measured on the X axis of that reference, and at a distance measured
% horizontally of 10 units. The plane in which the origin of B and G lie is
% perpendicular to the X axis of the camera (see figure):
    % a) Compute the homogeneous transformation that represents the location
    % of the gripper relative to the base. Obtain the location vector BxG=(x,y,z,
    % φ,θ,ψ), expressing the orientation in Euler angles.
    
    
    % b) What movement must the robot's gripper make (translation and rotation)
    % to pick up the object O that is on the table? Express it as a sequence of
    % translations and rotations, as a homogeneous matrix, and as a location
    % vector with RPY angles.
    
    % c) What movement (translation and rotation) will observe the camera?
    
%% Exercise 6:
% Represent the robot’s gripper (G) at the initial and final
% positions when the robot is going to pick up the object, as seen from the
% camera. Represent the object by using a 3D cube.