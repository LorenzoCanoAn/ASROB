% ----------------------------------------------------------------
% Lorenzo Cano
% Carlos Tierno
% ----------------------------------------------------------------

%% Exercise 1: 
% Obtain the homogeneous matrices that transform a reference system
% according to the following basic movements (all expressed in the original
% reference system).
fprintf("----------------------------------------------------\n");
disp("EXERCISE 1\n");
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
fprintf("Comparison between transformations: \n");
answer =   strcat(...
            "\t The first interesting thing to observe is that the rotation\n",...
            "\t matrix of both transformations stays the same. This makes\n",...
            "\t complete sense, as the fact that there is a translation prior\n",...
            "\t to it does not affect the new orientation of the reference\n",...
            "\t frame. On the other hand, the translation vector of the\n",...
            "\t transformation matrices does change between them.\n",...
            "\t When the translation is applied first, the translation vector\n",...
            "\t is the same as the translation applied to the frame, this is\n",...
            "\t because the axis of both frames are aligned. However, in the\n",...
            "\t case where the rotation is applied first, the X axis of the\n",...
            "\t tranformed frame aligns with the Z axis of the base frame in the\n",...
            "\t negative direction, and so, the translation occurs in the -Z direction\n");
fprintf(answer);

%% Exercise 2:
% Compute and compare the following two transformations. Compare the
% results as well with the results of 1):
fprintf("----------------------------------------------------\n");
fprintf("\nEXERCISE 2\n")

    % Rotation of 90º on the Y axis, followed by a translation of 10 on the X axis of
    % the original reference system
    
T = troty(pi/2);
T = transl(10,0,0) * T; % To apply a tranformation wrt. the original ref.s.
                        % is equivalent to pre-multiplying by the
                        % tranformation.
msg = "90º rotation along Y followed by tranlsation of 10 along original X:\n";
disp(msg);disp(T);
    
    % Translation of 10 on the X axis, followed by a 90º rotation on the new Y axis.
    
T =  transl(10,0,0) * troty(pi/2);
msg = "Tranlsation of 10 along X followed by 90º rotation along Y :\n";
disp(msg);disp(T);

    
    % Interpret the result
fprintf("Comparison between transformations: \n");
answer =   strcat(...
            "\t The only thing of intrest here is that, when applying",...
            "\t a transformation wrt the original reference system, it",...
            "\t is pre-multiplied. In this case, it is equivalent to first",...
            "\t translate the reference along X, and then rotating.\n");
fprintf(answer);
    
%% Exercise 3:
fprintf("----------------------------------------------------\n");
fprintf("\n EXERCISE 3\n");
T_c_o = [   0,  1,  0,  1;  ...
            1,  0,  0,  10; ...
            0,  0,  -1, 0;  ...
            0,  0,  0,  1;  ];
        
T_c_b = [   1,  0,  0,  1;  ...
            0,  -1, 0,  20; ...
            0,  0,  -1, 10; ...
            0,  0,  0,  1   ];
    % a) Compute the location of the object relative to the robot base, BTO
T_b_c = eye(size(T_c_b))/T_c_b;
T_b_o =  T_b_c * T_c_o;
fprintf("\ta) Location of object wrt the base of the robot: \n");
disp(T_b_o);
    
    % b) After the work station setup, someone rotated the camera by 180º on the X
    % axis of its own reference frame. After that, they translated the camera
    % relative to the new frame axis, by 11 units following the direction of the
    % new X axis, and -10 units on the new Y axis. Compute the new location of
    % the camera relative to the robot base.
    
T_b_c = eye(size(T_c_b))/T_c_b;
T = trotx(pi) * transl(11,-10,0);   % Compute the tranformation
T_b_c = T_b_c * T;                  % Apply the tranformation
T_c_b = eye(size(T_b_c))/T_b_c;                 % Update the inverse
fprintf(strcat(...
            "\tb) Camera frame wrt to robot base after 180º rotation along X,\n",...
            "\ttranlation of 11 units along new X and -10 units along new Y\n"));
disp(T_b_c);

    % c) After performing these modifications, the object has been rotated 90º on its
    % Z axis, and it has been translated 10 units on the X axis of the robot base.
    % Compute the new location of the object relative to the camera frame.
T_o_c = eye(size(T_c_b))/T_c_b;
T_o_c = T_o_c * T; % Update the relation between the camera and the object
T_c_o = eye(size(T_o_c))/T_o_c;
T_c_o = T_c_o*trotz(pi/2); % Apply rotation to object
T_b_o = T_b_c * T_c_o;     % The translation wrt the robot base requires T_b_o
T_b_o = transl(10,0,0) * T_b_o; % Tranlation applied
T_c_o = T_c_b * T_b_o;

fprintf(strcat(...
    "\tc) Object frame wrt the camera frame after rotating it by 180º along\n",...
    "\tits Z axis and moving it 10 units along the robot's base X axis\n"));
disp(T_c_o);






    
%% Exercise 4:
fprintf("----------------------------------------------------\n");
fprintf("EXERCISE 4\n");


    % a) Three Euler angles (45º, 0, 15º). Compare it against the rotation matrix
    % associated with Euler angles (60º, 0, 0). What is happening? How do
    % you explain this?
    
    R_1 = rotx(45,'angle') * rotz(15,'angle');
    
    fprintf("\n\nR(45,0,15)\n");
    disp(R_1);
    
    
    R_2 = rotx(60,'angle');
    
    fprintf("\n\nR(60,0,0)\n");
    disp(R_2);
    
    fprintf("Hay que escribir algo aqui pero no tengo ni idea de que quiere\n");
    
    % b) Compute the RPY angles associated with the previous cases. 
    
    rpy_1 = tr2rpy(R_1);
    fprintf("\n\nRPY R(45,0,15)\n");
    disp(rpy_1);
    
    rpy_2 = tr2rpy(R_2);
    fprintf("\n\nRPY R(60,0,0)\n");
    disp(rpy_2)
    

%% Exercise 5:
fprintf("----------------------------------------------------\n");
fprintf("EXERCISE 5\n");

    % Obtain the rotation matrix associated with:
    T_c_o = [   0,  1,  0,  1;  ...
                1,  0,  0,  10; ...
                0,  0,  -1, 0;  ...
                0,  0,  0,  1;  ];

    T_c_b = [   1,  0,  0,  1;  ...
                0,  -1, 0,  20; ...
                0,  0,  -1, 10; ...
                0,  0,  0,  1   ];
        % a) Compute the location of the object relative to the robot base, BTO
    T_b_c = eye(size(T_c_b))/T_c_b;
    T_b_o =  T_b_c * T_c_o;


% The center of the robot's gripper (G) is initially located at the same height
% from the ground as the base reference (B), but with an orientation of 45º
% measured on the X axis of that reference, and at a distance measured
% horizontally of 10 units. The plane in which the origin of B and G lie is
% perpendicular to the X axis of the camera (see figure):
    % a) Compute the homogeneous transformation that represents the location
    % of the gripper relative to the base. Obtain the location vector BxG=(x,y,z,
    % φ,θ,ψ), expressing the orientation in Euler angles.
    
    T_b_g = transl(0,10,0) * trotx(pi/4);
    xBxG = T_b_g*[0 0 0 1]';
    x_eul_b_g = [xBxG(1:3)' tr2rpy(T_b_g)];
    
    fprintf("The location vector of G refernced to B (x,y,z,fi,theta,psi):\n");
    disp(x_eul_b_g)
    
    
    % b) What movement must the robot's gripper make (translation and rotation)
    % to pick up the object O that is on the table? Express it as a sequence of
    % translations and rotations, as a homogeneous matrix, and as a location
    % vector with RPY angles.

    T_g_o = T_b_g \ T_b_o;
    
    fprintf("Gripper movement as homogeneous matrix: \n");
    disp(T_g_o);
    
    fprintf("As a sequence of translation and rotations:\n");
    fprintf("Translation:\n");
    t_g_o = T_g_o*[0 0 0 1]';
    
    disp(t_g_o(1:3));
    fprintf("Rotation:");
    r_g_o = tr2eul(T_g_o);
    disp(r_g_o);
    
    
    
    
    % c) What movement (translation and rotation) will observe the camera?
    tr2eul(T_g_o)
    T_g_o*[0 0 0 1]'
    

