function [F, acc] = position_controller(current_state, desired_state, params, question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   F: u1 or thrust
%
%   acc: will be stored as desired_state.acc = [xdotdot; ydotdot; zdotdot]
%
%************  POSITION CONTROLLER ************************

% Example PD gains
Kp1 = 17;
Kd1 = 6.6;

Kp2 = 17;
Kd2 = 6.6;

Kp3 = 20;
Kd3 = 9;

% Write code here

%if question==2
if question==2

%     Kp1 = 12;
%     Kd1 = 6.6;
% 
%     Kp2 = 12;
%     Kd2 = 6.6;
% 
%     Kp3 = 14;
%     Kd3 = 9;

%     %Gain Set 1
%     Kp1 = 20;
%     Kd1 = 8;
% 
%     Kp2 = 20;
%     Kd2 = 8;
% 
%     Kp3 = 18;
%     Kd3 = 9;    

    %Gain Set 2
    Kp1 = 20;
    Kd1 = 8;

    Kp2 = 20;
    Kd2 = 8;

    Kp3 = 10;
    Kd3 = 19;   
    
    %compute err_dotdot_xyz
    %compute position errors
    err_x_pos = current_state.pos(1)-desired_state.pos(1);
    err_y_pos = current_state.pos(2)-desired_state.pos(2);
    err_z_pos = current_state.pos(3)-desired_state.pos(3);
    %compute velocity errors
    err_x_vel = current_state.vel(1) - desired_state.vel(1);
    err_y_vel = current_state.vel(2) - desired_state.vel(2);
    err_z_vel = current_state.vel(3) - desired_state.vel(3);
    %compute feedforward uff term
    uff = [desired_state.acc(1) ; desired_state.acc(2) ; desired_state.acc(3)];
    %create gain matrix
    Kp = [Kp1 0 0 ; 0 Kp2 0 ; 0 0 Kp3];
    Kd = [Kd1 0 0 ; 0 Kd2 0 ; 0 0 Kd3];
    
    err_dotdot_xyz = -Kp*[err_x_pos ; err_y_pos ; err_z_pos] - Kd*[err_x_vel ; err_y_vel ; err_z_vel];
    
    %determine b3 vector
    b3 = [0 ; 0 ; 1];
    
    %determine gravity which is pointing down
    del_phi = current_state.rot(1) - desired_state.rot(1);
    del_theta = current_state.rot(2) - desired_state.rot(2);
    del_psi = current_state.rot(3) - desired_state.rot(3);
    R = [1 0 0 ; 0 cos(del_phi) -sin(del_phi) ; 0 sin(del_phi) cos(del_phi)]*[cos(del_theta) 0 sin(del_theta) ; 0 1 0 ; -sin(del_theta) 0 cos(del_theta)];
    g_vect = R*[0 ; 0 ; params.gravity];
    
    %compute thrust f
    F = params.mass*transpose(b3)*(g_vect + err_dotdot_xyz + uff);
    
    
    %TODO: pass along the current desired state????
    acc = err_dotdot_xyz + [desired_state.acc];
    
elseif question == 3 

    Kp3 = 80;
    Kd3 = 9;
    
    %compute err_dotdot_xyz
    %compute position errors
    err_x_pos = current_state.pos(1)-desired_state.pos(1);
    err_y_pos = current_state.pos(2)-desired_state.pos(2);
    err_z_pos = current_state.pos(3)-desired_state.pos(3);
    %compute velocity errors
    err_x_vel = current_state.vel(1) - desired_state.vel(1);
    err_y_vel = current_state.vel(2) - desired_state.vel(2);
    err_z_vel = current_state.vel(3) - desired_state.vel(3);
    %compute feedforward uff term
    uff = [desired_state.acc(1) ; desired_state.acc(2) ; desired_state.acc(3)];
    %create gain matrix
    Kp = [Kp1 0 0 ; 0 Kp2 0 ; 0 0 Kp3];
    Kd = [Kd1 0 0 ; 0 Kd2 0 ; 0 0 Kd3];
    
    err_dotdot_xyz = -Kp*[err_x_pos ; err_y_pos ; err_z_pos] - Kd*[err_x_vel ; err_y_vel ; err_z_vel];
    
    %determine b3 vector
    b3 = [0 ; 0 ; 1];
    
    %determine gravity which is pointing down
    del_phi = current_state.rot(1) - desired_state.rot(1);
    del_theta = current_state.rot(2) - desired_state.rot(2);
    del_psi = current_state.rot(3) - desired_state.rot(3);
    R = [1 0 0 ; 0 cos(del_phi) -sin(del_phi) ; 0 sin(del_phi) cos(del_phi)]*[cos(del_theta) 0 sin(del_theta) ; 0 1 0 ; -sin(del_theta) 0 cos(del_theta)];
    g_vect = R*[0 ; 0 ; params.gravity];
    
    %compute thrust f
    F = params.mass*transpose(b3)*(g_vect + err_dotdot_xyz + uff);
    
    acc = err_dotdot_xyz + [desired_state.acc];
elseif question == 0 %positional controller for takeoff and landing
    Kp3 = 80;
    Kd3 = 9;
    
    %compute err_dotdot_xyz
    %compute position errors
    err_x_pos = current_state.pos(1)-desired_state.pos(1);
    err_y_pos = current_state.pos(2)-desired_state.pos(2);
    err_z_pos = current_state.pos(3)-desired_state.pos(3);
    %compute velocity errors
    err_x_vel = current_state.vel(1) - desired_state.vel(1);
    err_y_vel = current_state.vel(2) - desired_state.vel(2);
    err_z_vel = current_state.vel(3) - desired_state.vel(3);
    %compute feedforward uff term
    uff = [desired_state.acc(1) ; desired_state.acc(2) ; desired_state.acc(3)];
    %create gain matrix
    Kp = [Kp1 0 0 ; 0 Kp2 0 ; 0 0 Kp3];
    Kd = [Kd1 0 0 ; 0 Kd2 0 ; 0 0 Kd3];
    
    err_dotdot_xyz = -Kp*[err_x_pos ; err_y_pos ; err_z_pos] - Kd*[err_x_vel ; err_y_vel ; err_z_vel];
    
    %determine b3 vector
    b3 = [0 ; 0 ; 1];
    
    %determine gravity which is pointing down
    del_phi = current_state.rot(1) - desired_state.rot(1);
    del_theta = current_state.rot(2) - desired_state.rot(2);
    del_psi = current_state.rot(3) - desired_state.rot(3);
    R = [1 0 0 ; 0 cos(del_phi) -sin(del_phi) ; 0 sin(del_phi) cos(del_phi)]*[cos(del_theta) 0 sin(del_theta) ; 0 1 0 ; -sin(del_theta) 0 cos(del_theta)];
    g_vect = R*[0 ; 0 ; params.gravity];
    
    %compute thrust f
    F = params.mass*transpose(b3)*(g_vect + err_dotdot_xyz + uff);
    
    
    %TODO: pass along the current desired state????
    acc = err_dotdot_xyz + [desired_state.acc];
    
    
    
end



end
