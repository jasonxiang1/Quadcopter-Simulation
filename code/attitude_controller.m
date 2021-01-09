function [M] = attitude_controller(state,desired_state,params,question)

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
%   M: u2 or moment [M1; M2; M3]
%
%************  ATTITUDE CONTROLLER ************************

% Example PD gains
Kpphi = 190;
Kdphi = 30;

Kptheta = 198;
Kdtheta = 30;

Kppsi = 80;
Kdpsi = 17.88;

% Write code here


if question == 2

      %question 2 updated gains
%     Kpphi = 210;
%     Kdphi = 28;
% 
%     Kptheta = 218;
%     Kdtheta = 28;
% 
%     Kppsi = 80;
%     Kdpsi = 15.88;

%     %Gain Set 1
%     Kpphi = 190;
%     Kdphi = 30;
% 
%     Kptheta = 190;
%     Kdtheta = 30;
% 
%     Kppsi = 70;
%     Kdpsi = 18;

    %Gain Set 2
    Kpphi = 190;
    Kdphi = 30;

    Kptheta = 190;
    Kdtheta = 30;

    Kppsi = 20;
    Kdpsi = 18;

    %create gain matrix
    Kp = [Kpphi 0 0 ; 0 Kptheta 0 ; 0 0 Kppsi];
    Kd = [Kdphi 0 0 ; 0 Kdtheta 0 ; 0 0 Kdpsi];
    
    %define intertia matrix
    I_mat = params.inertia;
    
    %define error vectors
    pos_err = [state.rot(1) - desired_state.rot(1) ; state.rot(2) - desired_state.rot(2) ; state.rot(3) - desired_state.rot(3)];
    vel_err = [state.omega(1) - desired_state.omega(1) ; state.omega(2) - desired_state.omega(2) ; state.omega(3) - desired_state.omega(3)];
    
    %define feed forward term
    uff = [desired_state.acc(1) ; desired_state.acc(2) ; desired_state.acc(3)];
    
    %compute moments
    mom_mat = I_mat*(-Kp*pos_err - Kd*vel_err + uff);
    
    %assign values to return variables of function
    M = mom_mat;
    
elseif question == 3
    %create gain matrix
    Kp = [Kpphi 0 0 ; 0 Kptheta 0 ; 0 0 Kppsi];
    Kd = [Kdphi 0 0 ; 0 Kdtheta 0 ; 0 0 Kdpsi];
    
    %define intertia matrix
    I_mat = params.inertia;
    
    %define error vectors
    pos_err = [state.rot(1) - desired_state.rot(1) ; state.rot(2) - desired_state.rot(2) ; state.rot(3) - desired_state.rot(3)];
    vel_err = [state.omega(1) - desired_state.omega(1) ; state.omega(2) - desired_state.omega(2) ; state.omega(3) - desired_state.omega(3)];
    
    %define feed forward term
    uff = [desired_state.acc(1) ; desired_state.acc(2) ; desired_state.acc(3)];
    
    %compute moments
    mom_mat = I_mat*(-Kp*pos_err - Kd*vel_err + uff);
    
    %assign values to return variables of function
    M = mom_mat;
    
elseif question == 0 %attitude controller for takeoff and landing
    %create gain matrix
    Kp = [Kpphi 0 0 ; 0 Kptheta 0 ; 0 0 Kppsi];
    Kd = [Kdphi 0 0 ; 0 Kdtheta 0 ; 0 0 Kdpsi];
    
    %define intertia matrix
    I_mat = params.inertia;
    
    %define error vectors
    pos_err = [state.rot(1) - desired_state.rot(1) ; state.rot(2) - desired_state.rot(2) ; state.rot(3) - desired_state.rot(3)];
    vel_err = [state.omega(1) - desired_state.omega(1) ; state.omega(2) - desired_state.omega(2) ; state.omega(3) - desired_state.omega(3)];
    
    %define feed forward term
    uff = [desired_state.acc(1) ; desired_state.acc(2) ; desired_state.acc(3)];
    
    %compute moments
    mom_mat = I_mat*(-Kp*pos_err - Kd*vel_err + uff);
    
    %assign values to return variables of function
    M = mom_mat;    

end

end

