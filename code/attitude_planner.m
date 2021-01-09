function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
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
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here

%define gravity term
g = params.gravity;
%define psi_desired
psi_d = desired_state.rot(3);

%define A matrix
pos_mat = (1/g)*[sin(psi_d) -cos(psi_d) ; cos(psi_d) sin(psi_d)];
%define reference vect
%TODO: use the desired x_dotdot and y_dotdot to get the roll pitch?
vect_x_elem = desired_state.acc(1);
vect_y_elem = desired_state.acc(2);

%compute phi and theta desired values
rollpitch_des = pos_mat*[vect_x_elem ; vect_y_elem];
phi_d = rollpitch_des(1);
theta_d = rollpitch_des(2);

%define deriv vectors
vect_xdot_elem = desired_state.acc(1)*desired_state.omega(3);
vect_ydot_elem = desired_state.acc(2)*desired_state.omega(3);

%compute phi_dot and theta_dot desired values
%TODO: ignore triple derivatives
vel_mat = (1/g)*[cos(psi_d) sin(psi_d) ; -sin(psi_d) cos(psi_d)];
rollpitchdot_d = vel_mat * [vect_xdot_elem ; vect_ydot_elem];
phidot_d = rollpitchdot_d(1);
thetadot_d = rollpitchdot_d(2);

%assign function outputs to values
rot = [phi_d ; theta_d ; desired_state.rot(3)];
omega = [phidot_d ; thetadot_d ; desired_state.omega(3)];
    
    



end

