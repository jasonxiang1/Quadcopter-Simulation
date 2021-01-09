function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************

% Write code here
% compute state_dot for all 12 values
% break down 12-long state_dot vector into four sections
% xyz_dot, xyz_dotdot, rot_dot, omega_dot

% compute xyz_dot vector
xyz_dot = [state(4) ; state(5) ; state(6)];

%compute xyz_dotdot vector
xyz_dotdot_1 = F*(cos(state(7))*cos(state(9))*sin(state(8)) + sin(state(8))*sin(state(9)));
xyz_dotdot_2 = F*(cos(state(7))*sin(state(8))*sin(state(9)) - cos(state(9))*sin(state(7)));
xyz_dotdot_3 = F*cos(state(8))*cos(state(7)) - params.mass*params.gravity;
xyz_dotdot = [xyz_dotdot_1 ; xyz_dotdot_2 ; xyz_dotdot_3];

%compute rot_dot vector
rot_dot = [state(10) ; state(11) ; state(12)];

%compute omega_dot vector
%assumption that w about equals omega_dotdot

%omega = [state(10) ; state(11) ; state(12)];

omega = [cos(state(8)) 0 -cos(state(7))*sin(state(8)) ; 0 1 sin(state(7)) ; sin(state(7)) 0 cos(state(7))*cos(state(8))]*[state(10) ; state(11) ; state(12)];

omega_dot = params.inertia\(M-cross(omega,params.inertia*omega));
%omega_dot = inv(params.inertia)*M;


%compute rpm_dot vector
rpm_dot = rpm_motor_dot;

%compute state vector
state_dot = [xyz_dot ; xyz_dotdot ; rot_dot ; omega_dot ; rpm_dot];

end 