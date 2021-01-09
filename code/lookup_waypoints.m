function [waypoints, waypoint_times] = lookup_waypoints(question,x_state,y_state,z_state)
%
% Input parameters
%
%   question: which question of the project we are on 
%      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
%
% Output parameters
%
%   waypoints: of the form [x; y; z; yaw]
% 
%   waypoint_times: [1 x n] vector of times where n is the number of waypoings, 
%   represents the seconds you should be at each respective waypoint
%
%************ LOOKUP WAYPOINTS ************************

% Write code here

%Sample waypoints for hover trajectory
if question == 2
    waypoints = [0 0.1 0.2 0.3; 0 0 0 0; 0.5 0.5 0.5 0.5; 0 0 0 0].*4;
    waypoint_times = [0 2 4 6];

elseif question == 3
    iter = 0.1;
    waypoint_times_1 = [0:iter:3];
    waypoint_times_2 = [3+iter:iter:5];
    waypoint_times_3 = [5+iter:iter:8];
    waypoints_1 = (1/9).*waypoint_times_1.*waypoint_times_1;
    waypoints_2 = ones(1,length(waypoint_times_2));
    waypoints_3 = (-1/39).*waypoint_times_3.*waypoint_times_3+(64/39);
    
    waypoint_times = [waypoint_times_1 waypoint_times_2 waypoint_times_3];
    waypoints_z = [waypoints_1 waypoints_2 waypoints_3];
    zero_waypoints = zeros(1,length(waypoint_times));
    waypoints = [zero_waypoints ; zero_waypoints ; waypoints_z ; zero_waypoints];
    
elseif question == 4 %question 4 tracking case
    waypoints = [0 0 0 0; 0 0 0 0; 0.1 0.1 0.1 0.1; 0.262 0.262 0.262 0.262];
    waypoint_times = [0 2 4 6];
    
elseif question == 0 %takeoff case
    iter = 0.1;
    waypoint_times = [0:iter:3];
    zero_waypoints = zeros(1,length(waypoint_times));
    waypoints = [zero_waypoints ; zero_waypoints ; (1/9).*waypoint_times.*waypoint_times ; zero_waypoints];

elseif question == 9 %hover case
    iter = 0.5;
    waypoint_times = [0:iter:2];
    zero_waypoints = zeros(1,length(waypoint_times));
    waypoints = [ones(1,length(waypoint_times)).*x_state ; ones(1,length(waypoint_times)).*y_state ; ones(1,length(waypoint_times)).*z_state ; zero_waypoints];
   
elseif question == 8 %landing case
    iter = 0.1;
    waypoint_times = [0:iter:3];
    zero_waypoints = zeros(1,length(waypoint_times));
    waypoints = [ones(1,length(waypoint_times)).*x_state ; ones(1,length(waypoint_times)).*y_state ; linspace(z_state,0,length(zero_waypoints)) ; zero_waypoints];
    
end

end
