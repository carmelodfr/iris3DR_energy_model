% An accurate measurement-driven Energy model for an IRIS 3DR quadrotor.
% Copyright (c) 2019 
% Author: Carmelo Di Franco 
% Email: carmelodfr@gmail.com
% This code is licensed under MIT license (see LICENSE.txt for details)
%% 
function [toal_energy, total_time, total_distance] = ...
    predict_energy_mission(E_model,waypoints, altitude,max_speed, stop_at_every_waypoint)
    

    toal_energy = 0;
    total_time = 0;
    total_distance = 0;
 
    
    % number of waypoints
    numwp = size(waypoints, 1);
    
    
    % compute the angles between waypoints. First angle is 0.
    angles = zeros(numwp,1);
    for i = 2 : numwp -1
        angles(i) = angle_between_3_points(waypoints(i-1,:), ...
                                             waypoints(i,:), ...
                                             waypoints(i+1,:));             
    end    
     % computing the last angle to Return to Launch (RTL)
    angles(end) = angle_between_3_points(waypoints(end-1,:), ...
                                             waypoints(end,:), ...
                                             waypoints(1,:));

                                         
                                         
    vin = 0;

    for i = 1 : numwp -1
        
        % ene   rgy consumpion in straight distances
        distance = norm(waypoints(i,:) - waypoints(i+1,:));
        total_distance = total_distance + distance;
        
        % rounding the angle
        angle = floor(angles(i)); 
        % if old approach
        if stop_at_every_waypoint 
            % angle = 0 means that we stop : v_out will be 0
            angle = 0;
        end
 
        
        % energy consumption during straight distance
        [e_tot, t_tot, ~, v_out] = ...
            predict_energy_and_v_opt(distance,vin, angle, max_speed, E_model);
        
        
        
%       If you want to rotate and predict the energy and time:
%       t_partial    = (angle*pi/180)/v_rotate;
%       energy_total = energy_total + E_rotate*t_partial;
%       time_total   = time_total + t_partial;

        toal_energy = toal_energy + e_tot;
        total_time = total_time + t_tot;
        vin = v_out;
            
    end
    
    
    % compute enery and time to ascend
    [e_climb, t_climb] = E_climb(E_model, 0, altitude);
    % descend (15, 0)
    [e_descend, t_descend] = E_climb(E_model, altitude, 0);
  
    % go to starting point
    distance = norm(waypoints(end,:) - waypoints(1,:));
    
    angle = 180;                                     
    if distance > 0
        [e_rtl, t_rtl, ~, ~] = ...
            predict_energy_and_v_opt(distance,vin, angle, max_speed, E_model);

    else
        e_rtl = 0;
        t_rtl = 0;
    end

    toal_energy     = toal_energy    + e_climb + e_rtl + e_descend;
    total_time      = total_time     + t_climb + t_rtl + t_descend;
    total_distance  = total_distance + distance;
end  