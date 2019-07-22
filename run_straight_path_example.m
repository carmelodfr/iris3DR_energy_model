% An accurate measurement-driven Energy model for an IRIS 3DR quadrotor.
% Copyright (c) 2019 
% Author: Carmelo Di Franco 
% Email: carmelodfr@gmail.com
% This code is licensed under MIT license (see LICENSE.txt for details)
%%

clc
clear
close all

% load the original energy model (2016)
load('energy_model/E_model.mat')
% the improvement presented at IROS2018 is inside the struct E_model:
% E_model.E_angle_speed , computed by Taua Cabreira and Carmelo Di Franco


% WARNING: This function makes the assumptions that vin , vout, speed are
% reasonable to respect to the distance to travel. It has not been tested
% for all the possible cases. For example, do not try distance = 1 meter,
% speed = 10 m/s and, v_in = 0 , v_out = 0. It will be impossible to travel
% 1 meter in that time.
% It is possible to compute the maximum speed according to the distance but
% it is out of scope of this code.
vin = 0;
vout = 3;
speed = 6;
distance = 100;
[ total_energy,total_time] = predict_energy_straight_path(distance, vin, vout, speed,E_model);



   
   
% print results
fprintf('Total Energy is %f J \n', total_energy);
fprintf('Total time is %f s \n',total_time);
fprintf('Total distance is %f m \n',distance);





