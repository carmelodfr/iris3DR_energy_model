function [ e_tot,t_tot] = predict_energy_straight_path(distance, vin, vout, speed,E_model)
%predict_energy_and_opt_dist predicts the energy required to travel a
% given distance. The function also computes the optimal speed
%   
% WARNING: This function makes the assumptions that vin , vout, speed are
% reasonable to respect to the distance to travel. It has not been tested
% for all the possible cases. For example, do not try distance = 1 meter,
% speed = 10 m/s and, v_in = 0 , v_out = 0. It will be impossible to travel
% 1 meter in that time.
%
%   Input:
%   distance  : Distance to travel
%   vin       : Initial speed of the quadrotor
%   vout      : Final speed of the quadrotor
%   speed     : Desired speed
%   E_model   : Energy model functions and parameters
%
%   Output:
%   e_tot     : predicted energy
%   t_tot     : predicted time
%
% Copyright (c) 2019 
% Author: Carmelo Di Franco 
% Email: carmelodfr@gmail.com
% This code is licensed under MIT license (see LICENSE.txt for details)
 
%%

% WARNING: This function makes the assumptions that vin , vout, speed are
% reasonable to respect to the distance to travel. It has not been tested
% for all the possible cases. For example, do not try distance = 1 meter,
% speed = 10 m/s and, v_in = 0 , v_out = 0. It will be impossible to travel
% 1 meter in that time.



% these are the functions from the energy model
% speed as a function of the time during the acceleration
func_v_acc = poly2sym(E_model.a_coefficents);
func_v_dec = poly2sym(E_model.dec_coefficents);

t_dec_max  = min(clean_values(solve(func_v_dec == 0),inf)); %get the max time to decelerate


syms x vout_x;

% these two function are the ones shown in the paper. however they can be
% approximated to constant acceleration.
% func_v_acc = func_v_acc - subs(func_v_acc,0); % shift to zezo
% func_v_dec = subs(func_v_dec,-x + t_dec_max);
% inestead we consider constant acceleration (px4 acceleration is 1m/s^2)
func_v_acc = poly2sym([0 1 0]);
func_v_dec = poly2sym([0 1 0]);



x = vout;



% compute integrals of the functions to get distance functions
func_d_acc = int(func_v_acc);  % x is time
func_d_dec = int(func_v_dec);  % x is time

t_vin = min(clean_values(solve(func_v_acc == vin ),inf));
 


v_bounded = speed;


t_dec  = min(clean_values(solve(func_v_dec == v_bounded),inf));
t_dec_vout          = min(clean_values(solve(func_v_dec ==v_bounded),inf));
dist_traveled_dec   = double(subs(func_d_dec,t_dec) - subs(func_d_acc,t_dec_vout));
dist_traveled_acc   = double(subs(func_d_acc,t_dec) - subs(func_d_acc,t_vin));
 


distance_v_const = distance - dist_traveled_acc - dist_traveled_dec;

            
            


pv_acc_coeff = E_model.pv_acc_coeff;
pv_dec_coeff = E_model.pv_dec_coeff;



pva = poly2sym(pv_acc_coeff);
pvd = poly2sym(pv_dec_coeff);
% the function goes from maximum speed to zero. We need to flip it to make it consistent
pvd = subs(pvd,15 - x); 


P_speed       = poly2sym(E_model.v_coefficents);

P_v_const = double(subs(P_speed ,v_bounded));

E_dec = int(pvd);
E_acc = int(pva);
 
 
E_accVin = subs(E_acc,vin);
E_decVout = subs(E_dec,vout);
E_accV = subs(E_acc,v_bounded);
E_decV = subs(E_dec,v_bounded);

E_to_accelerate = E_accV - E_accVin;
E_to_decelerate = E_decV - E_decVout;



if (distance_v_const >0)
    t_const_speed = distance_v_const/v_bounded;
    E_const_speed = P_v_const*t_const_speed;
    
end


e_tot = double(E_to_accelerate + E_const_speed + E_to_decelerate);



t_vout = min(clean_values(solve(func_v_dec == vout),inf));
t_acc_init = t_vin;
t_acc_final  = min(clean_values(solve(func_v_acc == v_bounded),inf));
t_dec  = min(clean_values(solve(func_v_dec == v_bounded),inf));

t_tot = (t_acc_final- t_acc_init) + t_const_speed + (t_dec - t_vout);


figure()
plot([0 (t_acc_final- t_acc_init)],[0 v_bounded],'b-')
hold on
plot([(t_acc_final- t_acc_init) (t_acc_final- t_acc_init+t_const_speed)],[v_bounded v_bounded],'b-');
plot([(t_acc_final- t_acc_init+t_const_speed) (t_acc_final- t_acc_init+t_const_speed +t_dec - t_vout)],[v_bounded vout],'b-');
hold off
xlabel('time [s]')
ylabel('speed [m/s]')




end

