function [ e_tot,t_tot,v_opt,v_out] = predict_energy_and_v_opt(distance, vin, angle, max_speed,E_model)
%predict_energy_and_opt_dist predicts the energy required to travel a
% given distance. The function also computes the optimal speed
%   
%   Input:
%   distance  : Distance to travel
%   vin       : Initial speed of the quadrotor
%   angle     : the turning angle at the end of the straight path. it is
%               used to compute the final speed
%   max_speed : if you want to put a maximum speed instead of the optimal
%               speed
%   E_model   : Energy model functions and parameters
%
%   Output:
%   e_tot     : predicted energy
%   t_tot     : predicted time
%   v_opt     : Optimal speed that minimizes the energy
%   v_out     : final speed of the quadrotor. if angle == 0, v_out will be
%               zero
%
% Copyright (c) 2019 
% Author: Carmelo Di Franco 
% Email: carmelodfr@gmail.com
% This code is licensed under MIT license (see LICENSE.txt for details)
 
%%
DEBUG = 0;



% the table containing the speed reduction goes from 1 to 180. hence if
% angle = 0 , angle = 1 in order to get the first element of the array.
if (angle == 0)
    angle = 1;
end

E_angle_speed = E_model.E_angle_speed;

% get the percentage reduction of the speed.
vout_percentage = E_angle_speed(angle);


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


% we use the angle to compute what is the speed that we are going to reach
% at the end of the straight path
vout_x = x*vout_percentage;
 


% compute integrals of the functions to get distance functions
func_d_acc = int(func_v_acc);  % x is time
func_d_dec = int(func_v_dec);  % x is time

t_vin = min(clean_values(solve(func_v_acc == vin ),inf));
d0 = subs(func_d_acc,t_vin);

if DEBUG
    %----------------------------------------
    xx = 0:0.1:15;
    y_acc   = subs(func_v_acc,xx);
    y_dec   = xx(end) - subs(func_v_dec,xx);
    y_d_acc = subs(func_d_acc,xx);
    y_d_dec = subs(func_d_dec,xx(end)) - subs(func_d_dec,xx);

    figure(1)
    subplot(2,2,1)
    plot(xx,y_acc);
    hold on
    plot([t_vin t_vin],[0,vin],'r-');
    plot([0 t_vin],[vin,vin],'r-');
    xlabel('time');
    ylabel('speed (v) [m/s]');
    legend('speed when accelerating','time when it reaches vin');
    hold off

    subplot(2,2,2)
    plot(xx,y_dec);
    xlabel('time');
    ylabel('speed (v) [m/s]');
    legend('speed when decelerating');

    subplot(2,2,3)
    plot(xx,y_d_acc);
    hold on
    plot([t_vin t_vin],[0,d0],'r-');
    plot([0 t_vin],[d0,d0],'r-');
    xlabel('time');
    ylabel('distance (x) [m]');
    legend('distance traveled when accelerating','traveled distance from 0 to vin');
    hold off

    subplot(2,2,4)
    plot(xx,y_d_dec);
    xlabel('time');
    ylabel('distance (x) [m]');
    legend('distance traveled when decelerating');
    %----------------------------------------
end 



 

func_d_vconst = distance - ( func_d_acc - d0 ) - ...
                ( func_d_dec - subs(func_d_dec,vout_x) ) ;


if DEBUG         
aa = subs(func_v_acc,x+vin);
func_test_dec =  ( func_d_dec - subs(func_d_dec,vout_x) );
dd = distance - aa - func_test_dec
t_bound2 = min(clean_values(solve(dd == 0),inf));            
end

%at this time we only accelerate and decelerate
t_bound = min(clean_values(solve(func_d_vconst == 0),inf));
 
if DEBUG
    %----------------------------------------
    figure(2)
    xx = 0:0.1:10;
    yy_d = subs(func_d_vconst,xx);
    plot(xx,yy_d)
    hold on
    plot([0 t_bound ],[0 0 ],'r-');
    plot(t_bound,0,'rx');
    hold off

    xx = 0:0.1: t_bound +2;
    y_acc   = subs(func_v_acc,xx);
    y_dec   = xx(end) - subs(func_v_dec,xx);
    y_d_acc = subs(func_d_acc,xx);
    y_d_dec = subs(func_d_dec,xx(end)) - subs(func_d_dec,xx);

    figure(2)
    subplot(2,2,1)
    h1 = plot(xx,y_acc);
    hold on
    h2 = plot([t_vin t_vin],[0,vin],'r-');
    plot([0 t_vin],[vin,vin],'r-');
    v_bound = subs(func_v_acc,t_bound);
    h3 = plot([0, t_bound],[v_bound,v_bound],'g-');
    plot([t_bound, t_bound],[0,v_bound],'g-'); 
    xlabel('time');
    ylabel('speed (v) [m/s]');
    legend([h1 h2 h3],'distance traveled when accelerating','t_{vin}', 't_{bound} + t_{vin}');
    hold off

    subplot(2,2,2)
    plot(xx,y_dec);
    xlabel('time');
    ylabel('speed (v) [m/s]');
    legend('speed when decelerating');

    subplot(2,2,3)
    h1 = plot(xx,y_d_acc);
    hold on
    h2 = plot([t_vin t_vin],[0,d0],'r-');
    plot([0 t_vin],[d0,d0],'r-');

    d_bound = subs(func_d_acc, t_bound);

    h3 = plot([0, t_bound],[d_bound,d_bound],'g-');
    plot([t_bound, t_bound],[0,d_bound],'g-'); 

    xlabel('time');
    ylabel('distance (x) [m]');
    legend([h1 h2 h3],'distance traveled when accelerating','t_{vin}', 't_{bound} + t_{vin}');
    hold off

    subplot(2,2,4)
    plot(xx,y_d_dec);
    xlabel('time');
    ylabel('distance (x) [m]');
    legend('distance traveled when decelerating');
    %----------------------------------------
end


 
v_bound             = double(subs(func_v_acc,t_bound));
t_dec_vout          = min(clean_values(solve(func_v_dec ==v_bound*vout_percentage),inf));
dist_traveled_dec   = double(subs(func_d_dec,t_bound) - subs(func_d_acc,t_dec_vout));
dist_traveled_acc   = double(subs(func_d_acc,t_bound) - subs(func_d_acc,t_vin));
 
speed_bound         = min(max_speed, v_bound);


pv_acc_coeff = E_model.pv_acc_coeff;
pv_dec_coeff = E_model.pv_dec_coeff;



pva = poly2sym(pv_acc_coeff);
pvd = poly2sym(pv_dec_coeff);
% the function goes from maximum speed to zero. We need to flip it to make it consistent
pvd = subs(pvd,15 - x); 


P_speed       = poly2sym(E_model.v_coefficents);
 E_speed       = P_speed*func_d_vconst/x;

E_dec = int(poly2sym(E_model.p_dec_coefficents));
E_acc = int(poly2sym(E_model.p_a_coefficents));

E_dec = int(pvd);
E_acc = int(pva);
 
 
E_accVin = subs(E_acc,vin);
E_decVout = subs(E_dec,vout_x);
 

func_e_tot = (E_acc - E_accVin) + E_speed +  (E_dec - E_decVout) ;
 


v_opt =  min(clean_values(solve(diff(func_e_tot) == 0),speed_bound));

%if v_opt is null , should never happen
if(length(v_opt) == 0)
   v_opt = speed_bound;
end
e_tot = double(subs(func_e_tot,v_opt));



v_out = v_opt*vout_percentage; 

%t_dec_max = min(clean_values(solve(poly2sym(E_model.dec_coefficents) == 0),100));


t_vout = min(clean_values(solve(func_v_dec == v_out),inf));
t_acc_init = t_vin;
t_acc_final  = min(clean_values(solve(func_v_acc == v_opt),inf));
t_dec  = min(clean_values(solve(func_v_dec == v_opt),inf));

e_acc =  subs(E_acc,v_opt) - subs(E_acc,vin);
e_dec =  subs(E_dec,v_opt) - subs(E_dec,v_out);
 

e_acc =  subs(E_acc,v_opt) - subs(E_acc,vin);
e_dec =  subs(E_dec,v_opt) - subs(E_dec,v_out);

e_acc + e_dec;
 

d_acc  = double(subs(func_d_acc,t_acc_final)) - double(subs(func_d_acc,t_acc_init));
d_dec  = double(subs(func_d_dec,t_dec)) - double(subs(func_d_dec,t_vout));

d_const = distance - d_acc - d_dec ;

t_v_const = d_const/v_opt;

t_tot = t_acc_final - t_vin + t_v_const + t_dec - t_vout;






end

