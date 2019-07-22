% An accurate measurement-driven Energy model for an IRIS 3DR quadrotor.
% Copyright (c) 2019 
% Author: Carmelo Di Franco 
% Email: carmelodfr@gmail.com
% This code is licensed under MIT license (see LICENSE.txt for details)

function[angle] = angle_between_3_points(v2, v1, v3)
    

    angle = atan2d(abs((v2(1)-v1(1))*(v3(2)-v1(2))-(v2(2)-v1(2))*(v3(1)-v1(1))),...
                       (v2(1)-v1(1))*(v3(1)-v1(1))+(v2(2)-v1(2))*(v3(2)-v1(2)));

    angle = 180 - angle;
end