function [ x ] = clean_values( x,max_value )
%clean_values is a function that removes imaginary values(complex values) 
%from x and those values that are greater than max_value
%
% Copyright (c) 2019 
% Author: Carmelo Di Franco 
% Email: carmelodfr@gmail.com
% This code is licensed under MIT license (see LICENSE.txt for details)

    n = length( x);
    x = double (x);
    i = 1;
    while( i <= n)
       
        if (abs(imag(x(i))) > 0.00000000001 )
            x(i) = [];
            n = n -1;
       else
           if(x(i) < 0 || x(i)>max_value)
                x(i) = [];
                n = n -1;
           else
                i = i+1;
           end
       end
    end
    x = real(x);
end

