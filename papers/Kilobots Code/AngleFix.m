function [ angle ] = AngleFix( angle )
% ANGLEFIX takes an angle and makes it between pi/2 and -pi/2
%   ANGLEFIX(x)
%
%       x = an angle in radians

    while (angle > pi/2||angle<= -pi/2)
        if angle>pi/2
            angle=angle - pi;
        elseif angle <= -pi/2
            angle=angle + pi;
        end
    end 

end