function [ angle ] = AngleFix( angle , threshold)
% ANGLEFIX takes an angle and makes it an equivalent in between a threshold 
%          in which on the the stated threshold is both the positive and 
%          neagtive threshold. 
%   ANGLEFIX(angle,threshold)
%
%       x = an angle you would like to change
%       threshold = the edge values wanted for angles
%
%       if threshold = pi/2 or 90 then the angle created will be under the
%       assumption that you have a half circle and the negative version of  
%       an angle is the same as the positive version of an angle
%       
%       if threshold = pi or 180 then the angle created will be under the
%       assumption that you have a full circle and the negative version of  
%       an angle is not the same as the positive version of an angle

    while (angle > threshold||angle<= -threshold)
        if angle>threshold
            angle=angle - 2*threshold;
        elseif angle <= -threshold
            angle=angle + 2*threshold;
        end
    end 

end