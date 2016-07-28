
function [ currgoalX,currgoalY ] = NeagtiveFlow(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY) 
% NEAGTIVEFLOW calculates the vector for a negative force on an a point
%              to repel it away from another point.
%   NEAGTIVEFLOW(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY)
%
%       RobotMeanX,RobotMeanY = here the robots currently are
%       AttPointX,AttPointY = Where the robots should move towards
%       RepPointX,RepPointY = Where the robots should be pushed from
%
%       eta is the scaling factor of the repulsive field
%       rhoNot is the area of effect for the repulsive fields
%
% By Lillian Lin Summer 2016
    eta=50;
    rhoNot=7.5;

    rho=dist2points(RepPointX,RepPointY,RobotMeanX,RobotMeanY);
    if (rho<rhoNot)
        FrepX=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(RepPointX-RobotMeanX);
        FrepY=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(RepPointY-RobotMeanY);
    else 
       FrepX=0;
       FrepY=0;
    end
    FattX=0;
    FattY=0;
        
    currgoalX=cos(atan2((-FrepY-FattY),(-FattX-FrepX)));
    currgoalY=sin(atan2((-FrepY-FattY),(-FattX-FrepX)));
end