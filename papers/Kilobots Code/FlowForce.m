
function [ currgoalX,currgoalY ] = FlowForce(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY) 
% FLOWFORCE calculates a vector for the potential field effect on a point
%   FLOWFORCE(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY)
%
%       RobotMeanX,RobotMeanY = here the robots currently are
%       AttPointX,AttPointY = Where the robots should move towards
%       RepPointX,RepPointY = Where the robots should be pushed from
%
%       eta and zeta are the ratio between the repulsive and attractive
%       forces respectively.
%       rhoNot is the area of effect for the repulsive fields
%
% By Lillian Lin Summer 2016
    eta=50;
    zeta=1;
    rhoNot=7.5;

    rho=dist2points(RepPointX,RepPointY,RobotMeanX,RobotMeanY);
    if (rho<rhoNot)
        FrepX=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(RepPointX-RobotMeanX);
        FrepY=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(RepPointY-RobotMeanY);
    else 
       FrepX=0;
       FrepY=0;
    end
    rho=dist2points(AttPointX,AttPointY,RobotMeanX,RobotMeanY);
    FattX=zeta*(RobotMeanX-AttPointX)/rho;
    FattY=zeta*(RobotMeanY-AttPointY)/rho;
        
    currgoalX=cos(atan2((-FrepY-FattY),(-FattX-FrepX)));
    currgoalY=sin(atan2((-FrepY-FattY),(-FattX-FrepX)));
end