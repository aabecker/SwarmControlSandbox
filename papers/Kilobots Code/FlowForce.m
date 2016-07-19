
function [ currgoalX,currgoalY ] = FlowForce(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY) 
% FLOWFORCE calculates distance between two points
%   FLOWFORCE(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY)
%
%       RobotMeanX,RobotMeanY = here the robots currently are
%       AttPointX,AttPointY = Where the robots should move towards
%       RepPointX,RepPointY = Where the robots should be pushed from
%
%       eta and zeta are the ratio between the repulsive and attractive
%       forces respectively.
%       rhoNot is the area of effect for the repulsive fields
    eta=50;
    zeta=1;
    rhoNot=7.5;

    rho=dist2points(RepPointX,RepPointY,RobotMeanX,RobotMeanY);
    FrepX=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(RepPointX-RobotMeanX);
    FrepY=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(RepPointY-RobotMeanY);

    rho=dist2points(AttPointX,AttPointY,RobotMeanX,RobotMeanY);
    FattX=zeta*(RobotMeanX-AttPointX)/rho;
    FattY=zeta*(RobotMeanY-AttPointY)/rho;
        
    currgoalX=RobotMeanX+cos(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;
    currgoalY=RobotMeanY+sin(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;
end