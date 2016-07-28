
function [ currgoalX,currgoalY ] = PositiveFlow(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY) 
% POSITIVEFLOW calculates the vector for a positive force on an a point
%              towards an object.
%   POSITIVEFLOW(RobotMeanX,RobotMeanY,AttPointX,AttPointY,RepPointX,RepPointY)
%
%       RobotMeanX,RobotMeanY = here the robots currently are
%       AttPointX,AttPointY = Where the robots should move towards
%       RepPointX,RepPointY = Where the robots should be pushed from
%
%       zeta is the scaling factor of the attractive field
%
% By Lillian Lin Summer 2016
    zeta=1;

    FrepX=0;
    FrepY=0;
    
    rho=dist2points(AttPointX,AttPointY,RobotMeanX,RobotMeanY);
    FattX=zeta*(RobotMeanX-AttPointX)/rho;
    FattY=zeta*(RobotMeanY-AttPointY)/rho;
        
    currgoalX=cos(atan2((-FrepY-FattY),(-FattX-FrepX)));
    currgoalY=sin(atan2((-FrepY-FattY),(-FattX-FrepX)));
end