function [ distance ] = dist2points(x1,y1,x2,y2)
% DIST2POINTS calculates distance between two points
%   DIST2POINTS(x1,y1,x2,y2)
%       x1,y1 = start points
%       x2,y2 = end points
%
% By Lillian Lin Summer 2016
    xs=x2-x1;
    ys=y2-y1;
    xs=xs^2;
    ys=ys^2;
    distance=sqrt(xs+ys);
end

