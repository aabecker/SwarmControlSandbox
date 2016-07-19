function KilobotFinderV2()
%  searces an image to identify the locations of kilobots
% background is white, we may see up to 101 kilobots, and the image may be
% illuminated from different angles and get errors.
% %  Aaron T. Becker, atbecker@uh.edu

% Shadows cause probems.  Ideas:
%  1. threshold image to remove light-grey shadows, then use imfindcircles
%  2. use a template image (of a kilobot), use this to make an intensity
%  map, use imfindcircles to find robots
%  


% ******************Loading input Image**********************************
imName = 'TestImage4.png';  %kilobots are 30 px in my image
%  Aaron T. Becker, atbecker@uh.edu
% Read in a webcam snapshot.

% Crop the image to use ROI.
originalImage = imread(imName);
% make grayscale.
originalImage = rgb2gray(originalImage);

%originalImage = originalImage(33:33+550,250:250+810,:);  %ROI, you can use imrect to draw a rectangle using your mouse.
figure(1)
imshow(originalImage);

BW = im2bw(originalImage, 0.25);  %threshold the image to remove shadows (and only show dark parts of kilobots)
figure(2)
imshow(BW)
figure(3)
% Too easy, from
% http://www.mathworks.com/help/images/examples/detect-and-measure-circular-objects-in-an-image.tml
tic
[centers, radii] = imfindcircles(BW,[12 19],'ObjectPolarity','dark','Sensitivity',0.94 );
toc
M = mean(centers);
V = var(centers);
C = cov(centers);


imshow(originalImage);
%Draw the circles
h = viscircles(centers,radii);
hold on
plot(M(1,1) , M(1,2),'*','Markersize',16)
plot_gaussian_ellipsoid(M,C);
plot(centers(:,1),centers(:,2),'+','Markersize',16)
hold off
clear('cam');