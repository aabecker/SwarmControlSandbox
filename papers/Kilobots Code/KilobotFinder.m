% ******************Loading input Image**********************************
cam = webcam(2); %Defines the webcam.
% Read in a webcam snapshot.
originalImage = snapshot(cam);
% Crop the image to use ROI.
originalImage = imcrop(originalImage,[345 60 1110 850]);
% make grayscale.
originalImage = rgb2gray(originalImage);

%originalImage = originalImage(33:33+550,250:250+810,:);  %ROI, you can use imrect to draw a rectangle using your mouse.
imshow(originalImage)
% Too easy, from
% http://www.mathworks.com/help/images/examples/detect-and-measure-circular-objects-in-an-image.tml
tic
BW = im2bw(originalImage, 0.25);  %threshold the image to remove shadows (and only show dark parts of kilobots)
    [centers, radii] = imfindcircles(BW,[12 19],'ObjectPolarity','dark','Sensitivity',0.94 );
%[centers, radii] = imfindcircles(originalImage,[12 19],'ObjectPolarity','dark','Sensitivity',0.91 );
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