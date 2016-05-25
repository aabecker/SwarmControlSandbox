% Used http://www.mathworks.com/help/images/image-segmentation-using-the-color-thesholder-app.html
% if they overlap:http://www.mathworks.com/help/images/ref/imerode.html

% ******************Loading input Image**********************************
%fn = 'BadResult.png';  % robots are 14 px in diameter in this
%fn = 'FailImageGreen.png';
fn = 'fail119.jpeg';
rgbIm = imread(fn);
% Crop the image to use ROI.
%rgbIm = imcrop(rgbIm,[345 60 1110 850]);
% make grayscale.
I = rgb2hsv(rgbIm);


% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.199;
channel1Max = 0.302;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.084;
channel2Max = 0.646;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.551;
channel3Max = 0.807;

% % Define thresholds for channel 1 based on histogram settings
% channel1Min = 0.087;
% channel1Max = 0.547;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min = 0.057;
% channel2Max = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min = 0.452;
% channel3Max = 1.000;
 

% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
% Too easy, from
% http://www.mathworks.com/help/images/examples/detect-and-measure-circular-objects-in-an-image.tml
tic
  %threshold the image to remove shadows (and only show dark parts of kilobots)
    [centers, radii] = imfindcircles(BW,[10 19],'ObjectPolarity','bright','Sensitivity',0.92 );
     toc
M = mean(centers);
V = var(centers);
C = cov(centers);

%figure(1);BW = im2bw(originalImage, 0.19); imshow(BW);
figure(1)
imshow(rgbIm)
title([num2str(numel(radii)),' kilobots'])
%Draw the circles
h = viscircles(centers,radii);
hold on
plot(M(1,1) , M(1,2),'*','Markersize',16)
plot_gaussian_ellipsoid(M,C);
plot(centers(:,1),centers(:,2),'+','Markersize',16)
hold off

return
%li = imline
%imline measure size
%BW2= imfill(BW,'holes')
%se = strel('disk',4);
%I2 = imdilate(BW,se);
%[centers, radii] = imfindcircles(originalImage,[12 19],'ObjectPolarity','dark','Sensitivity',0.91 );

%%
I = rgb2gray(imread('FailImage.png'));
J = imadjust(I);
imshow(I)
figure, imshow(J)

% Could I find the holes in an image, and fill them?
%imfill(im,'holes'); 

%% http://www.mathworks.com/help/images/ref/imfill.html
fn = 'FailImage.png';
originalImage = imread(fn);
% Crop the image to use ROI.
originalImage = imcrop(originalImage,[345 60 1110 850]);
% make grayscale.
I = rgb2gray(originalImage);
I2 = imfill(I,'holes');
figure, imshow(I), figure, imshow(255-I)

