originalImage = imread('fail05-25-201609.21.11.jpeg');
% make HSV scale.
I = rgb2hsv(originalImage);
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.292;
channel1Max = 0.437;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.113;
channel2Max = 0.538;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.383;
channel3Max = 0.765;
% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

[B,L] = bwboundaries(BW, 'noholes');


stat = regionprops(L,'Centroid','Area','PixelIdxList');


[maxValue,index] = max([stat.Area]);
centroids = cat(1, stat.Centroid);
ObjectCentroidX = centroids(index,1);
ObjectCentroidY = centroids(index,2);

BW(stat(index).PixelIdxList)=0;
imshow(BW);
hold on
plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','cyan','linewidth',3);
hold off