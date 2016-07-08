
load('Map3', 'movesX', 'movesY','corners');
load('ThresholdMaps','transferRegion','mainRegion');

counter = 1;
c = 0;
meanControl=false;
webcamShot = false;

%Flow Around Variables
eta=50;
zeta=0.1;
rhoNot=7.5;
alphaWant=0;

if webcamShot
    cam = webcam(2);
    rgbIm = snapshot(cam);
    imwrite(rgbIm,'flowAroundTest.png');
else
    rgbIm = imread('flowAroundTest.png');
end

%crop to have just the table view.
if (ispc== 1)  
    originalImage = imcrop(rgbIm,[50 10 500 400]);
else 
    originalImage = imcrop(rgbIm,[345 60 1110 850]);
end 
 s = size(originalImage);
 scale = 30;
 epsilon = 1* scale;
 sizeOfMap = floor(s/scale);
 imshow(originalImage);
 I = rgb2hsv(originalImage);
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.065;
channel1Max = 0.567;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.288;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.400;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

%  Determine objects properties
% STATS = regionprops(L, 'all');
% ObjectCentroid = STATS.Centroid;
%  Determine objects properties
[B,L] = bwboundaries(BW, 'noholes');


stat = regionprops(L,'Centroid','Area','PixelIdxList');

[maxValue,index] = max([stat.Area]);
centroids = cat(1, stat.Centroid);
ObjectCentroidX = centroids(index,1);
ObjectCentroidY = centroids(index,2);
indOX = floor(ObjectCentroidX/scale);
    indOY = floor(ObjectCentroidY/scale);
    if indOY ==1
        indOY = 2;
    end
    r = 2.5;
    alphaWant=atan2(movesX(indOY,indOX),movesY(indOY,indOX));
    repPointX=ObjectCentroidX/scale - r * cos(alphaWant+pi);
    repPointY=ObjectCentroidY/scale - r * sin(alphaWant+pi);
hold on
plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','black','linewidth',3);


    s = size(movesX);
    X = zeros(size(movesX));
    Y = zeros(size(movesX));
    DX = zeros(size(movesX));
    DY = zeros(size(movesX));
    for i = 1:s(2)
        for j = 1:s(1)
            X(i,j) = i;
            Y(i,j) = j;
            thetaD = atan2(j - repPointY,i - repPointX);
            angdiffD = alphaWant-thetaD;
            rhoD=sqrt((i-repPointX)^2 + (j-repPointY)^2);
            if(angdiffD > pi) 
        angdiffD = angdiffD - 2*pi;
            end
    if(angdiffD < -pi) 
        angdiffD = angdiffD + 2*pi;
    end
    FrepX=eta*((rhoD^(-1))-(rhoNot^(-1)))*(rhoD^(-1))^2*(repPointX-i);
        FrepY=eta*((rhoD^(-1))-(rhoNot^(-1)))*(rhoD^(-1))^2*(repPointY-j);
        
        attPointX=ObjectCentroidX/scale - r * cos(alphaWant);
        attPointY=ObjectCentroidY/scale - r * sin(alphaWant);
        rhoD=sqrt((i-attPointX)^2 + (j-attPointY)^2);
        FattX=zeta*(i-attPointX)/rhoD;
        FattY=zeta*(j-attPointY)/rhoD;
        

        DX(i,j)=cos(atan2((-FrepY-FattY),(-FattX-FrepX)));
        DY(i,j)=sin(atan2((-FrepY-FattY),(-FattX-FrepX)));
        end
    end
    hold on
        plot(attPointX *scale, attPointY*scale,'*','Markersize',16,'color','blue','linewidth',3);
        plot(repPointX*scale , repPointY*scale,'*','Markersize',16,'color','cyan','linewidth',3);
     hq=quiver(X*scale,Y*scale,DX,DY,'color',[0,0,0]); 
    if webcamShot 
        clear('cam');
    end