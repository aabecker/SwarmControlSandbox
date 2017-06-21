%% Orientation Control With Kilobots.
% In this code we use arduino and our vision system to control a swarm of
% kilobots to push a very long block to a randomized goal angle
%           See also FLOWFORCE
% By Shiva Shahrokhi and Lillian Lin July 2016

clear all

%% Define webcam
webcamShot = true;

if webcamShot
    cam = webcam(1);
else
    rgbIm = imread('colortest.png');
end

%% initalize variables
relay = true;
success = false;
flowDebug = true;
first = true;
delayTime = 1;
load('EmptyMapMac', 'corners');

Relay=1;
VarCont = false;
frameCount = 1;
again = true;
bigEpsilon = 2 ;
smallEpsilon = 1;
epsilon = bigEpsilon;
obstacles = [];
corInd = 0;
maxVar = 12000; %was 16000
minVar = 11000; %was 12000
version = 0;
t0 = tic;

if relay
    if (ispc)  
        a = arduino('Com4','uno');
    else 
        a = arduino('/dev/tty.usbmodem1421','uno');
    end 
end

while success == false
    if relay
        if again== true
            relayOn(a,0);
            pause (delayTime);
            rgbIm = snapshot(cam);
        end 
    end
    again=false;
    if webcamShot
         %% Read in a webcam snapshot.
        pause (delayTime);
        rgbIm = snapshot(cam);
        %% crop to have just the table view.
        if (ispc)  
            originalImage = imcrop(rgbIm,[50 10 500 400]);
             imwrite(originalImage,'colortest.png');
%             success=true;
        else 
            originalImage = imcrop(rgbIm,[345 60 1110 860]);
                     imwrite(originalImage,'test.png');
        end 
    else
        originalImage = rgbIm;
        success = true;     % Makes it only go through once if static image
    end
    
    scale = floor(size(originalImage,2)/30);
    scaledPic = size(originalImage)/scale;
    
    %% Threshold for Object
    I2 = rgb2hsv(originalImage);
    
    % Define thresholds for channel 1 based on histogram settings
    channel1Min2 = 0.862;
    channel1Max2 = 0.945;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min2 = 0.272;
    channel2Max2 = 1.000;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min2 = 0.000;
    channel3Max2 = 1.000;

    % Create mask based on chosen histogram thresholds
    BW2 = (I2(:,:,1) >= channel1Min2 ) & (I2(:,:,1) <= channel1Max2) & ...
        (I2(:,:,2) >= channel2Min2 ) & (I2(:,:,2) <= channel2Max2) & ...
        (I2(:,:,3) >= channel3Min2 ) & (I2(:,:,3) <= channel3Max2);
    %% Threshold for Robots
    % make HSV scale.
    I = rgb2hsv(originalImage);

    %Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.065;
    channel1Max = 0.567;

    %Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.288;
    channel2Max = 1.000;

    %Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.400;
    channel3Max = 1.000;

    % Create mask based on chosen histogram thresholds
    BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

    %% finding Object Orientation
    [B,L] = bwboundaries(BW2, 'noholes');
    stat = regionprops(L,'Centroid','Orientation','MajorAxisLength', 'Area');
    [maxValue,index] = max([stat.Area]);
    centroids = cat(1, stat.Centroid);
    orientations = cat(1, stat.Orientation);
    majorLength = cat(1, stat.MajorAxisLength);
    ObjectCentroidX = centroids(index,1);
    ObjectCentroidY = centroids(index,2);
    ObjectOrientation = orientations(index);
    ObjectOrientation=deg2rad(ObjectOrientation);
    ObjectOrientation=AngleFix(ObjectOrientation,pi/2);
    ObjectLength = majorLength(index);
    
    imshow(originalImage);

    hold on
    
    %% Draw object
    plot(ObjectCentroidX,ObjectCentroidY,'*','Markersize',16,'color','cyan','linewidth',3);
    t = (-01:.01:1)*100;
    line(ObjectCentroidX+t*sin(ObjectOrientation+pi/2),ObjectCentroidY+t*cos(ObjectOrientation+pi/2) , 'Color', 'black','linewidth',3);
    
    %% threshold the image to remove shadows (and only show dark parts of kilobots)
    if ispc
        [centers, radii] = imfindcircles(BW,[4 6],'ObjectPolarity','bright','Sensitivity',0.97); 
    else
        [centers, radii] = imfindcircles(BW,[10 19],'ObjectPolarity','bright','Sensitivity',0.92 );
    end
    
    % Mean
    M = mean(centers);
    %Variance
    V = var(centers);
    %Covariance
    C = cov(centers);
    
    [s, l] = size(centers);
    h = viscircles(centers,radii,'EdgeColor','b');
    plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red', 'linewidth',3);
    plot_gaussian_ellipsoid(M,C);
        
    hold off
    %% Turn on Lights
 
    if M(1,1) > ObjectCentroidX+epsilon
        if M(1,2) > ObjectCentroidY + epsilon
            Relay = 2;
            relayOn(a,Relay);
            pause(delayTime);
        elseif M(1,2) < ObjectCentroidY - epsilon
            Relay = 8;
            relayOn(a,Relay);
            pause(delayTime);
        else
            Relay = 1;
            relayOn(a,Relay);
            pause(delayTime);
        end
    elseif M(1,1) < ObjectCentroidX-epsilon   
        if M(1,2) > ObjectCentroidY + epsilon
            Relay = 4;
            relayOn(a,Relay);
            pause(delayTime);
        elseif M(1,2) < ObjectCentroidY - epsilon
            Relay = 6;
            relayOn(a,Relay);
            pause(delayTime);
        else
            Relay = 5;
            relayOn(a,Relay);
            pause(delayTime);
        end     
    elseif M(1,2) > ObjectCentroidY+epsilon  
        Relay = 3;
        relayOn(a,Relay);
        pause(delayTime);
    elseif M(1,2) < ObjectCentroidY-epsilon
        Relay=7;
        relayOn(a,Relay);
        pause(delayTime);
    end
end