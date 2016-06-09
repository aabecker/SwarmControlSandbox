%%% Pure Torque Control With Kilobots.
%%% In this code we want to use arduino and our vision system to control
%%% kilobots for pushing a very long pivoted block in the fastest way we
%%% could do that.
%%% By Shiva Shahrokhi June 2016

clear all

%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
webcam = false;
relay = false;
success = false;
if webcam
    cam = webcam(1);
end

Relay=0;
VarCont = false;
frameCount = 1;
again = true;
scale = 30;
epsilon = 1* scale;
if relay
    if (ispc)  
        a = arduino('Com4','uno');
    else 
        a = arduino('/dev/tty.usbmodem1421','uno');
    end 
end
minDis = 10000;
corInd = 0;
maxVar = 12000; %was 16000
minVar = 11000; %was 12000

while success == false
    success = true;
    if relay
    if again== true
        relayOn(a,0);
    pause (10);
    end 
    end
    % Read in a webcam snapshot.
    if webcam
        rgbIm = snapshot(cam);
    else
        rgbIm = imread('Test2.jpeg');
    end
    %crop to have just the table view.
    if webcam
    if (ispc)  
        originalImage = imcrop(rgbIm,[50 10 500 400]);
    else 
        originalImage = imcrop(rgbIm,[345 60 1110 860]);
    end 
    else
        originalImage = rgbIm;
    end
    
    I2 = rgb2hsv(originalImage);

% Define thresholds for channel 1 based on histogram settings
channel1Min2 = 0.847;
channel1Max2 = 0.946;

% Define thresholds for channel 2 based on histogram settings
channel2Min2 = 0.174;
channel2Max2 = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min2 = 0.657;
channel3Max2 = 1.000;

% Create mask based on chosen histogram thresholds
BW2 = (I2(:,:,1) >= channel1Min2 ) & (I2(:,:,1) <= channel1Max2) & ...
    (I2(:,:,2) >= channel2Min2 ) & (I2(:,:,2) <= channel2Max2) & ...
    (I2(:,:,3) >= channel3Min2 ) & (I2(:,:,3) <= channel3Max2);
    
    % make HSV scale.
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

    %finding Object Orientation
    [B,L] = bwboundaries(BW2, 'noholes');
    stat = regionprops(L,'Centroid','Orientation','MajorAxisLength');
    centroids = cat(1, stat.Centroid);
    orientations = cat(1, stat.Orientation);
    majorLength = cat(1, stat.MajorAxisLength);
    ObjectCentroidX = centroids(1,1);
    ObjectCentroidY = centroids(1,2);
    imshow(originalImage);
    hold on
    plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','black','linewidth',3);
    for i = 1:2
        plot(centroids(i,1) , centroids(i,2),'*','Markersize',16,'color','black','linewidth',3);
        t = (-01:.01:1)*100;
        line(centroids(i,1)+t*cos(orientations(i)*pi/180),centroids(i,2)+t*sin(orientations(i)*pi/180) , 'Color', 'black','linewidth',3);
    plot(centroids(i,1) - cos(orientations(i)*pi/180)* majorLength(i) * 1/4 , centroids(i,2) + sin(orientations(i)*pi/180)* majorLength(i) * 1/4,'*','Markersize',16,'color','cyan','linewidth',3);
    end
    
    %threshold the image to remove shadows (and only show dark parts of kilobots)
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
    if s > 5 
        again = false;
        
        if (V > maxVar)
            VarCont = true;
            for i = 1:size(corners)
                dist = sqrt((Mean(1,1)/scale - corners(i,1)) * (Mean(1,1)/scale - corners(i,1)) + (Mean(1,2)/scale- corners(i,2)) * (Mean(1,2)/scale- corners(i,2)));
                if minDis > dist
                    minDis = dist;
                    corInd = i;
                end   
            end
            currgoalX = (corners(corInd,1))*scale;
            currgoalY = (corners(corInd,2))*scale;
    
        end
   if V< minVar
      VarCont = false;
   end
   if ~VarCont
       C = 1/4;
       currgoalX = ObjectCentroidX - cos(orientations(1)*pi/180)* majorLength(1) * C;
       currgoalY = ObjectCentroidY + sin(orientations(1)*pi/180)* majorLength(1)*C;
   end
   plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red', 'linewidth',3);
    plot(currgoalX , currgoalY,'*','Markersize',16,'color','cyan','linewidth',3);
    
    plot_gaussian_ellipsoid(M,C);
    M(frameCount)=getframe(gcf); 
     frameCount = frameCount +1;
      hold off
     if relay 
      if M(1,1) > currgoalX+epsilon
        if M(1,2) > currgoalY + epsilon
        Relay = 2;
        relayOn(a,Relay);
        pause(delayTime);
        else if M(1,2) < currgoalY - epsilon
        Relay = 8;
        relayOn(a,Relay);
        pause(delayTime);
            else
        Relay = 1;
        relayOn(a,Relay);
        pause(delayTime);
            end
        end
    else if M(1,1)  < currgoalX-epsilon
          
       if M(1,2) > currgoalY + epsilon
        Relay = 4;
        relayOn(a,Relay);
        pause(delayTime);
        else if M(1,2) < currgoalY - epsilon
        Relay = 6;
       relayOn(a,Relay);
        pause(delayTime);
            else
        Relay = 5;
       relayOn(a,Relay);
        pause(delayTime);
            end
        end
       
        else if M(1,2) > currgoalY+epsilon
     
        Relay = 3;
       relayOn(a,Relay);
        pause(delayTime);
            else if M(1,2) < currgoalY-epsilon
                    Relay=7;
                    relayOn(a,Relay);
                    pause(delayTime);
       
                else       
        %VarCont = true;
        again = true;
                end
            end
        end
      end
    end
    end
end