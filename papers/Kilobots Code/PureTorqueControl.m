%%% Pure Torque Control With Kilobots.
%%% In this code we want to use arduino and our vision system to control
%%% kilobots for pushing a very long pivoted block in the fastest way we
%%% could do that.
%%% By Shiva Shahrokhi June 2016

clear all

%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
webcam1 = false;
relay = false;
inDebug = true;
success = false;
first = true;
delayTime = 10;
load('EmptyMap', 'corners');

% We have 8 Relays.
%west
RELAY1 = 9;
% northwest
RELAY2 = 8;
%north
RELAY3 = 5;
% northeast
RELAY4 = 2;
%east
RELAY5 = 7;
%southeast
RELAY6 = 3;
%south
RELAY7 = 6;
%southwest
RELAY8 = 4;

if webcam1
    cam = webcam(1);
end

Relay=1;
VarCont = false;
frameCount = 1;
again = true;
scale = 30;
bigEpsilon = 2 ;
smallEpsilon = 1;
epsilon = bigEpsilon;
obstacles = [];
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
t0 = tic;
while success == false
    if inDebug
        success = true;
    end
    if relay
    if again== true
        relayOn(a,0);
        
%         writeDigitalPin(a,RELAY3,0);
%         writeDigitalPin(a, RELAY8,0);
%         writeDigitalPin(a,RELAY5,0);
%         writeDigitalPin(a,RELAY7,0);
     pause (delayTime);
    end 
    end
    % Read in a webcam snapshot.
    if webcam1
        rgbIm = snapshot(cam);
    else
        rgbIm = imread('test.png');
        figure
        imshow(rgbIm)
    end
    %crop to have just the table view.
    if webcam1
    if (ispc)  
        originalImage = imcrop(rgbIm,[50 10 500 400]);
    else 
        originalImage = imcrop(rgbIm,[345 60 1110 860]);
                 imwrite(originalImage,'test.png');
    end 
    else
        originalImage = rgbIm;
    end
    
    I2 = rgb2hsv(originalImage);
    
% Define thresholds for channel 1 based on histogram settings
channel1Min2 = 0.902;
channel1Max2 = 0.938;

% Define thresholds for channel 2 based on histogram settings
channel2Min2 = 0.205;
channel2Max2 = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min2 = 0.795;
channel3Max2 = 1.000;

% % Define thresholds for channel 1 based on histogram settings
% channel1Min2 = 0.907;
% channel1Max2 = 0.933;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min2 = 0.488;
% channel2Max2 = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min2 = 0.888;
% channel3Max2 = 1.000;



% % Define thresholds for channel 1 based on histogram settings
% channel1Min2 = 0.899;
% channel1Max2 = 0.932;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min2 = 0.616;
% channel2Max2 = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min2 = 0.779;
% channel3Max2 = 1.000;
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
    stat = regionprops(L,'Centroid','Orientation','MajorAxisLength', 'Area');
    [maxValue,index] = max([stat.Area]);
    area = cat(1,stat.Area)
    centroids = cat(1, stat.Centroid);
    orientations = cat(1, stat.Orientation);
    majorLength = cat(1, stat.MajorAxisLength);
    ObjectCentroidX = centroids(index,1);
    ObjectCentroidY = centroids(index,2);
    ObjectOrientation = orientations(index);
    ObjectLength = majorLength(index);
    imshow(originalImage);
    hold on
    for i = 1:size(corners)
        txt = int2str(i);
        text(corners(i,1)* scale,corners(i,2)*scale,txt,'HorizontalAlignment','right')
        %plot( corners(i,1)* scale, corners(i,2)*scale,'*','Markersize',16,'color','red','linewidth',3);
    end
    if first
        drawTime = [ObjectOrientation,toc(t0)];
        first = false;
    end
    if inDebug
   for i = 1: size(area)
       if area(i) > 50
           obstacles = [obstacles; i];
       end
   end
   size(obstacles)
   for i = 1: size(obstacles)
       hold on
       plot(centroids(obstacles(i),1) , centroids(obstacles(i),2),'*','Markersize',16,'color','black','linewidth',3);
       t = (-01:.01:1)*100;
       line(centroids(obstacles(i),1)+t*sin(orientations(obstacles(i))*pi/180+pi/2),centroids(obstacles(i),2)+t*cos(orientations(obstacles(i))*pi/180+pi/2) , 'Color', 'black','linewidth',3);
       plot(centroids(obstacles(i),1) + cos(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3,centroids(obstacles(i),2) - sin(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3 ,'*','Markersize',16,'color','white','linewidth',3);
   end
%     for i = 1:size(centroids)
%        % plot(centroids(i,1) , centroids(i,2),'*','Markersize',16,'color','black','linewidth',3);
%         t = (-01:.01:1)*100;
%         %line(centroids(i,1)+t*sin(orientations(i)*pi/180+pi/2),centroids(i,2)+t*cos(orientations(i)*pi/180+pi/2) , 'Color', 'black','linewidth',3);
%         %line(centroids(i,1)+ majorLength(i)*sin(orientations(i)*pi/180+pi/2) , centroids(i,2)+majorLength(i)*cos(orientations(i)*pi/180+pi/2));
%     %plot(centroids(i,1) + cos(orientations(i)*pi/180)* majorLength(i) * 1/4 , centroids(i,2) - sin(orientations(i)*pi/180)* majorLength(i) * 1/4,'*','Markersize',16,'color','cyan','linewidth',3);
%     
%     %plot(centroids(i,1) + cos(orientations(i)*pi/180)* majorLength(i),centroids(i,2) - sin(orientations(i)*pi/180)* majorLength(i) * 1/4,'*','Markersize',16,'color','white','linewidth',3);
%     end
    %plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','blue','linewidth',3);
     %  t = (-01:.01:1)*100;
     %   line(ObjectCentroidX+t*sin(ObjectOrientation*pi/180+pi/2),ObjectCentroidY+t*cos(ObjectOrientation*pi/180+pi/2) , 'Color', 'black','linewidth',3);

        %plot(ObjectCentroidX + cos(ObjectOrientation*pi/180)* ObjectLength,ObjectCentroidY - sin(ObjectOrientation*pi/180)* ObjectLength ,'*','Markersize',16,'color','white','linewidth',3);
    %currgoalX1 = ObjectCentroidX + cos(ObjectOrientation*pi/180)*ObjectLength/2.3 ;
    %   currgoalY1 = ObjectCentroidY - sin(ObjectOrientation*pi/180)* ObjectLength/2.3;
       %plot(currgoalX1 , currgoalY1,'*','Markersize',16,'color','cyan','linewidth',3);
    
       
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
            epsilon = bigEpsilon;
%             for i = 1:size(corners)
%                 dist = sqrt((M(1,1)/scale - corners(i,1)) * (M(1,1)/scale - corners(i,1)) + (M(1,2)/scale- corners(i,2)) * (M(1,2)/scale- corners(i,2)));
%                 if minDis > dist
%                     minDis = dist;
%                     corInd = i;
%                 end   
%             end
            corInd = 1;
            currgoalX = (corners(corInd,1))*scale;
            currgoalY = (corners(corInd,2))*scale;
    
        end
   if V< minVar
      VarCont = false;
   end
   if ~VarCont
       CL =-1/4;
       currgoalX = ObjectCentroidX + cos(ObjectOrientation*pi/180)*ObjectLength * CL;
       currgoalY = ObjectCentroidY - sin(ObjectOrientation*pi/180)* ObjectLength*CL;
   end
   plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red', 'linewidth',3);
    plot(currgoalX , currgoalY,'*','Markersize',16,'color','cyan','linewidth',3);
    
    plot_gaussian_ellipsoid(M,C);
    newDot = [ObjectOrientation, toc(t0)];
    drawTime = [drawTime;newDot];
    %M(frameCount)=getframe(gcf); 
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
        epsilon = smallEpsilon;
                end
            end
        end
      end

        
%         if M(1,2) > currgoalY+epsilon
%         writeDigitalPin(a,RELAY3,0);
%         writeDigitalPin(a, RELAY8,1);
%         writeDigitalPin(a,RELAY5,1);
%         writeDigitalPin(a,RELAY7,1);
%         pause(delayTime);
%        
%     else if M(1,2) < currgoalY-epsilon
%             writeDigitalPin(a,RELAY3,1);
%         writeDigitalPin(a, RELAY8,1);
%         writeDigitalPin(a,RELAY5,1);
%         writeDigitalPin(a,RELAY7,0);
%         pause(delayTime);
%         else if M(1,1) > currgoalX+epsilon
%         writeDigitalPin(a,RELAY3,1);
%         writeDigitalPin(a, RELAY8,0);
%         writeDigitalPin(a,RELAY5,1);
%         writeDigitalPin(a,RELAY7,1);
%         pause(1);
%        
%     else if M(1,1) < currgoalX-epsilon
%             writeDigitalPin(a,RELAY3,1);
%         writeDigitalPin(a, RELAY8,1);
%         writeDigitalPin(a,RELAY5,0);
%         writeDigitalPin(a,RELAY7,1);
%         pause(delayTime);
%         else 
%             epsilon = smallEpsilon;
%             writeDigitalPin(a,RELAY3,0);
%             writeDigitalPin(a, RELAY8,0);
%             writeDigitalPin(a,RELAY5,0);
%             writeDigitalPin(a,RELAY7,0);
%             pause(delayTime);
%             again = true;
%         end
%             end
%         end
%         end
    save('TorqueResultC03rd', 'drawTime');
    if (toc(t0) > 300 && relay)
        success = true;
        
        figure
        plot(drawTime(:,2),drawTime(:,1));
    end
     end
    end
end
