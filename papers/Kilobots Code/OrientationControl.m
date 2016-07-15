%%% Orientation Control With Kilobots.
%%% In this code we want to use arduino and our vision system to control
%%% kilobots for pushing a very long pivoted block in the fastest way we
%%% could do that.
%%% By Shiva Shahrokhi and Lillian Lin June 2016

clear all

%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
webcamShot = true;
relay = false;
success = false;
first = true;
delayTime = 10;
load('EmptyMap', 'corners');

if webcamShot
    cam = webcam(2);
else
    rgbIm = imread('colortest.png');
    figure
    imshow(rgbIm)
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
        a = arduino('Com5','uno');
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
    if relay
        if again== true
            relayOn(a,0);
            pause (delayTime);
            rgbIm = snapshot(cam);
        end 
    end
    
    %% Read in a webcam snapshot.
    if webcamShot
        pause (5);
        rgbIm = snapshot(cam);
    end
    
    %% crop to have just the table view.
    if webcamShot
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
        success = true;
    end
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
    ObjectOrientation = orientations(index)
    ObjectLength = majorLength(index);
    imshow(originalImage);
    while (ObjectOrientation>90||ObjectOrientation<=-90)
        if ObjectOrientation>90
            ObjectOrientation=ObjectOrientation-180;
        elseif ObjectOrientation<=-90
            ObjectOrientation=ObjectOrientation+180;
        end
    end
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
    
    %% Draw object
    plot(ObjectCentroidX,ObjectCentroidY,'*','Markersize',16,'color','blue','linewidth',3);
    t = (-01:.01:1)*100;
    line(ObjectCentroidX+t*sin(ObjectOrientation*pi/180+pi/2),ObjectCentroidY+t*cos(ObjectOrientation*pi/180+pi/2) , 'Color', 'black','linewidth',3);
    
    point1X = ObjectCentroidX - cos(ObjectOrientation*pi/180)* ObjectLength/2.3;
    point1Y = ObjectCentroidY + sin(ObjectOrientation*pi/180)* ObjectLength/2.3;
    point2X = ObjectCentroidX + cos(ObjectOrientation*pi/180)* ObjectLength/2.3;
    point2Y = ObjectCentroidY - sin(ObjectOrientation*pi/180)* ObjectLength/2.3;
    
    slope = (point2Y-point1Y)/(point2X-point1X);
    offset = point1Y - slope*point1X;
    if point1Y<point2Y
        topPointX = point1X;
        topPointY = point1Y;
        botPointX = point2X;
        botPointY = point2Y;
    else
        botPointX = point1X;
        botPointY = point1Y;
        topPointX = point2X;
        topPointY = point2Y;
    end
    
    plot(botPointX,botPointY,'v','Markersize',16,'color','white','linewidth',3);
    plot(topPointX,topPointY,'^','Markersize',16,'color',[0.75 0 0.75],'linewidth',3);
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
    %% Create Goal Angle
    goalAngle=-60;
    while (goalAngle>90||goalAngle<=-90)
        if goalAngle>90
            goalAngle=goalAngle-180;
        elseif goalAngle<=-90
            goalAngle=goalAngle+180;
        end
    end
    
    line(ObjectCentroidX+t*sin(goalAngle*pi/180+pi/2),ObjectCentroidY+t*cos(goalAngle*pi/180+pi/2) , 'Color', 'green','linewidth',3);
   
    [s, l] = size(centers);
    h = viscircles(centers,radii,'EdgeColor','b');
    if s > 5 
        again = false;    
        if (V > maxVar)
            VarCont = true;
            epsilon = bigEpsilon;
            corInd = 1;
            currgoalX = (corners(corInd,1))*scale;
            currgoalY = (corners(corInd,2))*scale;
        end
        if V< minVar
            VarCont = false;
        end
        if ~VarCont
            %% Determine Non-Variance Control Goal w/ 5 Degree Tolerance
            if ObjectOrientation>goalAngle+5||ObjectOrientation<goalAngle-5
                goal1X = ObjectCentroidX - cos(goalAngle*pi/180)* ObjectLength/3;
                goal1Y = ObjectCentroidY + sin(goalAngle*pi/180)* ObjectLength/3;
                goal2X = ObjectCentroidX + cos(goalAngle*pi/180)* ObjectLength/3;
                goal2Y = ObjectCentroidY - sin(goalAngle*pi/180)* ObjectLength/3;

                if goal1Y<goal2Y
                    topGoalX = goal1X;
                    topGoalY = goal1Y;
                    botGoalX = goal2X;
                    botGoalY = goal2Y;
                else
                    botGoalX = goal1X;
                    botGoalY = goal1Y;
                    topGoalX = goal2X;
                    topGoalY = goal2Y;
                end          
                angdiffTop = ObjectOrientation*pi/180-atan2((M(2) - topGoalY),(M(1) - topGoalX));
                angdiffBot = ObjectOrientation*pi/180-atan2((M(2) - botGoalY),(M(1) - botGoalX));
                if(angdiffTop > pi) 
                    angdiff = angdiff - 2*pi;
                end

                if(angdiffTop < -pi) 
                    angdiff = angdiff + 2*pi;
                end
                if(angdiffBot > pi) 
                    angdiff = angdiff - 2*pi;
                end

                if(angdiffBot < -pi) 
                    angdiff = angdiff + 2*pi;
                end
                if 
                [ currgoalX,currgoalY ] = FlowForce(M(1),M(2),ObjectCentroidX,ObjectCentroidY,topGoalX,topGoalY)
                [ currgoalX,currgoalY ] = FlowForce(M(1),M(2),ObjectCentroidX,ObjectCentroidY,botGoalX,botGoalY)
                if (topGoalX>topPointX && M(1)<(M(2)-offset)/slope)||(topGoalX<topPointX && M(1)>(M(2)-offset)/slope)
                    currgoalX = topPointX;
                    currgoalY = topPointY;
                else
                    currgoalX = botPointX;
                    currgoalY = botPointY;
                end
                
            else
                VarCont = true;
            end
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
                elseif M(1,2) < currgoalY - epsilon
                    Relay = 8;
                    relayOn(a,Relay);
                    pause(delayTime);
                else
                    Relay = 1;
                    relayOn(a,Relay);
                    pause(delayTime);
                end
            elseif M(1,1)  < currgoalX-epsilon
                if M(1,2) > currgoalY + epsilon
                    Relay = 4;
                    relayOn(a,Relay);
                    pause(delayTime);
                elseif M(1,2) < currgoalY - epsilon
                    Relay = 6;
                    relayOn(a,Relay);
                    pause(delayTime);
                else
                    Relay = 5;
                    relayOn(a,Relay);
                    pause(delayTime);
                end
            elseif M(1,2) > currgoalY+epsilon
                Relay = 3;
                relayOn(a,Relay);
                pause(delayTime);
            elseif M(1,2) < currgoalY-epsilon
                Relay=7;
                relayOn(a,Relay);
                pause(delayTime);
            else       
                %VarCont = true;
                again = true;
                epsilon = smallEpsilon;
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
%         if (toc(t0) > 300 && relay)
%             success = true;
% 
%             figure
%             plot(drawTime(:,2),drawTime(:,1));
%         end
    end
end