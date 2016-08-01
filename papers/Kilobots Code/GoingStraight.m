%% Going Straight With Kilobots.
% In this code we use arduino and our vision system to control a swarm of
% kilobots to push a very long block in a straight line for as long as 
% possible
%           See also FLOWFORCE
% By Shiva Shahrokhi and Lillian Lin July 2016

clear all

%% Define webcam
webcamShot = false;

if webcamShot
    cam = webcam(1);
else
    rgbIm = imread('colortest.png');
end

%% initalize variables
relay = false;
success = false;
flowDebug = true;
first = true;
delayTime = 10;
load('EmptyMap', 'corners');

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
justOriented = 0;

if relay
    if (ispc)  
        a = arduino('Com4','uno');
    else 
        a = arduino('/dev/tty.usbmodem1421','uno');
    end 
end

while success == false
    success = true;
    if relay
        if again== true
            relayOn(a,0);
            pause (delayTime);
            rgbIm = snapshot(cam);
        end 
    end
    
    if webcamShot
         %% Read in a webcam snapshot.
        pause (5);
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
    line(ObjectCentroidX+t*sin(ObjectOrientation+pi/2),ObjectCentroidY+t*cos(ObjectOrientation+pi/2) , 'Color', 'black','linewidth',3);
    line(ObjectCentroidX+t*sin(pi),ObjectCentroidY-t, 'Color', 'green','linewidth',3);  
    
    point1X = ObjectCentroidX - cos(ObjectOrientation)* ObjectLength/2.3;
    point1Y = ObjectCentroidY + sin(ObjectOrientation)* ObjectLength/2.3;
    point2X = ObjectCentroidX + cos(ObjectOrientation)* ObjectLength/2.3;
    point2Y = ObjectCentroidY - sin(ObjectOrientation)* ObjectLength/2.3;
    
    goal1X = ObjectCentroidX - cos(ObjectOrientation)* ObjectLength/3;
    goal1Y = ObjectCentroidY + sin(ObjectOrientation)* ObjectLength/3;
    goal2X = ObjectCentroidX + cos(ObjectOrientation)* ObjectLength/3;
    goal2Y = ObjectCentroidY - sin(ObjectOrientation)* ObjectLength/3;
    
    slope = (point2Y-point1Y)/(point2X-point1X);
    offset = point1Y - slope*point1X;
    if point1Y<point2Y
        topPointX = point1X;
        topPointY = point1Y;
        botPointX = point2X;
        botPointY = point2Y;
        topGoalX = goal1X;
        topGoalY = goal1Y;
        botGoalX = goal2X;
        botGoalY = goal2Y;
        
        version = 1;
    else
        botPointX = point1X;
        botPointY = point1Y;
        topPointX = point2X;
        topPointY = point2Y;
        botGoalX = goal1X;
        botGoalY = goal1Y;
        topGoalX = goal2X;
        topGoalY = goal2Y;
        
        version = 2;
    end
    
    ideal1X = ObjectCentroidX;
    ideal1Y = ObjectCentroidY + ObjectLength/2.3;
    ideal2X = ObjectCentroidX;
    ideal2Y = ObjectCentroidY - ObjectLength/2.3;
    
    if ideal1Y<ideal2Y
        topIdealX = ideal1X;
        topIdealY = ideal1Y;
        botIdealX = ideal2X;
        botIdealY = ideal2Y;
    else
        botIdealX = ideal1X;
        botIdealY = ideal1Y;
        topIdealX = ideal2X;
        topIdealY = ideal2Y;
    end
    
    plot(botIdealX,botIdealY,'v','Markersize',16,'color','green','linewidth',3);
    plot(topIdealX,topIdealY,'^','Markersize',16,'color','green','linewidth',3);
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
   
    [s, l] = size(centers);
    h = viscircles(centers,radii,'EdgeColor','b');
    %% Variance Control
    if s > 5
        minDis = 10000;
        again = false;    
        if (V > maxVar)
            VarCont = true;
            epsilon = bigEpsilon;
            for i = 1:size(corners)
                dist = dist2points(corners(i,1),corners(i,2),M(1)/scale,M(2)/scale);
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
            %% Flow Around the ends if at ends
            angdiffTop = ObjectOrientation+atan2((M(2)/scale - topPointY/scale),(M(1)/scale - topPointX/scale));
            angdiffBot = ObjectOrientation+atan2((M(2)/scale - botPointY/scale),(M(1)/scale - botPointX/scale));

            angdiffTop=AngleFix(angdiffTop,pi);
            angdiffBot=AngleFix(angdiffBot,pi);
            if (version == 2 && abs(angdiffTop)<pi/2)||(version == 1 && abs(angdiffTop)>pi/2)
                [currgoalX,currgoalY] = FlowForce(M(1)/scale,M(2)/scale,ObjectCentroidX/scale,ObjectCentroidY/scale,topPointX/scale,topPointY/scale);
                currgoalX=M(1)+currgoalX*scale;
                currgoalY=M(2)+currgoalY*scale;
                justOriented = 1; 
            elseif (version == 2 && abs(angdiffBot)>pi/2)||(version == 1 && abs(angdiffBot)<pi/2)
                [currgoalX,currgoalY] = FlowForce(M(1)/scale,M(2)/scale,ObjectCentroidX/scale,ObjectCentroidY/scale,botPointX/scale,botPointY/scale);
                currgoalX=M(1)+currgoalX*scale;
                currgoalY=M(2)+currgoalY*scale;
                justOriented = 1; 
            %% Push Back to 90 Degrees
            elseif ObjectOrientation>pi/2+(5*pi/180)||ObjectOrientation<pi/2-(5*pi/180)
                if (topIdealX>topPointX && M(1)<(M(2)-offset)/slope)||(topIdealX<topPointX && M(1)>(M(2)-offset)/slope)
                    currgoalX = topGoalX;
                    currgoalY = topGoalY;
                else
                    currgoalX = botGoalX;
                    currgoalY = botGoalY;
                end
                justOriented = 1;   
            else
                %% Back up if just Oriented
                if justOriented
                    currgoalY = ObjectCentroidY;
                    if M(1) < ObjectCentroidX
                        currgoalX = ObjectCentroidX - ObjectLength/3;
                    else
                        currgoalX = ObjectCentroidX + ObjectLength/3;
                    end
                    if (M(1) < currgoalX + epsilon && M(1) > currgoalX - epsilon) && (M(2) < currgoalY + epsilon && M(2) > currgoalY - epsilon)
                        justOriented = 0;
                    end
                %% Push Straight
                else
                    currgoalX = ObjectCentroidX;
                    currgoalY = ObjectCentroidY;
                end
            end
        end
        %% Draw Flow Around
        if flowDebug
            X = zeros(size(s));
            Y = zeros(size(s));
            DX = zeros(size(s));
            DY = zeros(size(s));
            for i = 1:floor(scaledPic(2))
                for j = 1:floor(scaledPic(1))
                    angdiffTopD = ObjectOrientation+atan2((j - topPointY/scale),(i - topPointX/scale));
                    angdiffBotD = ObjectOrientation+atan2((j - botPointY/scale),(i - botPointX/scale));
                    
                    angdiffTopD=AngleFix(angdiffTopD,pi);
                    angdiffBotD=AngleFix(angdiffBotD,pi);

                    if (version == 2 && abs(angdiffTopD)<pi/2)||(version == 1 && abs(angdiffTopD)>pi/2)
                        [DX(i,j),DY(i,j)] = FlowForce(i,j,ObjectCentroidX/scale,ObjectCentroidY/scale,topPointX/scale,topPointY/scale);

                        X(i,j) = i;
                        Y(i,j) = j;
                    elseif (version == 2 && abs(angdiffBotD)>pi/2)||(version == 1 && abs(angdiffBotD)<pi/2)
                        [DX(i,j),DY(i,j)] = FlowForce(i,j,ObjectCentroidX/scale,ObjectCentroidY/scale,botPointX/scale,botPointY/scale);

                        X(i,j) = i;
                        Y(i,j) = j;
                    end
                end
            end
            hq=quiver(X*scale,Y*scale,DX,DY,'color','cyan');%[0,0,0.5]); 
        end
        plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red', 'linewidth',3);
        plot(currgoalX , currgoalY,'*','Markersize',16,'color','cyan','linewidth',3);
        plot_gaussian_ellipsoid(M,C);
        newDot = [ObjectOrientation, toc(t0)];
        drawTime = [drawTime;newDot];
        frameCount = frameCount +1;
        hold off
        %% Turn on Lights
        if relay 
            %% Prioritize Y direction
            if M(1,2) > currgoalY + epsilon
                if M(1,1) > currgoalX + epsilon
                    Relay = 2;
                    relayOn(a,Relay);
                    pause(delayTime);
                elseif M(1,1) < currgoalX - epsilon
                    Relay = 4;
                    relayOn(a,Relay);
                    pause(delayTime);
                else
                    Relay = 3;
                    relayOn(a,Relay);
                    pause(delayTime);
                end
            elseif M(1,2)  < currgoalY - epsilon
                if M(1,1) > currgoalX + epsilon
                    Relay = 8;
                    relayOn(a,Relay);
                    pause(delayTime);
                elseif M(1,1) < currgoalX - epsilon
                    Relay = 6;
                    relayOn(a,Relay);
                    pause(delayTime);
                else
                    Relay = 7;
                    relayOn(a,Relay);
                    pause(delayTime);
                end
            elseif M(1,1) > currgoalX + epsilon
                Relay = 1;
                relayOn(a,Relay);
                pause(delayTime);
            elseif M(1,1) < currgoalX - epsilon
                Relay=5;
                relayOn(a,Relay);
                pause(delayTime);
            else       
                again = true;
                epsilon = smallEpsilon;
            end
%                %% Prioritize X direction
%                 if M(1,1) > currgoalX + epsilon
%                     if M(1,2) > currgoalY + epsilon
%                         Relay = 2;
%                         relayOn(a,Relay);
%                         pause(delayTime);
%                     elseif M(1,2) < currgoalY - epsilon
%                         Relay = 8;
%                         relayOn(a,Relay);
%                         pause(delayTime);
%                     else
%                         Relay = 1;
%                         relayOn(a,Relay);
%                         pause(delayTime);
%                     end
%                 elseif M(1,1)  < currgoalX - epsilon
%                     if M(1,2) > currgoalY + epsilon
%                         Relay = 4;
%                         relayOn(a,Relay);
%                         pause(delayTime);
%                     elseif M(1,2) < currgoalY - epsilon
%                         Relay = 6;
%                         relayOn(a,Relay);
%                         pause(delayTime);
%                     else
%                         Relay = 5;
%                         relayOn(a,Relay);
%                         pause(delayTime);
%                     end
%                 elseif M(1,2) > currgoalY + epsilon
%                     Relay = 3;
%                     relayOn(a,Relay);
%                     pause(delayTime);
%                 elseif M(1,2) < currgoalY - epsilon
%                     Relay=7;
%                     relayOn(a,Relay);
%                     pause(delayTime);
%                 else       
%                     again = true;
%                     epsilon = smallEpsilon;
%                 end
    end
end