%% Object Manipulation Experiment With Kilobots
% In this code we use arduino and our vision system to control a swarm of
% kilobots to push an object through a maze.
% By Shiva Shahrokhi, Lillian Lin and Mable Wan, Summer 2016

close all
clear all

%% Load maps from RegionCode.m
load('MDPShot', 'movesX', 'movesY','corners');
load('ThresholdMapsMac','transferRegion','mainRegion'); 


%% Define webcam
webcamShot = true;

if webcamShot
    cam = webcam(1);
    %% Using Arduino for our lights, this is how we define arduino in Matlab:
    if (ispc==1)  
        a = arduino('Com5','uno');
    else 
        a = arduino('/dev/tty.usbmodem1421','uno');
    end 
    %% Setup End Goal Position
    if (ispc==1)  
        goalX = 5;
        goalY = 5;
        goalSize = 4;
    else 
        goalX = 5;
        goalY = 5;
        goalSize = 3.5;
    end 
end
%% Initalize Variables
Relay=0;
delayTime = 10;
VarCont = false;
flowDebug = true;
success = false;
again = true;
counter = 1;
c = 0;
%Flow Around Variables
rhoNot=7.5;
%1 is main regions, 0 is transfer regions
regionID=1;
regionNum=3;
%load regions
currentRegionMap=mainRegion(:,:,regionNum); %init map

while success == false  
    %% Read in a webcam snapshot.
    
    if webcamShot
        if (again == true)
            relayOn(a,0);
            pause (10);
        end 
        rgbIm = snapshot(cam);
    else
        rgbIm = imread('PC.jpeg');
        goalX = 5;
        goalY = 5;
        goalSize = 4;
    end
%% crop to have just the table view.
    if (ispc== 1)  
        originalImage = imcrop(rgbIm,[50 10 500 400]);
    else 
        originalImage = imcrop(rgbIm,[345 60 1110 850]);
    end 
    s = size(originalImage);
    scale = floor( size(originalImage,2)/size(mainRegion,2));
    epsilon = 1* scale;
    sizeOfMap = floor(s/scale);
    imshow(originalImage);

%% make HSV scale for robots and object
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
    

    [B,L] = bwboundaries(BW, 'noholes');

    stat = regionprops(L,'Centroid','Area','PixelIdxList','MajorAxisLength','MinorAxisLength');
%% Find the object and put that out of the image
    [maxValue,index] = max([stat.Area]);
    centroids = cat(1, stat.Centroid);
    lengthsMajor = cat(1,stat.MajorAxisLength);
    lengthsMinor = cat(1,stat.MinorAxisLength);

    ObjectRadius = mean([lengthsMajor(index) lengthsMinor(index)],2)/2;
    ObjectCentroidX = centroids(index,1);
    ObjectCentroidY = centroids(index,2);
    BW(stat(index).PixelIdxList)=0;

    ObjCentX=floor(ObjectCentroidX/scale);
    ObjCentY=floor(ObjectCentroidY/scale);
    if (ObjCentX==0)
        ObjCentX=1;
    end
    if (ObjCentY==0)
        ObjCentY=1;
    end

    position = currentRegionMap(ObjCentY,ObjCentX);
%% switching regions
    if(position==0)
        if (regionID==1)%check if it's in mainRegion state
            regionID=0; %changes to transferRegion state
            for i = 1:size(transferRegion,3)%find which trasnferRegion centroid is in
                tempMap=transferRegion(:,:,i);
                if(tempMap(ObjCentY,ObjCentX)==1)
                    regionNum=i; %set to new region
                    currentRegionMap=transferRegion(:,:,i);
                end
            end
        elseif (regionID==0) %if it is in transferRegion state
            regionID=1; %changes to mainRegion state
            for i = 1:size(mainRegion,3)%find which region centroid is in
                tempMap=mainRegion(:,:,i);
                if(tempMap(ObjCentY,ObjCentX)==1)
                    regionNum=i; %set to new region
                    currentRegionMap=mainRegion(:,:,i);
                end
            end
        end
    end

%% Finding the green circles on the image which are kilobots.
    if ispc
        [centers, radii] = imfindcircles(BW,[4 6],'ObjectPolarity','bright','Sensitivity',0.97); 
    else
        [centers, radii] = imfindcircles(BW,[10 19],'ObjectPolarity','bright','Sensitivity',0.92 );
    end
%% finding robots in cell
    sumX=0;
    sumY=0;
    countMean=0; %number of robots in cell
    [m,n]=size(centers);
    cenArray=[]; %dynamic array for selecting only robots within region
    for i=1:m
        cenX=floor(centers(i,1)/scale);
        cenY=floor(centers(i,2)/scale);
        if (cenX==0) 
            cenX=1;
        end 
        if (cenY==0) 
            cenY=1;
        end  
        if(currentRegionMap(cenY,cenX)==1) %if robot is in region
            sumX= sumX+centers(i,1);
            sumY= sumY+centers(i,2);
            countMean=countMean+1;
            cenArray(countMean,1)= centers(i,1); %adding selected robots to array
            cenArray(countMean,2)= centers(i,2);
        end
    end
    %% Robot Swarm Charactaristics
    %Mean
    M(1,1)= sumX/countMean;
    M(1,2)= sumY/countMean;
    %Variance
    V = var(cenArray);
    %Covariance
    C = cov(cenArray);
    % Drawing robots
    h = viscircles(centers,radii,'EdgeColor','b');
    %s is number of detected robots
    [s, l] = size(centers);
    if s > 5 
        again = false;
        hold on
        % Variance Control constants
        minDis = 10000;
        corInd = 0;
        maxVar = 12000; %was 16000
        minVar = 10000; %was 12000  
        %% Current region should at least have a minimum number of robots
        minNumRobD = 10;
        minNumRobU = 20;
        if countMean<minNumRob
            NumRobotCont = true;
            % finding the nearest corner
            for i = 1:size(corners)
                dist = dist2points(corners(i,1),corners(i,2),ObjectCentroidX/scale,ObjectCentroidY/scale);
                if minDis > dist
                    minDis = dist;
                    corInd = i;
                end   
            end
            currgoalX = (corners(corInd,1))*scale;
            currgoalY = (corners(corInd,2))*scale;
        elseif countMean> minNumRobU
            NumRobotCont = false;
        end
%% Variance Control
        if (V > maxVar)
            VarCont = true;
            for i = 1:size(corners)
                dist = dist2points(corners(i,1),corners(i,2),ObjectCentroidX/scale,ObjectCentroidY/scale);
                if minDis > dist
                    minDis = dist;
                    corInd = i;
                end   
            end
            currgoalX = (corners(corInd,1))*scale;
            currgoalY = (corners(corInd,2))*scale;
        elseif V< minVar
            VarCont = false;
        end
        %% normal control
        if ~VarCont & ~NumRobotCont
            ep = 5;
            minDistance =  5*scale;

            indX = floor(M(1,1)/scale);
            indY = floor(M(1,2)/scale);

            indOX = floor(ObjectCentroidX/scale);
            indOY = floor(ObjectCentroidY/scale);
            if indOY ==1 || indOY == 0
                indOY = 2;
            end

    % Flow Around Goal Algorithm
            alphaWant=atan2(movesX(indOY,indOX),movesY(indOY,indOX));
            
            attPointX=ObjectCentroidX/scale - ObjectRadius/scale * cos(alphaWant);
            attPointY=ObjectCentroidY/scale - ObjectRadius/scale * sin(alphaWant);
            repPointX=ObjectCentroidX/scale;
            repPointY=ObjectCentroidY/scale;

            theta = atan2((M(1,2)/scale - repPointY),(M(1,1)/scale - repPointX));
            angdiff = alphaWant-theta;
            rho=dist2points(repPointX,repPointY,M(1,1)/scale,M(1,2)/scale);
            angdiff=AngleFix(angdiff,pi);

            if ((rho<rhoNot) && (abs(angdiff)<pi*4/8))
                epsilon = 0.2 * scale;

                [ currgoalX,currgoalY ] = FlowForce(M(1,1)/scale,M(1,2)/scale,attPointX,attPointY,repPointX,repPointY) ;
                currgoalX=M(1,1)+currgoalX*scale;
                currgoalY=M(1,2)+currgoalY*scale;
            else 
            % Old Goal Algorithm 
                epsilon = 1*scale;
                if M(1,1) > ObjectCentroidX- minDistance && M(1,1) < ObjectCentroidX+minDistance && M(1,2) < ObjectCentroidY+minDistance && M(1,2) > ObjectCentroidY-minDistance
                    r = 0.1;
                else
                    r = 2.5;
                end
                currgoalX = ObjectCentroidX - r*scale * movesY(indOY,indOX);
                currgoalY = ObjectCentroidY - r*scale * movesX(indOY,indOX);
            end
            hold on; % Don't blow away the image.
            if flowDebug
                s = size(movesX);
                X = zeros(size(movesX));
                Y = zeros(size(movesX));
                DX = zeros(size(movesX));
                DY = zeros(size(movesX));
                for i = 1:s(2)
                    for j = 1:s(1)
                        thetaD = atan2(j - repPointY,i - repPointX);
                        angdiffD = alphaWant-thetaD;
                        rhoD=sqrt((i-repPointX)^2 + (j-repPointY)^2);
                        angdiffD=AngleFix(angdiffD,pi);
                        if ((rhoD<rhoNot) && (abs(angdiffD)<pi*4/8))
                            [ DX(i,j),DY(i,j) ] = FlowForce(i,j,attPointX,attPointY,repPointX,repPointY);

                            X(i,j) = i;
                            Y(i,j) = j;

                            DX(i,j)=DX(i,j);
                            DY(i,j)=DY(i,j);
                        end
                    end
                end
                hq=quiver(X*scale,Y*scale,DX,DY,'color',[0 0.8 1]);%[0,0,0.5]); 
            end
        end
%% Draw everything
        plot(attPointX*scale, attPointY*scale,'o','Markersize',16,'color','blue','linewidth',3);
        plot(repPointX*scale, repPointY*scale,'o','Markersize',16,'color','cyan','linewidth',3);
        plot(M(1,1),M(1,2),'*','Markersize',16,'color','red', 'linewidth',0.5);
        plot(currgoalX , currgoalY,'*','Markersize',16,'color','yellow','linewidth',0.5);
        plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','cyan','linewidth',3);
        circle(goalX*scale, goalY*scale,goalSize*scale);
        circle(ObjectCentroidX, ObjectCentroidY,ObjectRadius);
%% show the corners
        for i = 1:size(corners)
            txt = int2str(i);
            text(corners(i,1)* scale,corners(i,2)*scale,txt,'HorizontalAlignment','right')
        end
%%Current Mean and Covariance Ellipse
        plot_gaussian_ellipsoid(M,C);
        counter = counter+1;
        
        %% Turn on Lights
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
        elseif M(1,1) < currgoalX-epsilon   
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
        end
    end
end