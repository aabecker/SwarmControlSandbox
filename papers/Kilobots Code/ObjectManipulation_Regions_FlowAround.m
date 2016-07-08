%%% Object Manipulation Experiment With Kilobots
%%% In this code we want to use arduino and our vision system to control
%%% kilobots for compeleting a block pushing experiment.
%%% By Shiva Shahrokhi, Mable Wan and Lillian Lin Summer 2016

clear all
%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
cam = webcam(1);
Relay=0;

VarCont = false;
flowDebug = true;

goalX = 6;
goalY = 5;

success = false;
again = true;
% %%% First finding value iteration and creating the map:
% originalImage = snapshot(cam);
% img = imcrop(originalImage,[345 60 1110 850]);

% imwrite(img,'new.jpeg');
% imshow(img);
% [probability, movesX, movesY] = MDPgridworldExampleBADWALLS(map,goalX,goalY);
% [X,Y] = meshgrid(1:size(map,2),1:size(map,1));
%  hold on; hq=quiver(X,Y,DY,DX,0.5,'color',[0,0,0]); hold off
% set(hq,'linewidth',2);
% Using Arduino for our lamps, this is how we define arduino in Matlab:
load('Map3', 'movesX', 'movesY','corners');
load('ThresholdMaps','transferRegion','mainRegion');
% figure(1),imshow(transferRegion(:,:,1));
% figure(2),imshow(transferRegion(:,:,2));
% figure(3),imshow(mainRegion);
if (ispc==1)  
    a = arduino('Com4','uno');
else 
    a = arduino('/dev/tty.usbmodem1421','uno');
end 
%a = arduino('/dev/tty.usbmodem1421','uno');
%initialize mean controller 
%meanControlCount=0;
%delayTime = 7;

% figure
counter = 1;
c = 0;
meanControl=false;

%Flow Around Variables
eta=50;
zeta=1;
rhoNot=7.5;
alphaWant=0;

%1 is main regions, 0 is transfer regions
regionID=1;
regionNum=3;
%load regions
currentRegionMap=mainRegion(:,:,regionNum); %init map

while success == false
   
if (again == true)
    relayOn(a,0);
    pause (10);
end 
% Read in a webcam snapshot.
% rgbIm = snapshot(cam);
% pause(3);
rgbIm = snapshot(cam);
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
% map = zeros(sizeOfMap(1),sizeOfMap(2));
% map(1,:) = 1;
% map(:,1) = 1;
% map(sizeOfMap(1),:) = 1;
% map(:,sizeOfMap(2)) = 1;
% for i= 0:sizeOfMap(1)-1
%     for j = 0:sizeOfMap(2)-1
%         if (mainRegion(i+1,j+1,3)==1)
%             for k = 1:scale
%                 for l = 1:scale
%                     originalImage(i*scale+k,j*scale+l,1) = 0;  % Change the red value for the first pixel
%                     originalImage(i*scale+k,j*scale+l,2) = 0;    % Change the green value for the first pixel
%                     originalImage(i*scale+k,j*scale+l,3) = 255;    % Change the blue value for the first pixel
%                 end
%             end
%         end
%     end
% end
% imshow(originalImage);
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
hold on
plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','black','linewidth',3);
hold off
% originalImage(i*scale:i*scale+scale,j*scale:j*scale+scale,1) = 0;  % Change the red value for the first pixel
% originalImage(i*scale:i*scale+scale,j*scale:j*scale+scale,2) = 0;    % Change the green value for the first pixel
% originalImage(i*scale:i*scale+scale,j*scale:j*scale+scale,3) = 255;

ObjCentX=floor(ObjectCentroidX/scale);
ObjCentY=floor(ObjectCentroidY/scale);
if (ObjCentX==0)
    ObjCentX=1;
end
if (ObjCentY==0)
    ObjCentY=1;
end

position = currentRegionMap(ObjCentY,ObjCentX);
%switching regions
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

%plot(corners,'*','Markersize',16,'color','green','linewidth',3); 
BW(stat(index).PixelIdxList)=0;

  %threshold the image to remove shadows (and only show dark parts of kilobots)
  if ispc
     [centers, radii] = imfindcircles(BW,[4 6],'ObjectPolarity','bright','Sensitivity',0.97); 
  else
    [centers, radii] = imfindcircles(BW,[10 19],'ObjectPolarity','bright','Sensitivity',0.92 );
  end
%   %finding robots in cell
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
    %cou=countMean %debug
    % %Mean
   M(1,1)= sumX/countMean;
  M(1,2)= sumY/countMean;
%  M = mean(centers);
    %Variance
    V = var(cenArray);
    %Covariance
    C = cov(cenArray);
%C = cov(centers);
    
    h = viscircles(centers,radii,'EdgeColor','b');
    [s, l] = size(centers);
    if s < 85
%         DateString = datestr(datetime);
%         %timeString=datestr(datetime, 'HH.MM.SS');
%         name = 'fail';
%         
%         %fullName = strcat(name,DateString,timeString,'.jpeg' );
%         fullName = strcat(name,DateString,'.jpeg' );
%         imwrite(originalImage,fullName);
    end
    if s > 5 
        again = false;
    hold on
    minDis = 10000;
    corInd = 0;
    maxVar = 12000; %was 16000
    minVar = 10000; %was 12000
        
    
    %%%%Variance Control
   % if ((V > maxVar) |(meanControlCount>3))
%    for i = 1:size(corners)
%             %dist = sqrt(sum((M - corners(i)) .^ 2));
%             %dist = sqrt(sum((centroids(index) - corners(i)) .^ 2));
%             dist = sqrt((ObjectCentroidX/scale - corners(i,1)) * (ObjectCentroidX/scale - corners(i,1)) + (ObjectCentroidY/scale- corners(i,2)) * (ObjectCentroidY/scale- corners(i,2)));
%             
%             if minDis > dist
%                 minDis = dist;
%                 corInd = i;
%             end   
%    end

if (countMean<10)
        NumRobotCont = true;
        for i = 1:size(corners)
            %dist = sqrt(sum((M - corners(i)) .^ 2));
            %dist = sqrt(sum((centroids(index) - corners(i)) .^ 2));
            dist = sqrt((ObjectCentroidX/scale - corners(i,1)) * (ObjectCentroidX/scale - corners(i,1)) + (ObjectCentroidY/scale- corners(i,2)) * (ObjectCentroidY/scale- corners(i,2)));
            
            if minDis > dist
                minDis = dist;
                corInd = i;
            end   
        end
        currgoalX = (corners(corInd,1))*scale;
        currgoalY = (corners(corInd,2))*scale;
    else if countMean> 20
            NumRobotCont = false;
        end
    end

   if (V > maxVar)
        VarCont = true;
        for i = 1:size(corners)
            %dist = sqrt(sum((M - corners(i)) .^ 2));
            %dist = sqrt(sum((centroids(index) - corners(i)) .^ 2));
            dist = sqrt((ObjectCentroidX/scale - corners(i,1)) * (ObjectCentroidX/scale - corners(i,1)) + (ObjectCentroidY/scale- corners(i,2)) * (ObjectCentroidY/scale- corners(i,2)));
            
            if minDis > dist
                minDis = dist;
                corInd = i;
            end   
        end
        currgoalX = (corners(corInd,1))*scale;
        currgoalY = (corners(corInd,2))*scale;
    else if V< minVar
            VarCont = false;
        end
    end
if ~VarCont & ~NumRobotCont
     %r = 0; %was 0.1
     
      ep = 5;
     minDistance =  5*scale;
     
    indX = floor(M(1,1)/scale);
    indY = floor(M(1,2)/scale);
    
    indOX = floor(ObjectCentroidX/scale);
    indOY = floor(ObjectCentroidY/scale);
    if indOY ==1
        indOY = 2;
    end
     
%     if M(1,1) > ObjectCentroidX- minDistance || M(1,1) < ObjectCentroidX-minDistance
%        if M(1,2) > ObjectCentroidY-minDistance || M(1,2) < ObjectCentroidY-minDistance
%            r = 2.5
%        end
%     else
%         if M(1,2) > ObjectCentroidY-minDistance || M(1,2) < ObjectCentroidY-minDistance
%            r = 2.5;
%         else
%             r = 0.1
%         end
%     end
%% Flow Around Goal Algorithm
    r = 2.5;
    alphaWant=atan2(movesX(indOY,indOX),movesY(indOY,indOX));
    repPointX=ObjectCentroidX/scale;% - r * cos(alphaWant+pi);
    repPointY=ObjectCentroidY/scale;% - r * sin(alphaWant+pi);
    theta = atan2((M(1,2)/scale - repPointY),(M(1,1)/scale - repPointX));
    angdiff = alphaWant-theta;
    rho=sqrt((M(1,1)/scale-repPointX)^2 + (M(1,2)/scale-repPointY)^2);
    if(angdiff > pi) 
        angdiff = angdiff - 2*pi;
    end
    if(angdiff < -pi) 
        angdiff = angdiff + 2*pi;
    end
    
    if ((rho<rhoNot) && (abs(angdiff)<pi*5/8))
        epsilon = 0.2 * scale;
        disp('In Flow Around Goal Type')
        FrepX=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(repPointX-M(1,1)/scale);
        FrepY=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(repPointY-M(1,2)/scale);
        
        attPointX=ObjectCentroidX/scale - r * cos(alphaWant);
        attPointY=ObjectCentroidY/scale - r * sin(alphaWant);
        rho=sqrt((M(1,1)/scale-attPointX)^2 + (M(1,2)/scale-attPointY)^2);
        FattX=zeta*(M(1,1)/scale-attPointX)/rho;
        FattY=zeta*(M(1,2)/scale-attPointY)/rho;
        
         currgoalX=M(1,1)/scale*scale+cos(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;
         currgoalY=M(1,2)/scale*scale+sin(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;
        
%         FrepX=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(repPointX-M(1,1)/scale);
%         FrepY=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(repPointY-M(1,2)/scale);
%         
%         attPointX=ObjectCentroidX/scale - r * cos(alphaWant+pi);
%         attPointY=ObjectCentroidY/scale - r * sin(alphaWant+pi);
%         rho=sqrt((M(1,1)/scale-attPointX)^2 + (M(1,2)/scale-attPointY)^2);
%         FattX=0; %zeta*(M(1,1)/scale-attPointX)/rho;
%         FattY=0; %zeta*(M(1,2)/scale-attPointY)/rho;
%         
%          currgoalX=M(1,1)+cos(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;
%          currgoalY=M(1,2)+sin(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;

         lineLength = 1000;
angle = atan2((-FrepY-FattY),(-FattX-FrepX));
x(1) = M(1,1)/scale*scale;
y(1) = M(1,2)/scale*scale;
x(2) = x(1) + lineLength * cos(angle);
y(2) = y(1) + lineLength * sin(angle);
hold on; % Don't blow away the image.
plot(x, y);

%         currgoalX=M(1,1)+(-FattY-FrepY)/sqrt((-FrepX-FattX)^2 + (-FrepY-FattY)^2)*scale;
%         currgoalY=M(1,2)+(-FattX-FrepX)/sqrt((-FrepX-FattX)^2 + (-FrepY-FattY)^2)*scale;
    else 
%% Old Goal Algorithm 
epsilon = 1*scale;
        if M(1,1) > ObjectCentroidX- minDistance && M(1,1) < ObjectCentroidX+minDistance && M(1,2) < ObjectCentroidY+minDistance && M(1,2) > ObjectCentroidY-minDistance
            r = 0.1;
     %else if M(1,1) > ObjectCentroidX- (corners(corInd,1))*scale && M(1,1) < ObjectCentroidX+minDistance && M(1,2) < ObjectCentroidY+minDistance && M(1,2) > ObjectCentroidY-minDistance
     %        r = 2.5;
        else
            r = 2.5;
        end

        
        
        currgoalX = ObjectCentroidX - r*scale * movesY(indOY,indOX);
        currgoalY = ObjectCentroidY - r*scale * movesX(indOY,indOX);
%         if M(1,1) > ObjectCentroidX- r*scale+ep || M(1,1) < ObjectCentroidX-r*scale-ep
%            if M(1,2) > ObjectCentroidY-r*scale+ep || M(1,2) < ObjectCentroidY-r*scale-ep
%         currgoalX = ObjectCentroidX - r*scale * movesY(indOY,indOX);
%         currgoalY = ObjectCentroidY - r*scale * movesX(indOY,indOX);
%         tada = 0
%         else
%            currgoalX = ObjectCentroidX - r*scale * movesY(indOY,indOX);
%             currgoalY = ObjectCentroidY - r*scale * movesX(indOY,indOX);
%            something=7
%            end
%         else
%            currgoalX = M(1,1)+ movesY(indY,indX)*scale;
%            currgoalY = M(1,2) + movesX(indY,indX)*scale;
%            tada = 1
%         end
    end
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
            if(angdiffD > pi) 
        angdiffD = angdiffD - 2*pi;
            end
    if(angdiffD < -pi) 
        angdiffD = angdiffD + 2*pi;
    end
    if ((rhoD<rhoNot) && (abs(angdiffD)<pi*5/8))
        FrepX=eta*((rhoD^(-1))-(rhoNot^(-1)))*(rhoD^(-1))^2*(repPointX-i);
        FrepY=eta*((rhoD^(-1))-(rhoNot^(-1)))*(rhoD^(-1))^2*(repPointY-j);
        
        attPointX=ObjectCentroidX/scale - r * cos(alphaWant);
        attPointY=ObjectCentroidY/scale - r * sin(alphaWant);
        rhoD=sqrt((i-attPointX)^2 + (j-attPointY)^2);
        FattX=zeta*(i-attPointX)/rhoD;
        FattY=zeta*(j-attPointY)/rhoD;
        
        plot(attPointX *scale, attPointY*scale,'*','Markersize',16,'color','blue','linewidth',3);
        plot(repPointX*scale , repPointY*scale,'*','Markersize',16,'color','black','linewidth',3);
        X(i,j) = i;
        Y(i,j) = j;
        DX(i,j)=cos(atan2((-FrepY-FattY),(-FattX-FrepX)));
        DY(i,j)=sin(atan2((-FrepY-FattY),(-FattX-FrepX)));
            end
        end
    end
     hq=quiver(X*scale,Y*scale,DX,DY,'color','cyan');%[0,0,0.5]); 
    end
%     set(hq,'linewidth',2);
%         drawnow
       
       
end
    
    plot(M(1,1),M(1,2),'*','Markersize',16,'color','red', 'linewidth',0.5);
    plot(currgoalX , currgoalY,'*','Markersize',16,'color','yellow','linewidth',0.5);
    plot(goalX*scale , goalY*scale,'*','Markersize',16,'color','green','linewidth',3);
    plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','black','linewidth',3);
    circle(goalX*scale, goalY*scale,4*scale);
    for i = 1:size(corners)
        txt = int2str(i);
        text(corners(i,1)* scale,corners(i,2)*scale,txt,'HorizontalAlignment','right')
        %plot( corners(i,1)* scale, corners(i,2)*scale,'*','Markersize',16,'color','red','linewidth',3);
    end
    %Current Mean and Covariance Ellipse
    plot_gaussian_ellipsoid(M,C);
    %M(counter) = getframe();
    counter = counter+1;
    
    switch Relay
        case 0
            title('All Relays on')
        otherwise
            str=sprintf('relay %d',Relay);
            title(str)
    end
    
    hold off
    if (meanControl)
        delayTime=10; %was 42
        meanControl=false;
    else 
        delayTime=10; %was 14
    end
    %Mean Control activates Variance Control when the mean and goal are on
    %top of each other for more than 5 times 
    
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
       
%                 else       
%         %relayOn(a,Relay);
%         meanControl=true;
%         %VarCont = true;
%         %pause(delayTime);
%         
%         again = true;
                end
            end
        end
    end
    end

end

