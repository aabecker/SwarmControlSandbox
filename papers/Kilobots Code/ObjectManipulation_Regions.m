%%% Object Manipulation Experiment With Kilobots
%%% In this code we want to use arduino and our vision system to control
%%% kilobots for compeleting a block pushing experiment.
%%% By Shiva Shahrokhi Dec 2015, Jan 2016

clear all
%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
cam = webcam(1);



Relay=0;

VarCont = false;



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
load('Map2', 'movesX', 'movesY','corners');
load('ThresholdMaps','transferRegion','mainRegion');
% figure(1),imshow(transferRegion(:,:,1));
% figure(2),imshow(transferRegion(:,:,2));
% figure(3),imshow(mainRegion);
if (ispc)  
    a = arduino('Com4','uno');
else 
    a = arduino('/dev/tty.usbmodem1421','uno');
end 

%initialize mean controller 
%meanControlCount=0;
%delayTime = 7;


% figure
counter = 1;
c = 0;
meanControl=false;
 
%1 is main regions, 0 is transfer regions
regionID=1;
regionNum=3;
%load regions
currentRegionMap=mainRegion(:,:,regionNum); %init map
imshow(mainRegion)

while success == false
   
if again== true
    relayOn(a,0);
pause (10);
end 
% Read in a webcam snapshot.
rgbIm = snapshot(cam);
%crop to have just the table view.
if (ispc)  
    originalImage = imcrop(rgbIm,[50 10 500 400]);
else 
    originalImage = imcrop(rgbIm,[345 60 1110 850]);
end 
 s = size(originalImage);
 scale = 30;
 epsilon = 1* scale;
 sizeOfMap = floor(s/scale);
% map = zeros(sizeOfMap(1),sizeOfMap(2));
% map(1,:) = 1;
% map(:,1) = 1;
% map(sizeOfMap(1),:) = 1;
% map(:,sizeOfMap(2)) = 1;
imshow(originalImage);
hold on
for i= 1:sizeOfMap(1)-1
    for j = 1:sizeOfMap(2)-1
        for k = 0:scale-1
            for l = 0:scale-1
                if (mainRegion(i,j,1)==1)
                    originalImage(i*scale:i*scale+scale,j*scale:j*scale+scale,1) = 0;  % Change the red value for the first pixel
                    originalImage(i*scale:i*scale+scale,j*scale:j*scale+scale,2) = 0;    % Change the green value for the first pixel
                    originalImage(i*scale:i*scale+scale,j*scale:j*scale+scale,3) = 255;    % Change the blue value for the first pixel
                end
            end
        end
    end
end
%figure(2),
imshow(originalImage);
%figure
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


regionID
regionNum


comXInd = floor (ObjectCentroidX/scale)
comYInd = floor(ObjectCentroidY/scale)
position = currentRegionMap(comXInd,comYInd)
%switching regions
if (regionID)%check if it's in mainRegion state
    if (currentRegionMap(comXInd,comYInd)==0)
        regionID=0 %changes to transferRegion state
        for i = 1:size(transferRegion)%find which region centroid is in
            tempMap=transferRegion(:,:,i);
            if(tempMap(comXInd,comYInd)==1)
                regionNum=i; %set to new region
                currentRegionMap=transferRegion(:,:,i);
            end
            break;
        end
    end
end
if (regionID==0) %if it is in transferRegion state
    if (currentRegionMap(comXInd,comYInd)==0)
        for i = 1:size(mainRegion)%find which region centroid is in
            tempMap=mainRegion(:,:,i);
            if(tempMap(comXInd,comYInd)==1)
                regionNum=i; %set to new region
                regionID=1; %changes to mainRegion state
                currentRegionMap=mainRegion(:,:,i);
            end
            break;
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
%   sumX=0;
%   sumY=0;
%   countMean=0; %number of robots in cell
%   [m,n]=size(centers);
%   for i=1:m
%       if(currentMap(centers(i))) %if robot is in region
%         sumX= sumX+center(i,1);
%         sumY= sumY+center(i,2);
%         count=count+1;
%       end
%   end
  
    % %Mean
%     M(1,1)= sumX/countMean;
%     M(1,2)= sumY/countMean;
 M = mean(centers);
    %Variance
    V = var(centers);
    %Covariance
    C = cov(centers);

%      imshow(originalImage);
    
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
    minVar = 11000; %was 12000
        
    
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
   if (V > maxVar | VarCont)
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
    if ~VarCont
     r = 0; %was 0.1
      ep = 5;
     minDistance =  5*scale;
     
    indX = floor(M(1,1)/scale);
    indY = floor(M(1,2)/scale);
    indOX = floor(ObjectCentroidX/scale);
    indOY = floor(ObjectCentroidY/scale);
     if M(1,1) > ObjectCentroidX- minDistance && M(1,1) < ObjectCentroidX+minDistance && M(1,2) < ObjectCentroidY+minDistance && M(1,2) > ObjectCentroidY-minDistance
         r = 0.1;
     %else if M(1,1) > ObjectCentroidX- (corners(corInd,1))*scale && M(1,1) < ObjectCentroidX+minDistance && M(1,2) < ObjectCentroidY+minDistance && M(1,2) > ObjectCentroidY-minDistance
     %        r = 2.5;
     else
             r = 2.5;
         
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

   if M(1,1) > ObjectCentroidX- r*scale+ep || M(1,1) < ObjectCentroidX-r*scale-ep
       if M(1,2) > ObjectCentroidY-r*scale+ep || M(1,2) < ObjectCentroidY-r*scale-ep
    currgoalX = ObjectCentroidX - r*scale * movesY(indOY,indOX);
    currgoalY = ObjectCentroidY - r*scale * movesX(indOY,indOX);
    tada = false;
       end
   else
       currgoalX = M(1,1)+ movesY(indY,indX)*scale;
       currgoalY = M(1,2) + movesX(indY,indX)*scale;
       tada = true;
   end
    end
    plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red', 'linewidth',3);
    plot(currgoalX , currgoalY,'*','Markersize',16,'color','cyan','linewidth',3);
    plot(goalX*scale , goalY*scale,'*','Markersize',16,'color','green','linewidth',3);
    plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','cyan','linewidth',3);
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
       
                else       
        %relayOn(a,Relay);
        meanControl=true;
        VarCont = true;
        %pause(delayTime);
        
        again = true;
                end
            end
        end
    end
    end

end

