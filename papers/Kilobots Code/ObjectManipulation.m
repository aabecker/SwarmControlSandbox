%%% Object Manipulation Experiment With Kilobots
%%% In this code we want to use arduino and our vision system to control
%%% kilobots for compeleting a block pushing experiment.
%%% By Shiva Shahrokhi Dec 2015, Jan 2016


%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
cam = webcam(2);


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

VarCont = false;

epsilon = 5;

goalX = 12;
goalY = 12;

success = false;
again = true;
% %%% First finding value iteration and creating the map:
% originalImage = snapshot(cam);
% img = imcrop(originalImage,[345 60 1110 850]);
% s = size(img);
 scale = 30;
% sizeOfMap = floor(s/scale);
% map = zeros(sizeOfMap(1),sizeOfMap(2));
% map(1,:) = 1;
% map(:,1) = 1;
% map(sizeOfMap(1),:) = 1;
% map(:,sizeOfMap(2)) = 1;
% for i= 1:sizeOfMap(1)-1
%     for j = 1:sizeOfMap(2)-1
%         for k = 0:scale-1
%             for l = 0:scale-1
%                 if img(i*scale+k, j*scale+l,1) > 180 && map(i,j) ~= 1 && img(i*scale+k, j*scale+l,2) < 80 && img(i*scale+k, j*scale+l,3)>100
%                     map(i,j) = 1;
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,1) = 0;  % Change the red value for the first pixel
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,2) = 0;    % Change the green value for the first pixel
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,3) = 255;    % Change the blue value for the first pixel
%                 end
%             end
%         end
%     end
% end
% imwrite(img,'new.jpeg');
% imshow(img);
% [probability, movesX, movesY] = MDPgridworldExampleBADWALLS(map,goalX,goalY);
% [X,Y] = meshgrid(1:size(map,2),1:size(map,1));
%  hold on; hq=quiver(X,Y,DY,DX,0.5,'color',[0,0,0]); hold off
% set(hq,'linewidth',2);
% Using Arduino for our lamps, this is how we define arduino in Matlab:
load('Map1', 'movesX', 'movesY','corners');
a = arduino('/dev/tty.usbmodem1421','uno');

figure
counter = 1;
while success == false

    if again== true
        writeDigitalPin(a, RELAY1,0);
        writeDigitalPin(a,RELAY2,0);
        writeDigitalPin(a,RELAY3,0);
        writeDigitalPin(a,RELAY4,0);
        writeDigitalPin(a, RELAY5,0);
        writeDigitalPin(a,RELAY6,0);
        writeDigitalPin(a,RELAY7,0);
        writeDigitalPin(a,RELAY8,0);
    pause (1);
    end 
% Read in a webcam snapshot.
rgbIm = snapshot(cam);
%crop to have just the table view.
originalImage = imcrop(rgbIm,[345 60 1110 850]);
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
plot(ObjectCentroidX , ObjectCentroidY,'*','Markersize',16,'color','cyan','linewidth',3);
BW(stat(index).PixelIdxList)=0;

  %threshold the image to remove shadows (and only show dark parts of kilobots)
    [centers, radii] = imfindcircles(BW,[10 19],'ObjectPolarity','bright','Sensitivity',0.92 );
    
    % %Mean
    M = mean(centers);
    %Variance
    V = var(centers)
    %Covariance
    C = cov(centers);

    imshow(originalImage);
    
    h = viscircles(centers,radii,'EdgeColor','b');
    [s, l] = size(centers);
    if s > 5 
        again = false;
    hold on
        minDis = 10000;
    corInd = 100;
    maxVar = 16000;
    minVar = 8000;
    %%%%Variance Control
    if V > maxVar
        VarCont = true;
        for i = 1:size(corners)
            dist = sqrt(sum((M - corners(i)) .^ 2));
            if minDis > dist
                minDis = dist;
                corInd = i;
            end   
        end
        currgoalX = corners(corInd,1)*scale;
        currgoalY = corners(corInd,2)*scale;
    else if V< minVar
            VarCont = false;
        end
    end
    if ~VarCont
     r = 0.1;
    indX = floor(M(1,1)/scale);
    indY = floor(M(1,2)/scale);
    indOX = floor(ObjectCentroidX/scale);
    indOY = floor(ObjectCentroidY/scale);
    
   if M(1,1) > ObjectCentroidX- r*scale+epsilon || M(1,1) < ObjectCentroidX-r*scale-epsilon
       if M(1,2) > ObjectCentroidY-r*scale+epsilon || M(1,2) < ObjectCentroidY-r*scale-epsilon
    currgoalX = ObjectCentroidX + r*scale * movesY(indOY,indOX);
    currgoalY = ObjectCentroidY + r*scale * movesX(indOY,indOX);
       end
   else
       currgoalX = M(1,1)+ movesY(indY,indX)*scale;
       currgoalY = M(1,2) + movesX(indY,indX)*scale;
   end
    end
    plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red', 'linewidth',3);
    plot(currgoalX , currgoalY,'*','Markersize',16,'color','cyan','linewidth',3);
    plot(goalX*scale , goalY*scale,'*','Markersize',16,'color','green','linewidth',3);
    %Current Mean and Covariance Ellipse
    plot_gaussian_ellipsoid(M,C);
    %M(counter) = getframe();
    counter = counter+1;
    hold off
    
    delayTime = 3;

    
    if M(1,1) > currgoalX+epsilon
        if M(1,2) > currgoalY + epsilon
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,0);
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a, RELAY5,1);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,1);
        pause(delayTime);
        else if M(1,2) < currgoalY - epsilon
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a, RELAY5,1);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,0);
        pause(delayTime);
            else
        writeDigitalPin(a, RELAY1,0);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a, RELAY5,1);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,1);
        pause(delayTime);
            end
        end
    else if M(1,1)  < currgoalX-epsilon
          
       if M(1,2) > currgoalY + epsilon
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a,RELAY4,0);
        writeDigitalPin(a, RELAY5,1);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,1);
        pause(delayTime);
        else if M(1,2) < currgoalY - epsilon
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a, RELAY5,1);
        writeDigitalPin(a,RELAY6,0);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,1);
        pause(delayTime);
            else
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a, RELAY5,0);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,1);
        pause(delayTime);
            end
        end
       
        else if M(1,2) > currgoalY+epsilon
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,0);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a, RELAY5,1);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,1);
        pause(delayTime);
       
            else if M(1,2) < currgoalY-epsilon
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a, RELAY5,1);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,0);
        writeDigitalPin(a,RELAY8,1);
                pause(delayTime);
                else       
        writeDigitalPin(a, RELAY1,0);
        writeDigitalPin(a,RELAY2,0);
        writeDigitalPin(a,RELAY3,0);
        writeDigitalPin(a,RELAY4,0);
        writeDigitalPin(a, RELAY5,0);
        writeDigitalPin(a,RELAY6,0);
        writeDigitalPin(a,RELAY7,0);
        writeDigitalPin(a,RELAY8,0);
                    pause(delayTime);
                    again = true;
                end
            end
        end
    end
    end
end
    
