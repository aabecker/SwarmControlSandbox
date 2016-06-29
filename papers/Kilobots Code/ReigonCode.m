%%%%%%%%% By Shiva Shahrokhi June 2016: This code takes a snapshot of the
%%%%%%%%% webcam, then process it to find obstacles, gives that map to MDP
%%%%%%%%% and gets the result and draw the gradients and regions. 
success = false;
webcamShot = true;
obstacles = [];
if webcamShot
    cam = webcam(1);

end
t0 = tic;
results3= [];
%while success == false
if webcamShot
 originalImage = snapshot(cam);
     if (ispc)  
         original = imcrop(originalImage,[50 10 500 400]);
         img = imcrop(originalImage,[50 10 500 400]);
         rgbIm = imcrop(originalImage,[50 10 500 400]);
    else 
        original = imcrop(originalImage,[345 60 1110 860]);
         img = imcrop(originalImage,[345 60 1110 860]);
         rgbIm = imcrop(originalImage,[345 60 1110 860]);
     end 
else 
    rgbIm = imread('Obstacle.jpeg');
end


    I = rgb2hsv(rgbIm);
    
% % Define thresholds for channel 1 based on histogram settings
% channel1Min = 0.902;
% channel1Max = 0.938;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min = 0.205;
% channel2Max = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min = 0.795;
% channel3Max = 1.000;
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.866;
channel1Max = 0.950;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.353;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.802;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
%finding Object Orientation
    [B,L] = bwboundaries(BW, 'noholes');
    stat = regionprops(L,'Centroid','Orientation','MajorAxisLength', 'Area');
    area = cat(1,stat.Area);
    centroids = cat(1, stat.Centroid);
    orientations = cat(1, stat.Orientation);
    majorLength = cat(1, stat.MajorAxisLength);
    imshow(rgbIm);
    hold on
    for i = 1: size(area)
       if area(i) > 780
           obstacles = [obstacles; i];
       end
    end
    
    xlim = get(gca,'XLim');
    ylim = get(gca,'YLim');
    slope=zeros(size(obstacles));
    offset=zeros(size(obstacles));
    Tangent_offset=zeros(size(obstacles));
    
   for i = 1: size(obstacles)
       hold on
       plot(centroids(obstacles(i),1) , centroids(obstacles(i),2),'*','Markersize',16,'color','black','linewidth',3);
       t = (-01:.01:1)*100;
       %line(centroids(obstacles(i),1)+t*sin(orientations(obstacles(i))*pi/180+pi/2),centroids(obstacles(i),2)+t*cos(orientations(obstacles(i))*pi/180+pi/2) , 'Color', 'black','linewidth',3);
       tipx(i)=centroids(obstacles(i),1) + cos(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3;
       tipy(i)=centroids(obstacles(i),2) - sin(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3;
       if abs(tipy(i)-ylim(1))<50||abs(tipy(i)-ylim(2))<50||abs(tipx(i)-xlim(2))<50||abs(tipx(i)-xlim(1))<50
          tipx(i)=centroids(obstacles(i),1)-cos(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3;
          tipy(i)=centroids(obstacles(i),2)+sin(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3;
       end
       plot(tipx(i), tipy(i),'*','Markersize',16,'color','white','linewidth',3);
       slope(i)=(centroids(obstacles(i),2)-tipy(i))/(centroids(obstacles(i),1)-tipx(i));
       offset(i)=-slope(i)*centroids(obstacles(i),1)+centroids(obstacles(i),2);
       Tangent_offset(i)=tipy(i)+tipx(i)*(1/slope(i));
       line([xlim(1) xlim(2)],[slope(i)*xlim(1)+offset(i) slope(i)*xlim(2)+offset(i)])
       line([xlim(1) xlim(2)],[(-1/slope(i))*xlim(1)+Tangent_offset(i) (-1/slope(i))*xlim(2)+Tangent_offset(i)])
   end
   intersect = [];
   for i=1:size(slope,1)
      for j=i+1:size(slope,1)
          if ((offset(j)-offset(i))/(slope(i)-slope(j))>xlim(1) && (offset(j)-offset(i))/(slope(i)-slope(j))<xlim(2))
              if (offset(j)-offset(i))/(slope(i)-slope(j))*slope(i)+offset(i)>ylim(1) && (offset(j)-offset(i))/(slope(i)-slope(j))*slope(i)+offset(i)<ylim(2)
                  intersect = [[intersect]; [(offset(j)-offset(i))/(slope(i)-slope(j)),(offset(j)-offset(i))/(slope(i)-slope(j))*slope(i)+offset(i)]];
                  plot((offset(j)-offset(i))/(slope(i)-slope(j)),(offset(j)-offset(i))/(slope(i)-slope(j))*slope(i)+offset(i),'*','color','red');
              end
          end
      end
   end
hold off
goalX = 4;
goalY = 4;
s = size(original);
scale = 30;
sizeOfMap = floor(s/scale);
map = zeros(sizeOfMap(1),sizeOfMap(2));
mainRegion = zeros(sizeOfMap(1),sizeOfMap(2),(1+size(obstacles,1)));
transferRegion = zeros(sizeOfMap(1),sizeOfMap(2),(size(obstacles,1)));

corners =[];
map(1,:) = 1;
map(:,1) = 1;
map(sizeOfMap(1),:) = 1;
map(:,sizeOfMap(2)) = 1;
found = false;
hold on
%%% Plot Grid
for i= 1:sizeOfMap(1)
    plot(xlim,[i*scale i*scale],'color','red')
end

for j = 1:sizeOfMap(2)
    plot([j*scale j*scale],ylim,'color',[0.6 0.6 0.6])
end
hold off

ReigonCount=1;

for i=0:sizeOfMap(1)-1          %Horizontal Grid (y-values)
    for j=0:sizeOfMap(2)-1      %Vertical Grid (x-values)
        for equation=1:size(slope,1)
            if (15+j*30)>((15+i*30)-offset(equation))/slope(equation)
                ReigonCount=ReigonCount+1;
            end
        end
        mainRegion((i+1),(j+1),ReigonCount)=1;
        ReigonCount=1;
    end
end
for i=1:size(slope,1)
    lowx=xlim(1);
    highx=xlim(2);
    if abs(ylim(1)-tipy(i))<abs(ylim(2)-tipy(i)) % top is closer
        for ygrid=1:ceil(tipy(i)/scale)
            for j=1:size(slope,1)
                if i==j
                    continue;
                end
                if (((((ygrid*scale)-15)/2)-offset(j))/slope(j))-tipx(i)<0 % on left side
                   if abs(((((ygrid*scale)-15)/2)-offset(j))/slope(j)-tipx(i))<abs(lowx-tipx(i))
                       lowx=((((ygrid*scale)-15)/2)-offset(j))/slope(j);
                   end
               else % on right side
                   if abs(((((ygrid*scale)-15)/2)-offset(j))/slope(j)-tipx(i))<abs(highx-tipx(i))
                       highx=((((ygrid*scale)-15)/2)-offset(j))/slope(j);
                   end
               end
            end
            for m=ceil(lowx/scale):floor(highx/scale)
                transferRegion(ygrid,m,i)=1;
            end
        end
    else % bottom is closer
        for ygrid=floor(tipy(i)/scale):floor(ylim(2)/scale)
            for j=1:size(slope,1)
                if i==j
                    continue;
                end
                if (((((ylim(2)+((ygrid*scale)-15))/2)-offset(j))/slope(j))-tipx(i))<0 % on left side
                    if abs(((((ylim(2)+ ((ygrid*scale)-15))/2)-offset(j))/slope(j))-tipx(i))<abs(lowx-tipx(i))
                        lowx=((((ylim(2)+ ((ygrid*scale)-15))/2)-offset(j))/slope(j));
                    end
                else
                    if abs(((((ylim(2)+ ((ygrid*scale)-15))/2)-offset(j))/slope(j))-tipx(i))<abs(highx-tipx(i))
                        highx=((((ylim(2)+ ((ygrid*scale)-15))/2)-offset(j))/slope(j));
                    end
                end
            end
            for m=ceil(lowx/scale):floor(highx/scale)
                 transferRegion(ygrid,m,i)=1;
            end
        end
    end
end

for obs=1:size(obstacles,1)
    for along = ceil((centroids(obstacles(obs),2) + sin(orientations(obstacles(obs))*pi/180)* majorLength(obstacles(obs))/2.3)/scale) :ceil((centroids(obstacles(obs),2) - sin(orientations(obstacles(obs))*pi/180)* majorLength(obstacles(obs))/2.3)/scale) 
        if along<ylim(1)||along>ylim(2)
            continue;
        end
        x=ceil(((along-offset(obs))/slope(obs))/scale);
        transferRegion(along,x,:)=2;
        mainRegion(along,x,:)=2;
    end
end

figure(2), imshow(transferRegion(:,:,1));
figure(3), imshow(transferRegion(:,:,2));
figure(4), imshow(mainRegion);

for i= 1:sizeOfMap(1)-1
    for j = 1:sizeOfMap(2)-1 
        found = false;
        for k = 0:scale-1
            for l = 0:scale-1
                if BW(i*scale+k, j*scale+l,1) > 0
                    if ~found
                        found = true;
                        results3 = [results3, j i  toc(t0)];
%                         img(i*scale:i*scale+scale,j*scale:j*scale+scale,1) = 255;  % Change the red value for the first pixel
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,2) = 0;    % Change the green value for the first pixel
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,3) = 0;
                    
                    else
                        map(i,j) = 1; %%% points the obstacles.
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,1) = 0;  % Change the red value for the first pixel
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,2) = 0;    % Change the green value for the first pixel
%                     img(i*scale:i*scale+scale,j*scale:j*scale+scale,3) = 255;    % Change the blue value for the first pixel
%                 
                    end
                end
            end
        end
    end
end


% I2 = rgb2hsv(original);
% % % Define thresholds for channel 1 based on histogram settings
% % channel1Min = 0.184;
% % channel1Max = 0.423;
% % 
% % % Define thresholds for channel 2 based on histogram settings
% % channel2Min = 0.184;
% % channel2Max = 0.753;
% % 
% % % Define thresholds for channel 3 based on histogram settings
% % channel3Min = 0.400;
% % channel3Max = 1.000;
% 
% % Define thresholds for channel 1 based on histogram settings
% channel1Min2 = 0.065;
% channel1Max2 = 0.567;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min2 = 0.288;
% channel2Max2 = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min2 = 0.400;
% channel3Max2 = 1.000;
% 
% % Create mask based on chosen histogram thresholds
% BW2 = (I2(:,:,1) >= channel1Min2 ) & (I2(:,:,1) <= channel1Max2) & ...
%     (I2(:,:,2) >= channel2Min2 ) & (I2(:,:,2) <= channel2Max2) & ...
%     (I2(:,:,3) >= channel3Min2 ) & (I2(:,:,3) <= channel3Max2);
% 
% %threshold the image to remove shadows (and only show dark parts of kilobots)
%     [centers, radii] = imfindcircles(BW2,[10 19],'ObjectPolarity','bright','Sensitivity',0.92 );
%     
%     % %Mean
%     M = mean(centers);
%     %Variance
%     V = var(centers);
%     %Covariance
%     C = cov(centers);
%     imshow(original);
%      h = viscircles(centers,radii,'EdgeColor','b');
%     [s, l] = size(centers);
%     plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red', 'linewidth',3);
%     plot_gaussian_ellipsoid(M,C);
% % Convert RGB image to chosen color space
% 
% 
% end
% % 
for j = 2:sizeOfMap(2)-1
    for i = 2:sizeOfMap(1)-1
        if map(i,j) ~=1 
            if (map(i-1,j) == 1 && map(i,j-1) ==1) || (map(i+1,j) == 1 && map(i,j+1) ==1) ...
                    || (map(i+1,j) == 1 && map(i,j-1) ==1) ||(map(i-1,j) == 1 && map(i,j+1) ==1)
                corners = [corners; j i];           
            end
        end
    end
end
save('ThresholdMaps','transferRegion','mainRegion');
%imwrite(img,'Obstacle.jpeg');
%[probability, movesX, movesY] = MDPgridworldExampleBADWALLS(map,goalX,goalY);
%save('Map3', 'movesX', 'movesY','corners');
%save('EmptyMap', 'corners');


% [X,Y] = meshgrid(1:size(map,2),1:size(map,1));
%  hold on; hq=quiver(X*scale,Y*scale,movesY,movesX,0.5,'color',[0,0,0]); hold off
% set(hq,'linewidth',2);
% hold on
% for i= 1: size(corners)
%     plot(corners(i,1)*scale , corners(i,2)*scale,'*','Markersize',16,'color','red', 'linewidth',3);
% end
if webcamShot
clear('cam'); % (*turns off the camera*)
end
% 
% img = imread('testImage12.png');  % Load a jpeg image
% test2 = 12*30;
% test = 30;
% hold on
% 
% img(test:test+10,test2:test2+10,1)= 255;
% img(test:test+10,test2:test2+10,2)= 0;
% img(test:test+10,test2:test2+10,3)= 0;
% imshow(img)