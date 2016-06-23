%%%%%%%%% By Shiva Shahrokhi June 2016: This code takes a snapshot of the
%%%%%%%%% webcam, then process it to find obstacles, gives that map to MDP
%%%%%%%%% and gets the result and draw the gradients and regions. 
success = false;
webcamShot = false;
obstacles = [];
if webcamShot
    cam = webcam(1);

end
t0 = tic;
results3= [];
%while success == false
if webcamShot
 originalImage = snapshot(cam);
 original = imcrop(originalImage,[345 60 1110 860]);
 img = imcrop(originalImage,[345 60 1110 860]);
 %rgbIm = imcrop(originalImage,[345 60 1110 860]);
else 
    rgbIm = imread('test.png');
end



% I = rgb2hsv(img);
% 
% % Define thresholds for channel 1 based on histogram settings
% channel1Min = 0.847;
% channel1Max = 0.946;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min = 0.174;
% channel2Max = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min = 0.657;
% channel3Max = 1.000;

    %I = rgb2hsv(originalImage);
    I = rgb2hsv(rgbIm);
    
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.902;
channel1Max = 0.938;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.205;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.795;
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
       if area(i) > 50
           obstacles = [obstacles; i];
       end
    end
   for i = 1: size(obstacles)
       hold on
       plot(centroids(obstacles(i),1) , centroids(obstacles(i),2),'*','Markersize',16,'color','black','linewidth',3);
       t = (-01:.01:1)*100;
       line(centroids(obstacles(i),1)+t*sin(orientations(obstacles(i))*pi/180+pi/2),centroids(obstacles(i),2)+t*cos(orientations(obstacles(i))*pi/180+pi/2) , 'Color', 'black','linewidth',3);
       plot(centroids(obstacles(i),1) + cos(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3,centroids(obstacles(i),2) - sin(orientations(obstacles(i))*pi/180)* majorLength(obstacles(i))/2.3 ,'*','Markersize',16,'color','white','linewidth',3);
   end
hold off
goalX = 4;
goalY = 4;
s = size(BW);
scale = 30;
sizeOfMap = floor(s/scale);
map = zeros(sizeOfMap(1),sizeOfMap(2));

corners =[];
map(1,:) = 1;
map(:,1) = 1;
map(sizeOfMap(1),:) = 1;
map(:,sizeOfMap(2)) = 1;
found = false;

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
