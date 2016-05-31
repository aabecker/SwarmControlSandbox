 
%%% Controlling Covariance of kilobots with light by using a webcam, by: Shiva
%%% Shahrokhi and Aaron T. Becker @ University of Houston, Robotic Swarm
%%% Control Lab.

function TestLighting()
%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
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
cam = webcam(2);
t0 = tic;
success = false;
ifArd = true;
if ifArd
    if (ispc)  
        a = arduino('Com4','uno');
    else 
        a = arduino('/dev/tty.usbmodem1421','uno');
    end 
end
while success == false

   if ifArd
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY2,1);
        writeDigitalPin(a,RELAY3,0);
        writeDigitalPin(a,RELAY4,1);
        writeDigitalPin(a,RELAY5,1);
        writeDigitalPin(a,RELAY6,1);
        writeDigitalPin(a,RELAY7,1);
        writeDigitalPin(a,RELAY8,1);
    pause (120);
   end
    % Read in a webcam snapshot.
    rgbIm = snapshot(cam);

    %crop to have just the table view.
rgbIm = snapshot(cam);
if (ispc)  
    originalImage = imcrop(rgbIm,[50 10 500 400]);
else 
    originalImage = imcrop(rgbIm,[345 60 1110 850]);
end 
% make grayscale.
%printing image
% I = rgb2hsv(originalImage);
% 
%     DateString = datestr(datetime, '_mm-dd-yyyy_');
%     timeString=datestr(datetime, 'HH.MM.SS');
%     name = 'RelayALL';
% 
%     fullName = strcat(name,DateString,timeString,'.jpeg' );
%     imwrite(originalImage,fullName);
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

  %threshold the image to remove shadows (and only show dark parts of kilobots)
    [centers, radii] = imfindcircles(BW,[10 19],'ObjectPolarity','bright','Sensitivity',0.92 );
    
    % %Mean
    M = mean(centers);
    %Variance
    V = var(centers);
    %Covariance
    C = cov(centers);
    
    imshow(originalImage)
    h = viscircles(centers,radii,'EdgeColor','b');
    [s, l] = size(centers);

    
    if isnan(M)== false 
        
    if s > 85 
    hold on
    
    plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red');

    %Current Mean and Covariance Ellipse
    plot_gaussian_ellipsoid(M,C);
    hold off
    end
    end

    if(toc(t0) > 800)
        success = true;
       clear('cam')
    end
end

 


end
