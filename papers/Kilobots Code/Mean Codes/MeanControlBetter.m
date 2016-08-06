
%%% Controlling Covariance of kilobots with light by using a webcam, by: Shiva
%%% Shahrokhi and Aaron T. Becker @ University of Houston, Robotic Swarm
%%% Control Lab.

function MeanControlBetter()

% Using Arduino for our lamps, this is how we define arduino in Matlab:
a = arduino('/dev/tty.usbmodem1421','uno');
%Define webcam --the input may be 1 or 2 depending on which webcam of your laptop
%is the default webcam.
cam = webcam(1);
global q goalX

% We have 8 Relays.
%west
RELAY1 = 7;
% northwest
RELAY2 = 6;
%north
RELAY3 = 3;
% northeast
RELAY4 = 0;
%east
RELAY5 = 5;
%southeast
RELAY6 = 1;
%south
RELAY7 = 4;
%southwest
RELAY8 = 2;
% this is the mean y goal
goalYM = 434;
%this is the mean x goal
goalXM = 600;

epsilon = 10;

%These are the goal X.

goal1x = 800;
goal2x = 600;
goalX = goal1x;
t0 = tic;
q = zeros(1,2);

success = false;
again = true;

tHandle = timer('TimerFcn',...
    {@sqWave_callback_fcn, goal1x, goal2x,t0}, ...
    'Period' , 100, 'TasksToExecute' , 4, 'ExecutionMode', 'fixedDelay');
 
start(tHandle);


drawTime=[goalX,0];
while success == false

    if again== true
                    writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a, RELAY1,1);
            writeDigitalPin(a,RELAY5,1);
    pause (10);
    end 
    % Read in a webcam snapshot.
    rgbIm = snapshot(cam);
    %imwrite(rgbIm,'FailImage6.png');
    %crop to have just the table view.
originalImage = imcrop(rgbIm,[345 60 1110 850]);
% make grayscale.
I = rgb2hsv(originalImage);
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.184;
channel1Max = 0.423;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.184;
channel2Max = 0.753;

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
    
    imshow(originalImage);
    h = viscircles(centers,radii);
    [s, l] = size(centers);
    %goalC = [18000 goalX; goalX 18000];
    
    if isnan(M)== false 
        
    if s > 5 
        again = false;
    hold on
    
    plot(M(1,1) , M(1,2),'*','Markersize',16,'color','red');
    newDot = [M(1,1), toc(t0)];
    drawTime = [drawTime;newDot];
    %Current Mean and Covariance Ellipse
    plot_gaussian_ellipsoid(M,C);
    %Goal Mean and Covariance Ellipse
    %plot_gaussian_ellipsoid([goalXM goalYM],goalC);
    %plot(centers(:,1),centers(:,2),'+','Markersize',16);
    %Goal X.
    line([goalX goalX], ylim,'color','green','linewidth', 3.0);
    hold off
        if M(1,1) > goalX+epsilon
        writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a, RELAY1,0);
        writeDigitalPin(a,RELAY5,1);
        pause(1);
       
    else if M(1,1) < goalX-epsilon
            writeDigitalPin(a,RELAY3,1);
        writeDigitalPin(a, RELAY1,1);
        writeDigitalPin(a,RELAY5,0);
        pause(1);
       
        else 
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a, RELAY1,1);
            writeDigitalPin(a,RELAY5,1);
            pause(1);
            again = true;
        end
        end
    else
        again = true;
    end
    
    end

    if(toc(t0) > 400)
        success = true;
        figure(2)
       plot(q(:,1),q(:,2));
        xlabel('time (s)');
        ylabel('Mean');
        
        hold on 
        plot(drawTime(:,2), drawTime(:,1));
        hold off
        
       clear('cam')
    end
end

stop(tHandle)
 
function  sqWave_callback_fcn(src,evt, goal1x, goal2x,t0) %#ok<DEFNU>

   tval = toc(t0);
    q= [q;[tval, goalX]];
    
    if goalX == goal1x
        goalX = goal2x;
    else
        goalX = goal1x;
    end
    q= [q;[tval, goalX]];


end

end
