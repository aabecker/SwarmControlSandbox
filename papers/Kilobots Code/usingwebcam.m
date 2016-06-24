% webcamlist % (*Show the list of webcams*)

cam = webcam(1);

originalImage = snapshot(cam);
img = imcrop(originalImage,[345 60 1110 860]);
imwrite(img,'Obstacle.jpeg');
% make grayscale.
%originalImage = rgb2gray(originalImage);
imshow(img)% (*This command shows the image*)
%roipoly(img)

clear('cam'); % (*turns off the camera*)