%% Train buoys (example)
% This example shows you the steps involved in training a cascade object
% detector. It trains a 5-stage detector from a very small training set. In
% reality, an accurate detector requires many more stages and thousands of
% positive samples and negative images.
%% 
% Load the positive samples data from a .mat file. The file names and bounding boxes are contained in an array of structures named 'data'.

% Copyright 2015 The MathWorks, Inc.

load('buoyLabelingSession.mat');

%positiveInstances = positiveInstances(:,1:2);

%% load in the camera data
% load in the bumblebee2 data
% 
vid = videoinput('pointgrey', 1, 'F7_BayerBG8_640x480_Mode3');
%rosinit('192.168.0.10');
testpub = rospublisher('/cv_test','cv/blob');

%%
% Add the images location to the MATLAB path.

imDir = fullfile(matlabroot,'toolbox','vision','visiondata',...
    'buoyImages');
addpath(imDir);
%%
% Specify the folder for negative images.
negativeFolder = 'non_colored_buoys/'; %fullfile(matlabroot,'toolbox','vision','visiondata','nonStopSigns');
negativeImages = imageDatastore(negativeFolder);
%%
% Train a cascade object detector called 'buoyDetector.xml' using HOG features. The following command may take several minutes to run:
trainCascadeObjectDetector('buoyDetector.xml', positiveInstances, negativeFolder, 'FalseAlarmRate', 0.001, 'NumCascadeStages', 4);

%%
% Use the newly trained classifier to detect a buoy in an image.
detector = vision.CascadeObjectDetector('buoyDetector.xml'); 

image = getsnapshot(vid);

[im_row im_col] = size(image);
%imshow(image);

display('connected to ROS network and camera. start transmitting images...');

%%
%contents = dir('red_buoys');
%for i=4:21
while (1)   
    image = getsnapshot(vid);
    
    % splits the image
    image = image(:,1:(im_col/6),:);
    imshow(image);


    % Read the test image.
    [row, col] = size(image);
    col = col/3;

    bbox = step(detector,image);
    [m,n] = size(bbox);
    area = zeros(m,1);
    pixels = zeros(m,3);
    xcoord = zeros(m,1);
    ycoord = zeros(m,1);
    buoy_detected = 0;
     for j=1:m
        buoy_detected = 1;
        xCenter = floor(bbox(j,1)+ (bbox(j,3)/2));
        yCenter = floor(bbox(j,2)+ (bbox(j,4)/2));
        xcoord(j,1) = -1*((col/2) - yCenter)/(col/2);
        ycoord(j,1) = ((row/2) - xCenter)/(row/2);      
        pixels(j,:) = impixel(image, xCenter, yCenter);
        area(j,1) = bbox(j,3)*bbox(j,4);
     end
     
     if(~buoy_detected)
         msg = rosmessage(testpub);
         msg.BuoyDetected = 0;
         % give the message the string that needs to be sent
         msg.Color = num2str(0);
         % sent the x-y coordinates
         msg.Xcoord = 0;    
         msg.Ycoord = 0;    
         msg.Area = 0;
         send(testpub, msg)
         detectedImg = image;
     else
         
         [m, in] = max(area);
         msg = rosmessage(testpub);
         msg.BuoyDetected = 1;
         msg.Color = num2str(pixels(in,1));
         msg.Xcoord = xcoord(in,1);    
         msg.Ycoord = ycoord(in,1);    
         msg.Area = area(in,1);
         display(ycoord(in,1));
         display(xcoord(in,1));  
    
         send(testpub, msg)
         detectedImg = insertObjectAnnotation(image,'rectangle',bbox(in,:),['buoy' num2str(pixels(in,:))]);
     end
    
    imshow(detectedImg);
    pause(1);
end

    
%%
% Remove the image directory from the path.
rmpath(imDir);
