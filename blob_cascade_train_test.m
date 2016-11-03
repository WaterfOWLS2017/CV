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
%vid = videoinput('pointgrey', 1, 'F7_BayerBG8_640x480_Mode3');


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

%image = getsnapshot(vid);

%[m n] = size(image);
%imshow(image);

%%
contents = dir('red_buoys');
for i=4:21
%while (1)   
%    image = getsnapshot(vid);
%    image = image(:,1:(n/6),:);
%    imshow(image);
%1

    % Read the test image.
    image = imread(['red_buoys/' contents(i).name]);
    [row, col] = size(image);
    col = col/3;
    % Detect a stop sign.
    bbox = step(detector,image);
    [m,n] = size(bbox);
    detectedImg = image;
     for j=1:m
        xCenter = round(bbox(j,1)+ (bbox(j,3)/2));
        yCenter = round(bbox(j,2)+ (bbox(j,4)/2));
        display(xCenter);
        display(yCenter);
        ycoord = ((col/2) - yCenter)/(col/2);
        xcoord = -1*((row/2) - xCenter)/(row/2);
        display(xcoord);
        display(ycoord);
        pixels = impixel(image, xCenter, yCenter);
        detectedImg = insertObjectAnnotation(detectedImg,'rectangle',bbox(j,:),['buoy' num2str(pixels)]);
     end
    
    % Insert bounding boxes and return marked image.
    % detectedImg = insertObjectAnnotation(detectedImg,'rectangle',bbox,'buoy');
     
    
    % Display the detected stop sign.
    %figure;
    %imshow(detectedImg);
    
    imshow(detectedImg);
    pause;
end

    
%%
% Remove the image directory from the path.
rmpath(imDir);
