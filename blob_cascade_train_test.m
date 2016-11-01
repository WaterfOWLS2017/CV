%% Train buoys (example)
% This example shows you the steps involved in training a cascade object
% detector. It trains a 5-stage detector from a very small training set. In
% reality, an accurate detector requires many more stages and thousands of
% positive samples and negative images.
%% 
% Load the positive samples data from a .mat file. The file names and bounding boxes are contained in an array of structures named 'data'.

% Copyright 2015 The MathWorks, Inc.

load('buoyLabelingSession.mat');  

%positiveInstances = labeledBuoys(:,1:2);
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
trainCascadeObjectDetector('buoyDetector.xml', data, negativeFolder, 'FalseAlarmRate', 0.001, 'NumCascadeStages', 3);

%%
% Use the newly trained classifier to detect a buoy in an image.
detector = vision.CascadeObjectDetector('buoyDetector.xml'); 

%%
contents = dir('red_buoys/');
for i = 4:numel(contents)
    
    % Read the test image.
    img = imread(['red_buoys/' contents(i).name]);
  
    % Detect a stop sign.
    bbox = step(detector,img);
    
    % Insert bounding boxes and return marked image.
    detectedImg = insertObjectAnnotation(img,'rectangle',bbox(1, :),'buoy');
    
    % Display the detected stop sign.
    figure;
    imshow(detectedImg);
    pause;
end

    
%%
% Remove the image directory from the path.
rmpath(imDir);
