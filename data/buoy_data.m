%% Train buoys (example)
% This example shows you the steps involved in training a cascade object
% detector. It trains a 5-stage detector from a very small training set. In
% reality, an accurate detector requires many more stages and thousands of
% positive samples and negative images.

%%
% Add the images location to the MATLAB path.
imDir = fullfile(matlabroot,'toolbox','vision','visiondata',...
    'buoyImages');
addpath(imDir);
%%
% Specify the folder for negative images.
negativeFolder = 'non_colored_buoys/';
negativeImages = imageDatastore(negativeFolder);
%%
% Train a cascade object detector called 'buoyDetector.xml' using HOG features. The following command may take several minutes to run:
trainCascadeObjectDetector('buoyDetector.xml', positiveInstances, negativeFolder, 'FalseAlarmRate', 0.001,'FeatureType', 'LBP', 'NumCascadeStages', 4);


%%
% Use the newly trained classifier to detect a buoy in an image.
detector = vision.CascadeObjectDetector('buoyDetector.xml');


%%
contents = dir('eval');
for i=3:45
    image = imread(['eval/' contents(i).name]);
    bbox = step(detector,image);
    detectedImg = insertObjectAnnotation(image,'rectangle',bbox,'buoy');
    imshow(detectedImg);
    display(size(bbox));
    pause;
end


    
%%
% Remove the image directory from the path.
rmpath(imDir);