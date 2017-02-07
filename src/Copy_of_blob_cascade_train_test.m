%% Train buoys (example)
% This example shows you the steps involved in training a cascade object
% detector. It trains a 5-stage detector from a very small training set. In
% reality, an accurate detector requires many more stages and thousands of
% positive samples and negative images.
%% 
% Load the positive samples data from a .mat file. The file names and bounding boxes are contained in an array of structures named 'data'.

% Copyright 2015 The MathWorks, Inc.

load('C:\Users\roboboat\Master\cv\data\buoyLabelingSession.mat');

%%
% Add the images location to the MATLAB path.

imDir = fullfile(matlabroot,'toolbox','vision','visiondata',...
    'buoyImages');
addpath(imDir);
%%
% Specify the folder for negative images.
negativeFolder = 'C:\Users\roboboat\Master\cv\data\non_colored_buoys'; %fullfile(matlabroot,'toolbox','vision','visiondata','nonStopSigns');
negativeImages = imageDatastore(negativeFolder);
%%
% Train a cascade object detector called 'buoyDetector.xml' using HOG features. The following command may take several minutes to run:
trainCascadeObjectDetector('buoyDetector.xml', positiveInstances, negativeFolder, 'FalseAlarmRate', 0.001, 'NumCascadeStages', 4);

%%
% Use the newly trained classifier to detect a buoy in an image.
detector = vision.CascadeObjectDetector('buoyDetector.xml'); 

%%
W = 0.203;
P = 290;
D = 0.3048;
F = (P*D)/W;
display(F)
contents = dir('C:\Users\roboboat\Master\cv\data\master_buoy');
count = 0;
for i=4:144
    image = imread(['C:\Users\roboboat\Master\cv\data\master_buoy\' contents(i).name]); 
    bbox = step(detector,image);
    [num_of_obs, n] = size(bbox);
    [i_rows, i_cols] = size(image(:,:,1));
    detectedImg = insertObjectAnnotation(image,'rectangle',bbox,'buoy');
    imshow(detectedImg);
    pause;
    
    for j=1:num_of_obs
        x_start =  uint32(bbox(j,2)-20);
        x_end = uint32(bbox(j,2)+bbox(j,4)+20);
        y_start = uint32(bbox(j,1)-26.65);
        y_end = uint32(bbox(j,1)+bbox(j,3)+26.65);
        
        if(x_start > 1 && y_start > 1 && x_end <= i_rows && y_end <= i_cols)
            region_of_interest = image(x_start:x_end, y_start:y_end, :);
        else
            x_start =  uint32(bbox(j,2));
            x_end = uint32(bbox(j,2)+bbox(j,4));
            y_start = uint32(bbox(j,1));
            y_end = uint32(bbox(j,1)+bbox(j,3));
            region_of_interest = image(x_start:x_end, y_start:y_end, :);
        end
        
        [src, dst, circ] = shape_detector(region_of_interest);
        imshow(region_of_interest);
        pause;
        bbox_01 = step(detector,region_of_interest);
        
        if(bbox_01)
            image = insertObjectAnnotation(image,'rectangle',bbox(j,:),'buoy found (uncertain)!');
            if (circ == 0)
                detectedImg = insertObjectAnnotation(image,'rectangle',bbox(j,:),'buoy found              (more certain)!');
                count = count + 1;
            end
        else
            detectedImg = image;
        end
    end
    
    imshow(detectedImg);
    pause;
end

    
%%
% Remove the image directory from the path.
rmpath(imDir);
