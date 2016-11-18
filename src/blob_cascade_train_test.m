%% Train buoys (example)
% This example shows you the steps involved in training a cascade object
% detector. It trains a 5-stage detector from a very small training set. In
% reality, an accurate detector requires many more stages and thousands of
% positive samples and negative images.
%% 
% Load the positive samples data from a .mat file. The file names and bounding boxes are contained in an array of structures named 'data'.

% Copyright 2015 The MathWorks, Inc.

% load('buoyLabelingSession.mat');
load('handshakeStereoParams.mat');

%positiveInstances = positiveInstances(:,1:2);

%% load in the camera data
% load in the bumblebee2 data
% 
vid = videoinput('pointgrey', 1, 'F7_BayerBG8_640x480_Mode3');
%rosinit('192.168.0.10');
% testpub = rospublisher('/cv_test','cv/blob');

%%
% Add the images location to the MATLAB path.

imDir = fullfile(matlabroot,'toolbox','vision','visiondata',...
    'buoyImages');
addpath(imDir);
%%
% Specify the folder for negative images.
negativeFolder = 'C:\Users\roboboat\Master\cv\data\non_colored_buoys\'; %fullfile(matlabroot,'toolbox','vision','visiondata','nonStopSigns');
negativeImages = imageDatastore(negativeFolder);

%%
% Train a cascade object detector called 'buoyDetector.xml' using HOG features. The following command may take several minutes to run:
trainCascadeObjectDetector('buoyDetector.xml', positiveInstances, negativeFolder, 'FalseAlarmRate', 0.001, 'NumCascadeStages', 4);

%%
% Use the newly trained classifier to detect a buoy in an image.
detector = vision.CascadeObjectDetector('buoyDetector.xml'); 

redThresh = 0.25; % Threshold for red detection
vid = videoinput('pointgrey', 1, 'F7_BayerBG8_640x480_Mode3');

vidInfo = imaqhwinfo(vid); % Acquire input video property
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 800, ...
                                'MaximumBlobArea', 3000, ...
                                'MaximumCount', 10);
hshapeinsRedBox = vision.ShapeInserter('BorderColor', 'Custom', ... % Set Red box handling
                                        'CustomBorderColor', [1 0 0], ...
                                        'Fill', true, ...
                                        'FillColor', 'Custom', ...
                                        'CustomFillColor', [1 0 0], ...
                                        'Opacity', 0.4);
htextins = vision.TextInserter('Text', 'Number of Red Object: %2d', ... % Set text for number of blobs
                                    'Location',  [7 2], ...
                                    'Color', [1 0 0], ... // red color
                                    'FontSize', 12);
htextinsCent = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... % set text for centroid
                                    'LocationSource', 'Input port', ...
                                    'Color', [1 1 0], ... // yellow color
                                    'FontSize', 14);
hVideoIn = vision.VideoPlayer('Name', 'Final Video', ... % Output video player
                                'Position', [100 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);

% initialize counter
%
count = 0;

% get a snapsot of the video feed
%
image = getsnapshot(vid);

% size of the image
%
[im_row, im_col] = size(image);

display('connected to ROS network and camera. start transmitting images...');

%%
% contents = dir('C:\Users\roboboat\Master\cv\data\red_buoys');
% for i=4:21
while (1)   
    image = getsnapshot(vid);
    
    % splits the image the two camera shots
    %
    image = image(:,1:(im_col/6),:);
    % get the new size of the row and column (should be 480x640)
    %
    [i_rows, i_cols] = size(image(:,:,1));
    shape = image;
    image = flipdim(image,2); % obtain the mirror image for displaying
    t = image;
    diffFrame = imsubtract(image(:,:,1), rgb2gray(image)); % Get red component of the\                                                                                    
    diffFrame = medfilt2(diffFrame, [3 3]); % Filter out the noise by using median filter 
    binFrame = im2bw(diffFrame, redThresh); % Convert the image into binary image with the red objects as white                                                                        
    [centroid, bbox_r] = step(hblob, binFrame); % Get the centroids and bound
    centroid = uint16(centroid); % Convert the centroids into Integer for further steps     
    image(1:20,1:165,:) = 0; % put a black region on the output stream                   
    image = step(hshapeinsRedBox, image, bbox_r); % Instert the red box
    
    for object = 1:1:length(bbox_r(:,1)) % Write the corresponding centroids                  
        centX = centroid(object,1); centY = centroid(object,2);
        image = step(htextinsCent, image, [centX centY], [centX-6 centY-9]);
    end
    image = step(htextins, image, uint8(length(bbox_r(:,1)))); % Count the number of blobs    
    
    
    % detects the buoys in the image
    %
    %bbox = step(detector,image);
    
    if isempty(bbox_r)
         num_of_obs = 0;
    else
        % number of objects detected
        %
        [num_of_obs, temp] = size(bbox_r);
    end
   
    % loop through each object in bbox, parse the picture around the object
    % found and try to detect a buoy again.
    %
    for j=1:num_of_obs
        
        % define the row where the parsing starts
        %
        x_start =  uint32(centroid(j,2) - 40);
        
        % define the row where the parsing ends
        %
        x_end = uint32(centroid(j,2) + 40);
        
        % define the column where the parsing starts
        %
        y_start = uint32(centroid(j,1) - 46.67);
        
        % define the column where the parsing ends
        %
        y_end = uint32(centroid(j,1) + 46.67);
        
        % if the parsing is within range, parse the photo, store result in
        % region_of_interest
        %
        if((x_start > 1 && y_start > 1) && (x_end < i_rows && y_end < i_cols))
            region_of_interest = t(x_start:x_end, y_start:y_end, :);
        
        % if not, just parse by the bounding boxes (bbox) and store in 
        % and store in region_of_interest
        %
        else
            x_start = uint32(centroid(j,2) - 10);
            x_end = uint32(centroid(j,2) + 10);
            y_start = uint32(centroid(j,1) - 13.33);
            y_end = uint32(centroid(j,1) + 13.33);
            region_of_interest = t(x_start:x_end, y_start:y_end, :);
        end
        
        %imshow(region_of_interest);
        %pause;
        % test to see if there is a circular shape within the
        % region_of_interest
        %
        [src, dst, circ] = shape_detector(region_of_interest);
        
        % detect the objects inside the region_of_interest
        %

        bbox_01 = step(detector,region_of_interest);
        %[src, dst, circ] = shape_detector(region_of_interest);
        % if an object was detected in the region_of_interest, then output
        % the uncertainty of the buoy existing
        %
        if(circ == 0 || ~isempty(bbox_01))           
            % output uncertainty of the buoy being found
            %
            dst = insertObjectAnnotation(image,'rectangle',[y_start, x_start, y_end-y_start, x_end-x_start],'buoy found (more certain)!');
            disp('Buoy found!');
        % if not, then set the detected image to the image
        %
        else
           dst = image;
        end
    
    % end forloop
    end
   
    imshow(dst)

% end while loop
end
%     [m,n] = size(bbox);
%     area = zeros(m,1);
%     pixels = zeros(m,3);
%     xcoord = zeros(m,1);
%     ycoord = zeros(m,1);
%     buoy_detected = 0;
%      for j=1:m
%         buoy_detected = 1;
%         xCenter = floor(bbox(j,1)+ (bbox(j,3)/2));
%         yCenter = floor(bbox(j,2)+ (bbox(j,4)/2));
%         xcoord(j,1) = -1*((col/2) - yCenter)/(col/2);
%         ycoord(j,1) = ((row/2) - xCenter)/(row/2);      
%         pixels(j,:) = impixel(image, xCenter, yCenter);
%         area(j,1) = bbox(j,3)*bbox(j,4);
%      end
%      
%      if(~buoy_detected)
%          msg = rosmessage(testpub);
%          msg.BuoyDetected = 0;
%          % give the message the string that needs to be sent
%          msg.Color = num2str(0);
%          % sent the x-y coordinates
%          msg.Xcoord = 0;    
%          msg.Ycoord = 0;    
%          msg.Area = 0;
%          send(testpub, msg)
%          detectedImg = image;
%      else
%          
%          [m, in] = max(area);
%          msg = rosmessage(testpub);
%          msg.BuoyDetected = 1;
%          msg.Color = num2str(pixels(in,1));
%          msg.Xcoord = xcoord(in,1);    
%          msg.Ycoord = ycoord(in,1);    
%          msg.Area = area(in,1);
%          display(ycoord(in,1));
%          display(xcoord(in,1));  
%     
%          send(testpub, msg)
%          detectedImg = insertObjectAnnotation(image,'rectangle',bbox(in,:),['buoy' num2str(pixels(in,:))]);
%      end
%     
%     imshow(detectedImg);
%     pause(1);
% end

    
%%
% Remove the image directory from the path.
rmpath(imDir);
