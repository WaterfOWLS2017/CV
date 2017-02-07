%function [image, isgreen] = detectGreenObject(image)

% Threshold for red detection
%
greenThresh = 0.05; 

% video input
%
vid = videoinput('pointgrey', 1, 'F7_BayerBG8_640x480_Mode3');

% Acquire input video property
%
vidInfo = imaqhwinfo(vid); 

% Set blob analysis handling
%
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
                            'CentroidOutputPort', true, ...
                            'BoundingBoxOutputPort', true', ...
                            'MinimumBlobArea', 800, ...
                            'MaximumBlobArea', 3000, ...
                            'MaximumCount', 10);

% Set Red box handling
%
hshapeinsGreenBox = vision.ShapeInserter('BorderColor', 'Custom', ... 
                                        'CustomBorderColor', [1 0 0], ...
                                        'Fill', true, ...
                                        'FillColor', 'Custom', ...
                                        'CustomFillColor', [1 0 0], ...
                                        'Opacity', 0.4);

% Set text for number of blobs
%
htextins = vision.TextInserter('Text', 'Number of Red Object: %2d', ... 
                                    'Location',  [7 2], ...
                                    'Color', [0 1 0], ... // red color
                                    'FontSize', 12);

                                

% set text for centroid
%
htextinsCent = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... 
                                    'LocationSource', 'Input port', ...
                                    'Color', [1 1 0], ... // yellow color
                                    'FontSize', 14);
                                
% Output video player
% 
hVideoIn = vision.VideoPlayer('Name', 'Final Video', ...
                                'Position', [100 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);
                                

while (1)
    
    % get a snapsot of the video feed
    %
    image = getsnapshot(vid);

    % size of the image
    %
    [im_row, im_col] = size(image);
    
    % splits the image the two camera shots
    %
    image = image(:,1:(im_col/6),:);

    % obtain the mirror image for displaying
    %
    image = flipdim(image,2);
    
    % Get red component of the image
    %
    diffFrame = imsubtract(image(:,:,2), rgb2gray(image));
    
    % Filter out the noise by using median filter
    %
    diffFrame = medfilt2(diffFrame, [3 3]);
    
    % Convert the image into binary image with the red objects as white
    %
    binFrame = im2bw(diffFrame, greenThresh);
    
    % Get the centroids and bound
    %
    [centroid, bbox_r] = step(hblob, binFrame);
    
    % Convert the centroids into Integer for further steps
    %
    centroid = uint16(centroid);
    
    % put a black region on the output stream
    %
    image(1:20,1:165,:) = 0;
    
    % Instert the red box
    %
    image = step(hshapeinsGreenBox, image, bbox_r);
    
    % Write the corresponding centroids
    %
    for object = 1:1:length(bbox_r(:,1))
        centX = centroid(object,1); centY = centroid(object,2);
        image = step(htextinsCent, image, [centX centY], [centX-6 centY-9]);
    end
    
    % Count the number of blobs
    %
    image = step(htextins, image, uint8(length(bbox_r(:,1))));
    
    isred = 0;
    
    imshow(image);
    
end