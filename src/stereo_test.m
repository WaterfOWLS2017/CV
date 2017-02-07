%% Depth Estimation From Stereo Video
% This example shows how to detect people in video taken with a calibrated
% stereo camera and determine their distances from the camera.
%
%   Copyright 2013-2014 The MathWorks, Inc.

%% Load the Parameters of the Stereo Camera
% Load the |stereoParameters| object, which is the result of calibrating
% the camera using either the |stereoCameraCalibrator| app or the
% |estimateCameraParameters| function.

% Load the stereoParameters object.
load('handshakeStereoParams.mat');

% Visualize camera extrinsics.
showExtrinsics(stereoParams);

%% Create Video File Readers and the Video Player
% Create System Objects for reading and displaying the video.
vid = videoinput('pointgrey', 1, 'F7_BayerBG8_640x480_Mode3');
image = getsnapshot(vid);
[im_row, im_col] = size(image);
right_image = image(:,1:(im_col/6),:);
left_image = image(:,im_col/6 + 1:(im_col/3),:);

% Convert to grayscale.
I1gray = rgb2gray(left_image);
I2gray = rgb2gray(right_image);

% Display both images side by side. Then, display a color composite
% demonstrating the pixel-wise differences between the images.
figure;
imshowpair(left_image, right_image,'montage');
title('I1 (left); I2 (right)');
figure; 
imshow(stereoAnaglyph(left_image,right_image));
title('Composite Image (Red - Left Image, Cyan - Right Image)');

%% Step 2: Collect Interest Points from Each Image
% The rectification process requires a set of point correspondences between
% the two images. To generate these correspondences, you will collect
% points of interest from both images, and then choose potential matches
% between them. Use |detectSURFFeatures| to find blob-like features in both
% images.
blobs1 = detectSURFFeatures(I1gray, 'MetricThreshold', 1000);
blobs2 = detectSURFFeatures(I2gray, 'MetricThreshold', 1000);

%%
% Visualize the location and scale of the thirty strongest SURF features in
% I1 and I2.  Notice that not all of the detected features can be matched
% because they were either not detected in both images or because some of
% them were not present in one of the images due to camera motion.
figure; 
imshow(left_image);
hold on;
plot(selectStrongest(blobs1, 30));
title('Thirty strongest SURF features in I1');

figure; 
imshow(right_image); 
hold on;
plot(selectStrongest(blobs2, 30));
title('Thirty strongest SURF features in I2');

%% Step 3: Find Putative Point Correspondences
% Use the |extractFeatures| and |matchFeatures| functions to find putative
% point correspondences. For each blob, compute the SURF feature vectors
% (descriptors).
[features1, validBlobs1] = extractFeatures(I1gray, blobs1);
[features2, validBlobs2] = extractFeatures(I2gray, blobs2);

%%
% Use the sum of absolute differences (SAD) metric to determine indices of
% matching features.
indexPairs = matchFeatures(features1, features2, 'Metric', 'SAD', ...
  'MatchThreshold', 5);

%%
% Retrieve locations of matched points for each image.
matchedPoints1 = validBlobs1(indexPairs(:,1),:);
matchedPoints2 = validBlobs2(indexPairs(:,2),:);

%%
% Show matching points on top of the composite image, which combines stereo
% images. Notice that most of the matches are correct, but there are still
% some outliers.
figure; 
showMatchedFeatures(left_image, right_image, matchedPoints1, matchedPoints2);
legend('Putatively matched points in I1', 'Putatively matched points in I2');

%% Step 4: Remove Outliers Using Epipolar Constraint
% The correctly matched points must satisfy epipolar constraints. This
% means that a point must lie on the epipolar line determined by its
% corresponding point. You will use the |estimateFundamentalMatrix|
% function to compute the fundamental matrix and find the inliers that meet
% the epipolar constraint.
[fMatrix, epipolarInliers, status] = estimateFundamentalMatrix(...
  matchedPoints1, matchedPoints2, 'Method', 'RANSAC', ...
  'NumTrials', 10000, 'DistanceThreshold', 0.1, 'Confidence', 90);
  
if status ~= 0 || isEpipoleInImage(fMatrix, size(left_image)) ...
  || isEpipoleInImage(fMatrix', size(right_image))
  error(['Either not enough matching points were found or '...
         'the epipoles are inside the images. You may need to '...
         'inspect and improve the quality of detected features ',...
         'and/or improve the quality of your images.']);
end

inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

figure;
showMatchedFeatures(left_image, right_image, inlierPoints1, inlierPoints2);
legend('Inlier points in I1', 'Inlier points in I2');

%% Step 5: Rectify Images
% Use the |estimateUncalibratedRectification| function to compute the
% rectification transformations. These can be used to transform the images,
% such that the corresponding points will appear on the same rows.
[t1, t2] = estimateUncalibratedRectification(fMatrix, ...
  inlierPoints1.Location, inlierPoints2.Location, size(right_image));
tform1 = projective2d(t1);
tform2 = projective2d(t2);

%%
% Rectify the stereo images, and display them as a stereo anaglyph.
% You can use red-cyan stereo glasses to see the 3D effect.
[frameLeftRect, frameRightRect] = rectifyStereoImages(left_image, right_image, tform1, tform2);
figure;
imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
[frameLeftRect, frameRightRect] = rectifyStereoImages(left_image, right_image, stereoParams);
title('Rectified Stereo Images (Red - Left Image, Cyan - Right Image)');

%[frameLeftRect, frameRightRect] = ...
%    rectifyStereoImages(right_image, left_image, stereoParams);

figure;
imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
title('Rectified Video Frames');

%% Compute Disparity
% In rectified stereo images any pair of corresponding points are located 
% on the same pixel row. For each pixel in the left image compute the
% distance to the corresponding pixel in the right image. This distance is
% called the disparity, and it is proportional to the distance of the
% corresponding world point from the camera.
frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);
    
disparityMap = disparity(frameLeftGray, frameRightGray);

figure;
imshow(disparityMap, [0, 64]);
title('Disparity Map');
colormap jet
colorbar

%% Reconstruct the 3-D Scene
% Reconstruct the 3-D world coordinates of points corresponding to each
% pixel from the disparity map.
points3D = reconstructScene(disparityMap, stereoParams);

% Convert to meters and create a pointCloud object
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);

% Create a streaming point cloud viewer
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');

% Visualize the point cloud
view(player3D, ptCloud);


%% Detect People in the Left Image
% Use the |vision.PeopleDetector| system object to detect people.

% Create the people detector object. Limit the minimum object size for
% speed.
faceDetector = vision.CascadeObjectDetector();


% Detect people.
bboxes = step(faceDetector, frameLeftGray);

%% Determine The Distance of Each Person to the Camera
% Find the 3-D world coordinates of the centroid of each detected person
% and compute the distance from the centroid to the camera in meters.

% Find the centroids of detected people.
centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
    round(bboxes(:, 2) + bboxes(:, 4) / 2)];

% Find the 3-D world coordinates of the centroids.
centroidsIdx = sub2ind(size(disparityMap), centroids(:, 2), centroids(:, 1));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
centroids3D = [X(centroidsIdx)'; Y(centroidsIdx)'; Z(centroidsIdx)'];

% Find the distances from the camera in meters.
dists = sqrt(sum(centroids3D .^ 2));
    
% Display the detected people and their distances.
labels = cell(1, numel(dists));
for i = 1:numel(dists)
    labels{i} = sprintf('%0.2f meters', dists(i));
end
figure;
imshow(insertObjectAnnotation(frameLeftRect, 'rectangle', bboxes, labels));
title('Detected People');

%% Process the Rest of the Video
% Apply the steps described above to detect people and measure their
% distances to the camera in every frame of the video.

while (1)
    % Read the frames.
    %frameLeft = readerLeft.step();
    %frameRight = readerRight.step();
    image = getsnapshot(vid);
    [im_row, im_col] = size(image);
    right_image = image(:,1:(im_col/6),:);
    left_image = image(:,im_col/6 + 1:(im_col/3),:);
    
    % Rectify the frames.
    [frameLeftRect, frameRightRect] = ...
        rectifyStereoImages(left_image, right_image, stereoParams);
    
    % Convert to grayscale.
    frameLeftGray  = rgb2gray(frameLeftRect);
    frameRightGray = rgb2gray(frameRightRect);
    
    % Compute disparity. 
    disparityMap = disparity(frameLeftGray, frameRightGray);
    
    % Reconstruct 3-D scene.
    points3D = reconstructScene(disparityMap, stereoParams);
    points3D = points3D ./ 1000;
    ptCloud = pointCloud(points3D, 'Color', frameLeftRect);
    %view(player3D, ptCloud);
    
    % Detect people.
    bboxes =  step(faceDetector, frameLeftGray);
    
    if ~isempty(bboxes)
        % Find the centroids of detected people.
        centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
            round(bboxes(:, 2) + bboxes(:, 4) / 2)];
        
        % Find the 3-D world coordinates of the centroids.
        centroidsIdx = sub2ind(size(disparityMap), centroids(:, 2), centroids(:, 1));
        X = points3D(:, :, 1);
        Y = points3D(:, :, 2);
        Z = points3D(:, :, 3);
        centroids3D = [X(centroidsIdx), Y(centroidsIdx), Z(centroidsIdx)];
        
        % Find the distances from the camera in meters.
        dists = sqrt(sum(centroids3D .^ 2, 2));
        
        % Display the detect people and their distances.
        labels = cell(1, numel(dists));
        for i = 1:numel(dists)
            labels{i} = sprintf('%0.2f meters', dists(i));
        end
        dispFrame = insertObjectAnnotation(frameLeftRect, 'rectangle', bboxes,...
            labels);
    else
        dispFrame = frameLeftRect;
    end
    
    % Display the frame.
    imshow(dispFrame);
end


%% Summary
% This example showed how to localize pedestrians in 3-D using a calibrated
% stereo camera.

%% References
% [1] G. Bradski and A. Kaehler, "Learning OpenCV : Computer Vision with
% the OpenCV Library," O'Reilly, Sebastopol, CA, 2008.
%
% [2] Dalal, N. and Triggs, B., Histograms of Oriented Gradients for
% Human Detection. CVPR 2005.


