clc; clear;
% Create the face detector object.
blobDetector = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', 20000, 'MaximumBlobArea', 50000);

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam('FaceTime HD Camera (Built-in)');

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
numPts = 0;
frameCount = 0;

while runLoop && frameCount < 400

    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    videoFrameCanny = im2bw(videoFrameGray, 0.3);
    videoFrame = imresize(videoFrame, 0.7);
    
     
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        [area, centroid, bbox] = step(blobDetector, videoFrameCanny);
        
        [centers, radii] = imfindcircles(videoFrame,[100 150],'ObjectPolarity','dark', ...
                                        'Sensitivity',0.95);
        if ~isempty(centers)
            bbox = [centers(1,2) - radii(1) centers(1,1) + radii(1) centers(1,1) - radii(1) centers(1,2) - radii(1)];
            pixelValue = impixel(videoFrame, centers(1), centers(2))./255;
            color = rgb2name(pixelValue);
            videoFrame = insertShape(videoFrame, 'Filledcircle', [centers, radii(1)], 'color', 'white',...
                                      'opacity', 0.4);
            display(color);
        end
        
           
            
        
        
%         if ~isempty(bbox)
%             % Find corner points inside the detected region.
%             points = detectMinEigenFeatures(videoFrameCanny, 'ROI', bbox(1, :));
% 
%             % Re-initialize the point tracker.
%             xyPoints = points.Location;
%             numPts = size(xyPoints,1);
%             release(pointTracker);
%             initialize(pointTracker, xyPoints, videoFrameGray);
% 
%             % Save a copy of the points.
%             oldPoints = xyPoints;
% 
%             % Convert the rectangle represented as [x, y, w, h] into an
%             % M-by-2 matrix of [x,y] coordinates of the four corners. This
%             % is needed to be able to transform the bounding box to display
%             % the orientation of the face.
%             bboxPoints = bbox2points(bbox(1, :));
% 
%             % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
%             % format required by insertShape.
%             bboxPolygon = reshape(bboxPoints', 1, []);
% 
%             % Display a bounding box around the detected face.
%             videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
% 
%             % Display detected corners.
%             videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
%         end
% 
%     else
%         % Tracking mode.
%         [xyPoints, isFound] = step(pointTracker, videoFrameGray);
%         visiblePoints = xyPoints(isFound, :);
%         oldInliers = oldPoints(isFound, :);
% 
%         numPts = size(visiblePoints, 1);
% 
%         if numPts >= 10
%             % Estimate the geometric transformation between the old points
%             % and the new points.
%             [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
%                 oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
% 
%             % Apply the transformation to the bounding box.
%             bboxPoints = transformPointsForward(xform, double(bboxPoints));
% 
%             % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
%             % format required by insertShape.
%             bboxPolygon = reshape(bboxPoints', 1, []);
% 
%             % Display a bounding box around the face being tracked.
%             videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
% 
%             % Display tracked points.
%             videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');
% 
%             % Reset the points.
%             oldPoints = visiblePoints;
%             setPoints(pointTracker, oldPoints);
%         end

     end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(blobDetector);
