clc; clear;

% Create the webcam object.
cam = webcam('FaceTime HD Camera (Built-in)');

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

% set the loop to run unconditionally
runLoop = true;

% initialize the number of points
numPts = 0;

% count the number of frames
frameCount = 0;

% set up master node on global network
rosinit;

% create ros node on the network
node = robotics.ros.Node('/test_node_1');

% create publisher
testpub = rospublisher('/rostest','std_msgs/String');

% run while loop for 400 frames
while runLoop && frameCount < 400

    % Get the next frame.
    videoFrame = snapshot(cam);
    
    % conver the frame to grayscale
    videoFrameGray = rgb2gray(videoFrame);
    
    % convert the frame to a binary image
    videoFrameBin = im2bw(videoFrameGray, 0.3);
    
    % reduce the size of the frame
    videoFrame = imresize(videoFrame, 0.7);
    
    % increment the frame count  
    frameCount = frameCount + 1;
    
    % find the center and the radii of the circular object
    [centers, radii] = imfindcircles(videoFrame,[100 150],'ObjectPolarity','dark', ...
                                        'Sensitivity',0.98);
    % if the program detects a circular object
    if ~isempty(centers)
            
        % calculate the bounding boxes
        bbox = [centers(1,2) - radii(1) centers(1,1) + radii(1) centers(1,1) - radii(1) centers(1,2) - radii(1)];
            
        % calculate the pixel value of the center of the circle
        pixelValue = impixel(videoFrame, centers(1), centers(2))./255;
            
        % find the string name for the color
        color = rgb2name(pixelValue);
            
        % draw a white circle over the circle detected
        videoFrame = insertShape(videoFrame, 'Filledcircle', [centers(1, :), radii(1, :)], 'color', 'white',...
                                      'opacity', 0.4);
            
        % calculate the area of the circle detected (in pixels)
        area = pi*radii(1)^2;
            
        % conver the area to a string value (to communicate with ROS)
        str_area = num2str(area);
            
        % set up a message for the publisher
        msg = rosmessage(testpub);
            
        % give the message the string that needs to be sent
        msg.Data = [color str_area];
            
        % send the message
        send(testpub,msg);

        display([color ' ' str_area]);
    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);