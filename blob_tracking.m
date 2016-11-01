function [ area, color, xcoord, ycoord, frame_test ] =  blob_tracking (frame)

% area set to zero
area = 0;
xcoord = 0;
ycoord = 0;
area = 0;
frame_test = 0;
color = 0;

% hard-coded radius range (temporary)
radius = [25 35];

% read in image
image = imread(frame);

% find the size of the image
[row, col] = size(image);

% find the center and the radii of the circular object
[centers, radii] = imfindcircles(image, radius,'ObjectPolarity', 'dark','Sensitivity',0.98);

% if the program detects a circular object
if ~isempty(centers)
            
    % calculate the pixel value of the center of the circle centers(1)-col
    pixel_value = impixel(image, centers(1), centers(2))/255;
            
    % find the string name for the color
    color = rgb2name(pixel_value);
            
    % draw a white circle over the circle detected
    frame_test = insertShape(image, 'Filledcircle', [centers(1, :), radii(1, :)], 'color', 'white', 'opacity', 0.4);
            
    % calculate the area of the circle detected (in pixels)
    area = pi*radii(1)^2;
    
    % calculate the zcoord (normalized)
    xcoord = ((col/2) - centers(1,1))/(col/2);

    % calculate the ycoord (normalized)
    ycoord = ((row/2) - centers(1,2))/(row/2);
    
end
