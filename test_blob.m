%% Detect and Measure Circular Objects in an Image
% This example shows how to use |imfindcircles| to automatically detect
% circles or circular objects in an image. It also shows the use of
% |viscircles| to visualize the detected circles.

% Copyright 2012-2014 The MathWorks, Inc.
clc; clear;
%% Step 1: Load Image
% This example uses an image of round plastic chips of various colors.
tmp = imread('buoy_02.jpg');
rgb = imresize(tmp, 0.7);
figure
imshow(rgb)
radius = [140 150];

%% 
% Besides having plenty of circles to detect, there are a few interesting
% things going on in this image from a circle detection point-of-view:
% 
% # There are chips of different colors, which have different contrasts
% with respect to the background. On one end, the blue and red ones have
% strong contrast on this background. On the other end, some of the yellow
% chips do not contrast well with the background.
% # Notice how some chips are on top of each other and some others that are
% close together and almost touching each other. Overlapping object
% boundaries and object occlusion are usually challenging scenarios for
% object detection.

%% Step 2: Determine Radius Range for Searching Circles
% |imfindcircles| needs a radius range to search for the circles. A quick
% way to find the appropriate radius range is to use the interactive tool
% |imdistline| to get an approximate estimate of the radii of various
% objects.
d = imdistline;


%% 
% |imdistline| creates a draggable tool that can be moved to fit across a
% chip and the numbers can be read to get an approximate estimate of its
% radius. Most chips have radius in the range of 21-23 pixels. Use a
% slightly larger radius range of 20-25 pixels just to be sure. Before that
% remove the |imdistline| tool.
delete(d);

%% Step 3: Initial Attempt to Find Circles
% Call |imfindcircles| on this image with the search radius of [20 25]
% pixels. Before that, it is a good practice to ask whether the objects are
% brighter or darker than the background. To answer that question, look at
% the grayscale version of this image.

gray_image = rgb2gray(rgb);
J = imnoise(gray_image ,'salt & pepper',0.02);
imshow(J);
Kaverage = filter2(fspecial('average',3),J)/255;
figure
imshow(Kaverage)
Kmedian = medfilt2(J);
imshowpair(Kaverage,Kmedian,'montage')
gray_image = edge(gray_image, 'canny');
figure;
[Gmag,Gdir] = imgradient(gray_image);
imshowpair(gray_image, Gmag, 'montage');


%%
% The background is quite bright and most of the chips are darker than the
% background. But, by default, |imfindcircles| finds circular objects that
% are brighter than the background. So, set the parameter 'ObjectPolarity'
% to 'dark' in |imfindcircles| to search for dark circles.

%[centers, radii] = imfindcircles(rgb, radius); %,'ObjectPolarity','dark')

%% 
% Note that the outputs |centers| and |radii| are empty, which means that
% no circles were found. This happens frequently because |imfindcircles| is
% a circle _detector_, and similar to most detectors, |imfindcircles| has
% an internal _detection threshold_ that determines its sensitivity. In
% simple terms it means that the detector's confidence in a certain
% (circle) detection has to be greater than a certain level before it is
% considered a _valid_ detection. |imfindcircles| has a parameter
% 'Sensitivity' which can be used to control this internal threshold, and
% consequently, the sensitivity of the algorithm. A higher 'Sensitivity'
% value sets the detection threshold lower and leads to detecting more
% circles. This is similar to the sensitivity control on the motion
% detectors used in home security systems.

%% Step 4: Increase Detection Sensitivity
% Coming back to the chip image, it is possible that at the default
% sensitivity level all the circles are lower than the internal threshold,
% which is why no circles were detected. By default, 'Sensitivity', which
% is a number between 0 and 1, is set to 0.85. Increase 'Sensitivity' to
% 0.9.

[centers, radii] = imfindcircles(rgb,radius,'ObjectPolarity','dark', ...
    'Sensitivity',0.95);

%% 
% This time |imfindcircles| found some circles - eight to be precise.
% |centers| contains the locations of circle centers and |radii| contains
% the estimated radii of those circles.
 
%% Step 5: Draw the Circles on the Image
% The function |viscircles| can be used to draw circles on the image.
% Output variables |centers| and |radii| from |imfindcircles| can be passed
% directly to |viscircles|.

imshow(rgb);

h = viscircles(centers,radii);
pixelValue = impixel(rgb, centers(1), centers(2))./255;
color = rgb2name(pixelValue);
viscircles(centers,radii);