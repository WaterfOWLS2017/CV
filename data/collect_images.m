

vid = videoinput('pointgrey', 1, 'F7_BayerBG8_640x480_Mode3');
image = getsnapshot(vid);
[im_row, im_col] = size(image);

for i=21:30
    image = getsnapshot(vid);
    imshow(image);
    pause;
    % splits the image the two camera shots
    %
    right_image = image(:,1:(im_col/6),:);
    
    left_image = image(:,im_col/6:(im_col/3),:);
    
    imwrite(right_image, ['right_', num2str(i), '.png'],'png');
    imwrite(right_image, ['left_', num2str(i), '.png'],'png');
    
end
