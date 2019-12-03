%Enter in command window: stereoCameraCalibrator
%Checkerboard size: 50mm

clear;close all;clc;

l_image_dir = dir('05-21-19/left_data/left_rgb/*.png');
r_image_dir = dir('05-21-19/right_data/right_rgb/*.png');

load('05-17-19/Nissan/nissanStereoParams.mat');

for idx = 1:length(l_image_dir)
%for idx = 1
    l_img = imread(sprintf('%s/%s',l_image_dir(idx).folder, l_image_dir(idx).name));
    r_img = imread(sprintf('%s/%s',r_image_dir(idx).folder, r_image_dir(idx).name));
    [rec_l_img, rec_r_img] = rectifyStereoImages(l_img, r_img, stereoParams, 'OutputView', 'valid');
    %Copy input file name
    %imwrite(rec_l_img,sprintf('05-17-19/Accord/left_check_0517/left_check_11_rect/%s',l_image_dir(idx).name));
    %imwrite(rec_r_img,sprintf('05-17-19/Accord/right_check_0517/right_check_11_rect/%s',r_image_dir(idx).name));
    %Custom output file name
    imwrite(rec_l_img,sprintf('05-21-19/left_data/left_rect_rgb/cam_left_rgb_frame_%05i.png', idx));
    imwrite(rec_r_img,sprintf('05-21-19/right_data/right_rect_rgb/cam_right_rgb_frame_%05i.png', idx));
end
