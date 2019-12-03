% rename file name
% start index from one

clear;close all;clc;

image_dir = dir('C:/Users/josep/OneDrive/Documents/Research/Assignment 5 Calibration_Rectification/05-21-19/right_data/right_rgb/*.png');

for idx = 1:length(image_dir)
%for idx = 1
    img = imread(sprintf('%s/%s',image_dir(idx).folder, image_dir(idx).name));
    %Custom output file name
    imwrite(img,sprintf('C:/Users/josep/OneDrive/Documents/Research/Assignment 5 Calibration_Rectification/05-21-19/right_data/right_rgb_renamed/cam_right_rgb_%05i.png', idx));
end
