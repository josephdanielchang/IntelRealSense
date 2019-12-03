clear;close all;clc;

% declare video name and open it
writerObj = VideoWriter('05-21-19_Nissan.avi');
% declare frame rate, the data we record is 30 fps
writerObj.FrameRate = 30;
open(writerObj)
% define each frame
for idx=1:1000
    rgb = im2double(imread(sprintf('C:/Users/josep/OneDrive/Documents/Research/Assignment 4 Depth_Estimation/5G Forum Demo/05-21-19_Nissan_left_output_1000/scaled_rgb/cam_left_rgb_%05i.png',idx)));
    disp = repmat(double(imread(sprintf('C:/Users/josep/OneDrive/Documents/Research/Assignment 4 Depth_Estimation/5G Forum Demo/05-21-19_Nissan_left_output_1000/disp/cam_left_depth_%05i.png',idx)))/255,1,1,1);
    frame = [rgb,disp];
    writeVideo(writerObj, frame);
end
% close video
close(writerObj);