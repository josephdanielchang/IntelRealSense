% Code requires altering if depth is rgb instead of grayscale

clear;close all;clc;

% declare video name and open it
writerObj = VideoWriter('20191114_130938');
% declare frame rate
writerObj.FrameRate = 30;   % CHANGE fps
open(writerObj)
disp = zeros(480,640,3);    % CHANGE resolution
% define each frame
for idx=0:1000
    rgb = im2double(imread(sprintf('C:\\Users\\josep\\OneDrive\\Documents\\Research_Fall_2019\\Processed Data\\20191114_130938\\rgb\\rgb%06i.png',idx)));
    disp(:,:,1) = imread(sprintf('C:\\Users\\josep\\OneDrive\\Documents\\Research_Fall_2019\\Processed Data\\20191114_130938\\intel_depth\\depth%06i.png',idx));
    disp = double(disp);
    disp = mat2gray(disp);
    frame = [rgb,disp];
    writeVideo(writerObj, frame);
end
% close video
close(writerObj);