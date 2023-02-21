close all
clear all

datadir = '.\Training\';
depthfiles = dir([datadir 'depth_*.mat']); 
rgbfiles = dir([datadir 'rgb*.png']);

for i = 1:length(depthfiles),
    training_image_names(i).depth = [datadir depthfiles(i).name];
    training_image_names(i).rgb = [datadir rgbfiles(i).name];
end

datadir = '.\testing1\';
depthfiles = dir([datadir 'depth_*.mat']); 
rgbfiles = dir([datadir 'rgb*.png']);

for i = 1:length(depthfiles),
    test_image_names(i).depth = [datadir depthfiles(i).name];
    test_image_names(i).rgb = [datadir rgbfiles(i).name];
end

bookindex = books( test_image_names, training_image_names)