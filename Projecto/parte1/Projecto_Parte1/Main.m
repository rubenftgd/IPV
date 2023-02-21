close all
clear all
load calib_asus.mat

%Mudar aqui o path para a pasta do vosso PC
datadir = 'C:\Users\João Pedro Rosa\Desktop\PIV\Projecto_Parte1\Dataset1\'; % mudar aqui para Dataset1, 2, etc.
depthfiles = dir([datadir 'depth_*.mat']); 
rgbfiles = dir([datadir 'rgb*.png']);

for i = 1:length(depthfiles),
    image_names(i).depth = [datadir depthfiles(i).name];
    image_names(i).rgb = [datadir rgbfiles(i).name];
end
 
[pcloud, transforms] = Reconstruction(image_names, Depth_cam.K, RGB_cam.K, R_d_to_rgb, T_d_to_rgb);
% pc=pointCloud(pcloud(:,1:3),'Color',uint8(pcloud(:,4:6)));
showPointCloud(pcloud);
