close all
clear all
load calib_asus.mat

datadir = 'C:\Users\João Pedro Rosa\Desktop\PIV\Versão_Final_v1.1\';
depthfiles = dir([datadir '\Depths2\depth*.mat']); % mudar aqui para Depths, Depths2, Depths3, etc. e mudar Nims em Recons.
rgbfiles = dir([datadir '\Images2\rgb*.png']); % mudar aqui para Images, Images2, Images3, etc.

for i = 1:length(depthfiles),
    image_names(i).depth = ['C:\Users\João Pedro Rosa\Desktop\PIV\Versão_Final_v1.1\Depths2\' depthfiles(i).name];
    image_names(i).rgb = ['C:\Users\João Pedro Rosa\Desktop\PIV\Versão_Final_v1.1\Images2\' rgbfiles(i).name];
end
 
[pcloud, transforms] = Projecto_Oficial_Reconstruction_v3(image_names, Depth_cam.K, RGB_cam.K, R_d_to_rgb, T_d_to_rgb);
% pc=pointCloud(pcloud(:,1:3),'Color',uint8(pcloud(:,4:6)));
showPointCloud(pcloud);
