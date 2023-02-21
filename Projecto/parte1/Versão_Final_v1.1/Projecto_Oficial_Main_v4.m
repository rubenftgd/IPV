%Main que declara inputs e chama a função
clc;
close all;
clear all;
load calib_asus.mat;
K=[525 0 319.5;
    0 525 239.5;
    0 0 1]; %
% 'depth_1', load('Depths2\depth_1'), 

% datadir='C:\Users\João Pedro Rosa\Desktop\PIV\Versão_Final_v1.1';%change this for different datasets
% depth_array=dir([datadir 'depth*.mat']);%change this for different datasets
% rgb_array = dir([datadir 'rgb*.png']);%change this for different datasets
% for i=1:length(depthfiles),
%     image_names(i).depth=[datadir depthfiles(i).name];
%     image_names(i).rgb=[datadir rgbfiles(i).name];
% end


depth_array = struct ( 'depth_1', load('Depths3\depth_1'),'depth_2', load('Depths3\depth_2'), ...
'depth_3', load('Depths3\depth_3'));
% depth_array.depth_1;
field = 'd';
value = {depth_array.depth_1;
    depth_array.depth_2;
    depth_array.depth_3;};
    
depth_array = struct(field,value);
% 'rgb_image_1', imread('Images2\rgb_image_1.png'),
rgb_array = struct( 'rgb_image_1', imread('Images3\rgb_image_1.png'), 'rgb_image_2', imread('Images3\rgb_image_2.png'), ...
'rgb_image_3', imread('Images3\rgb_image_3.png'));
% rgb_array.rgb_image_1
field = 'rgb';
value = {rgb_array.rgb_image_1
    rgb_array.rgb_image_2;
    rgb_array.rgb_image_3;};
    
rgb_array = struct(field,value);

image_names = struct ('depth', depth_array, 'rgb', rgb_array);


%%
%invocar a função
[pcloud, transforms] = Projecto_Oficial_Reconstruction_v3(depth_array,rgb_array, Depth_cam, RGB_cam, R_d_to_rgb, T_d_to_rgb);