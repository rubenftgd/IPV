%Main que declara inputs e chama a função
clc;
close all;
clear all;
load calib_asus.mat;
K=[525 0 319.5;
    0 525 239.5;
    0 0 1]; %
% 'depth_1', load('Depths2\depth_1'), 
depth_array = struct ( 'depth_1', load('Depths2\depth_1'),'depth_2', load('Depths2\depth_2'), ...
'depth_3', load('Depths2\depth_3'), 'depth_4', load('Depths2\depth_4'),'depth_5', load('Depths2\depth_5'),...
'depth_6', load('Depths2\depth_6'),'depth_7', load('Depths2\depth_7'),'depth_8', load('Depths2\depth_8'),...
'depth_9', load('Depths2\depth_9'));
% depth_array.depth_1;
field = 'd';
value = {depth_array.depth_1;
    depth_array.depth_2;
    depth_array.depth_3;
    depth_array.depth_4;
    depth_array.depth_5;
    depth_array.depth_6;
    depth_array.depth_7;
    depth_array.depth_8;
    depth_array.depth_9;};
    
depth_array = struct(field,value);
% 'rgb_image_1', imread('Images2\rgb_image_1.png'),
rgb_array = struct( 'rgb_image_1', imread('Images2\rgb_image_1.png'), 'rgb_image_2', imread('Images2\rgb_image_2.png'), ...
'rgb_image_3', imread('Images2\rgb_image_3.png'), 'rgb_image_4', imread('Images2\rgb_image_4.png'), ...
'rgb_image_5', imread('Images2\rgb_image_5.png'),'rgb_image_6', imread('Images2\rgb_image_6.png'),'rgb_image_7', imread('Images2\rgb_image_7.png'),...
'rgb_image_8', imread('Images2\rgb_image_8.png'), 'rgb_image_9', imread('Images2\rgb_image_9.png'));
% rgb_array.rgb_image_1
field = 'rgb';
value = {rgb_array.rgb_image_1
    rgb_array.rgb_image_2;
    rgb_array.rgb_image_3;
    rgb_array.rgb_image_4;
    rgb_array.rgb_image_5;
    rgb_array.rgb_image_6;
    rgb_array.rgb_image_7;
    rgb_array.rgb_image_8;
    rgb_array.rgb_image_9;};
    
rgb_array = struct(field,value);

image_names = struct ('depth', depth_array, 'rgb', rgb_array);


%%
%invocar a função
[pcloud, transforms] = Projecto_Oficial_Reconstruction(depth_array,rgb_array, Depth_cam, RGB_cam, R_d_to_rgb, T_d_to_rgb);