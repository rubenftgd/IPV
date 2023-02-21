%Main que declara inputs e chama a função
clc;
close all;
clear all;
load calib_asus.mat;


depth_array = struct ('depth_1', load('Depths\depth_1'), 'depth_2', load('Depths\depth_2'), ...
'depth_3', load('Depths\depth_3'), 'depth_4', load('Depths\depth_4'),'depth_5', load('Depths\depth_5'));

field = 'd';
value = {depth_array.depth_1;
    depth_array.depth_2;
    depth_array.depth_3;
    depth_array.depth_4;
    depth_array.depth_5;};
depth_array = struct(field,value);

rgb_array = struct('rgb_image_1', imread('Images\rgb_image_1.png'), 'rgb_image_2', imread('Images\rgb_image_2.png'), ...
'rgb_image_3', imread('Images\rgb_image_3.png'), 'rgb_image_4', imread('Images\rgb_image_4.png'), ...
'rgb_image_5', imread('Images\rgb_image_5.png'));

field = 'rgb';
value = {rgb_array.rgb_image_1;
    rgb_array.rgb_image_2;
    rgb_array.rgb_image_3;
    rgb_array.rgb_image_4;
    rgb_array.rgb_image_5;};
rgb_array = struct(field,value);

image_names = struct ('depth', depth_array, 'rgb', rgb_array);


%%
%invocar a função
[pcloud, transforms] = Projecto_Oficial_Reconstruction(depth_array,rgb_array, Depth_cam, RGB_cam, R_d_to_rgb, T_d_to_rgb);