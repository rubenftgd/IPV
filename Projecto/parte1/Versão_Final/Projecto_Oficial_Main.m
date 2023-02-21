%Main que declara inputs e chama a fun��o

clear all
load calib_asus.mat;

depth_array = struct ('depth_1', load('Depths\depth_1'), 'depth_2', load('Depths\depth_2'), ...
'depth_3', load('Depths\depth_3'), 'depth_4', load('Depths\depth_4'),'depth_5', load('Depths\depth_5'));

rgb_array = struct('rgb_image_1', imread('Images\rgb_image_1.png'), 'rgb_image_2', imread('Images\rgb_image_2.png'), ...
'rgb_image_3', imread('Images\rgb_image_3.png'), 'rgb_image_4', imread('Images\rgb_image_4.png'), ...
'rgb_image_5', imread('Images\rgb_image_5.png'));

image_names = struct ('depth', depth_array, 'rgb', rgb_array);

%invocar a fun��o
[pcloud, transforms] = Projecto_Oficial_Reconstruction(image_names, Depth_cam, RGB_cam, R_d_to_rgb, T_d_to_rgb);