% Laboratorio 2 de PIV
% Nos temos em consideração as funções que o professor
% disponibilizou no laboratório

%Lab 2

%Tasks:

%1. Given u,v and depth compute XYZ
%2. Generate image projection from point cloud
%3. Rotate and Translate point cloud and do the same
%4. Project Point cloud in the RGB image. What about the reverse: click rgb and get the 3D!

%%%%%%%%%%%%%%%%%%%%%%%% FIRST TASK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;clc;close all;
load calib_asus.mat
load depth_10.mat
IM = imread ('rgb_image_10.png');

I=depth_array(:);
XYZ = get_xyzasus(I,[480 640], 1:480*640,Depth_cam.K,1,0);%obtem-se coordenadas 3D
%no ref depth, nao homogeneas

rgbd = get_rgbd(XYZ, IM, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);

cl = reshape(rgbd, 480*640,3);

showPointCloud(XYZ, cl);

% pcshow(XYZ);