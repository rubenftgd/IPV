%%Lab1 PIV 26-09-2016
clear;close all;

%1.1
%RGB 
IM = imread('rgb_image_10.png'); %ler imagem
tamanho = size(IM); %dimensoes da imagem
figure;
imshow(IM);
title('RGB image');
%DEPTH
load depth_10.mat
figure;
imagesc(depth_array);%o que é que isto faz?
title('Depth image');
%XY sao as coordenadas da imagem, index é a depth(cordenada da perfundidade)
% imagesc(C) displays the data in array C as an image that uses the full
% range of colors in the colormap. Each element of C specifies the color 
% for 1 pixel of the image. The resulting image is an m-by-n grid of pixels
% where m is the number of columns and n is the number of rows in C.
% The row and column indices of the elements determine the centers of the corresponding pixels.

%%
%1.2
figure;
B = reshape(IM,480,640*3);
imagesc(reshape(IM,480,640*3));%R,G,B
title('R image,                 G image,             B image');

%%
%2
%a
figure;
%colormap(gray);
IM2 = rgb2gray(IM);
imshow(IM2);
title('Depth as image');

%b
figure;
mesh(IM2);
title('Depth as a surface');

%c
load xyz.mat;
p = pointCloud(xyz);

figure;
showPointCloud(p);
title('Point cloud');
figure;
mesh(depth_array);
title('Mesh');
%Qual a diferenca entre  mesh im2 e o mesh depth image?

%d - a primeira figura e a 3a figura

%Qual é a melhor representacao??

