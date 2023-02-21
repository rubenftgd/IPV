%% %%%%%%% CODIGO BASE DO PROF %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
load('car_data.mat');

K=[525 0 319.5;
    0 525 239.5;
    0 0 1];
depth1=imread('cardepth2.png');
im1=imread('car2.jpg');
depth2=imread('cardepth393.png');
im2=imread('car393.jpg');
figure(1);
imagesc([im1 im2]);
figure(2);
imagesc([depth1 depth2]);
xyz1=get_xyzasus(depth1(:),[480 640],1:640*480,K,1,0);
xyz2=get_xyzasus(depth2(:),[480 640],1:640*480,K,1,0);
 
cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);
 
p1=pointCloud(xyz1,'Color',cl1);
p2=pointCloud(xyz2,'Color',cl2);

%%
figure(3)
showPointCloud(p1);
figure(4)
showPointCloud(p2);

%%
m1=depth1>0;
m2=depth2>0;
 
imaux1=double(repmat(m1,[1,1,3])).*double(im1)/255;
imaux2=double(repmat(m2,[1,1,3])).*double(im2)/255;
figure(5);
imagesc(imaux1);
figure(6);
imagesc(imaux2);
% select figure with im1 and click 5 points
%[u1,v1]=ginput(5);
%select figure with im2 and click in the corresponding points
%[u2 v2]=ginput(5);
u1 =[  216.7007  324.5143 329.1022 352.0412 372.6864]';
v1 =[  53.7925 80.2550 111.8629 194.9257 78.7848]';
u2=[257.9910  224.7294  135.2670   42.3638  192.6147]';
v2=[87.6057  108.1876  124.3591  168.4632  130.9747]';
 
figure(5);hold on;plot(u1,v1,'*r');hold off;
figure(6);hold on;plot(u2,v2,'*r');hold off;
ind1=sub2ind([480 640],uint64(v1),uint64(u1));
ind2=sub2ind([480 640],uint64(v2),uint64(u2));

%% Compute Centroids
cent1=mean(xyz1(ind1,:))';
cent2=mean(xyz1(ind2,:))';
pc1=xyz1(ind1,:)'-repmat(cent1,1,5);
pc2=xyz2(ind2,:)'-repmat(cent2,1,5);
[a b c]=svd(pc2*pc1')
R12=a*c'

%%
xyzt1=R12*(xyz1'-repmat(cent1,1,length(xyz1)));
xyzt2=xyz2'-repmat(cent2,1,length(xyz2));
T=cent2-R12*cent1;

%%
ptotal=pointCloud([xyzt1';xyzt2'],'Color',[cl1;cl2]);
figure(7);
showPointCloud(ptotal);
 
%% %%%%%%%%%%%%%% CHALLENGE 1 %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
depth1=imread('cardepth2.png');
im1=imread('car2.jpg');
depth2=imread('cardepth393.png');
im2=imread('car393.jpg');
% figure(1);
% imagesc([im1 im2]);
% figure(2);
% imagesc([depth1 depth2]);

%Modelo camara: x = K [R T] X'---------------------------------------------

nPontos = 5;

%intrinsics:  K(3x3)
Ki = [525 0 319.5;
      0 525 239.5;
      0 0 1];

%extrinsics:     [R T]
R12 = [0.2917   -0.5660    0.7711;
       0.4641    0.7886    0.4033;
      -0.8364    0.2402    0.4927];

T =[  -2.1053
      -0.7045
      1.0838];
  
  Pint = [Ki zeros(3,1);zeros(1,3) 1];
  Pext = [R12 T; zeros(1,3) 1];
%--------------------------------------------------------------------------
xyz1=get_xyzasus(depth1(:),[480 640],1:640*480,Ki,1,0);
xyz2=get_xyzasus(depth2(:),[480 640],1:640*480,Ki,1,0); 

u1 =[  216.7007  324.5143 329.1022 352.0412 372.6864]';
v1 =[  53.7925 80.2550 111.8629 194.9257 78.7848]';


ind1=sub2ind([480 640],uint64(v1),uint64(u1));
pc1=xyz1(ind1,:);
Lpc1 = [pc1';ones(1,nPontos)];
for i=1:1:nPontos
    Lpc2(:,i) = Pint*Pext*Lpc1(:,i);
end


for i=1:1:nPontos
    pc2(:,i) = [Lpc2(1,i)/Lpc2(3,i); Lpc2(2,i)/Lpc2(3,i)];
end

%2D images
m1=depth1>0;
m2=depth2>0;

imaux1=double(repmat(m1,[1,1,3])).*double(im1)/255;%?
imaux2=double(repmat(m2,[1,1,3])).*double(im2)/255;
figure(1);
imagesc(imaux1);
figure(2);
imagesc(imaux2);

figure(1);hold on;plot(u1,v1,'*r');hold off;
figure(2);hold on;plot(pc2(1,:),pc2(2,:),'*y');hold off;

%3D representation
 
cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);
p1=pointCloud(xyz1,'Color',cl1);
p2=pointCloud(xyz2,'Color',cl2);
figure(3)
showPointCloud(p1);
figure(4)
showPointCloud(p2);

%% %%%%%%%%%%%%%% CHALLENGE 2 %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K=[525 0 319.5;
    0 525 239.5;
    0 0 1];

depth1=imread('cardepth2.png');
im1=imread('car2.jpg');
depth2=imread('cardepth393.png');
im2=imread('car393.jpg');
depth3 = imread('cardepth641.png');
im3 = imread('car641.jpg');


figure(1);
imagesc([im1 im2 im3]);
figure(2);
imagesc([depth1 depth2 depth3]);
xyz1=get_xyzasus(depth1(:),[480 640],1:640*480,K,1,0);
xyz2=get_xyzasus(depth2(:),[480 640],1:640*480,K,1,0);
xyz3=get_xyzasus(depth3(:),[480 640],1:640*480,K,1,0);

cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);
cl12 = [cl1;cl2];
cl3=reshape(im3,480*640,3);

p1=pointCloud(xyz1,'Color',cl1);
p2=pointCloud(xyz2,'Color',cl2);
p3=pointCloud(xyz3,'Color',cl3);

figure(3)
showPointCloud(p1);
figure(4)
showPointCloud(p2);
figure(5)
showPointCloud(p3);

m1=depth1>0;
m2=depth2>0;
m3=depth3>0;

imaux1=double(repmat(m1,[1,1,3])).*double(im1)/255;%?
imaux2=double(repmat(m2,[1,1,3])).*double(im2)/255;
imaux3=double(repmat(m3,[1,1,3])).*double(im3)/255;
figure(6);
imagesc(imaux1);
figure(7);
imagesc(imaux2);

figure(9);
imagesc(imaux3);
% select figure with im1 and click 5 points
%[u1,v1]=ginput(5);
%select figure with im2 and click in the corresponding points
%[u2 v2]=ginput(5);
u1 =[  216.7007  324.5143 329.1022 352.0412 372.6864]';
v1 =[  53.7925 80.2550 111.8629 194.9257 78.7848]';
u2=[257.9910  224.7294  135.2670   42.3638  192.6147]';
v2=[87.6057  108.1876  124.3591  168.4632  130.9747]';
figure(6);hold on;plot(u1,v1,'*r');hold off;
figure(7);hold on;plot(u2,v2,'*r');hold off;
ind1=sub2ind([480 640],uint64(v1),uint64(u1));
ind2=sub2ind([480 640],uint64(v2),uint64(u2));

u3 =[530 486 471 442 413]';%Pontos comuns entre imagem 2 e 3
v3 =[194 270 329 287 359]';
u4=[153 85 122 27 57]';
v4=[196 268 337 277 373]';
figure(7);hold on;plot(u3,v3,'*g');hold off;
figure(9);hold on;plot(u4,v4,'*g');hold off;
ind3=sub2ind([480 640],uint64(v3),uint64(u3));
ind4=sub2ind([480 640],uint64(v4),uint64(u4));

cent1=mean(xyz1(ind1,:))';
cent2=mean(xyz2(ind2,:))';
pc1=xyz1(ind1,:)'-repmat(cent1,1,5);
pc2=xyz2(ind2,:)'-repmat(cent2,1,5);
[a b c]=svd(pc2*pc1')
R12=a*c'


cent3=mean(xyz2(ind3,:))';
cent4=mean(xyz3(ind4,:))';
pc3=xyz2(ind3,:)'-repmat(cent3,1,5);
pc4=xyz3(ind4,:)'-repmat(cent4,1,5);
[a b c]=svd(pc4*pc3')
R23=a*c'


xyzt1=R12*(xyz1'-repmat(cent1,1,length(xyz1)));%Estou a juntar a pointCould1 � pointCloud2 no ref 2!
xyzt2=xyz2'-repmat(cent2,1,length(xyz2));
T=cent2-R12*cent1;

xyzt12 = [xyzt1 xyzt2];

%Tenho de somar cent2 porque estou a aplicar uma rotacao em xyzt12
%em cujo referencial estava no centro de massa dos pontos pc2(cent2)
xyzt3=(R23*(xyzt12-repmat(cent3,1,length(xyzt12))+repmat(cent2,1,length(xyzt12))));%Estou a juntar a pointCould12 � pointCloud3 no ref 3!
xyzt4=xyz3'-repmat(cent4,1,length(xyz3));
T2=cent2-R23*cent3;%Nao garanto que esta seja a verdadeira translaccao...


% ptotal=pointCloud(xyzt12','Color',cl12);
% figure(17);
% showPointCloud(ptotal);

ptotal=pointCloud([xyzt3';xyzt4'],'Color',[cl12;cl3]);
figure(20);
showPointCloud(ptotal);
