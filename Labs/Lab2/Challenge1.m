%CHALENGE1
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

%%
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

%%
%3D representation
 
cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);
p1=pointCloud(xyz1,'Color',cl1);
p2=pointCloud(xyz2,'Color',cl2);
figure(3)
showPointCloud(p1);
figure(4)
showPointCloud(p2);