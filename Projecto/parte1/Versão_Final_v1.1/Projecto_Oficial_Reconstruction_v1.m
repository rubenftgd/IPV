function [pcloud, transforms]=Projecto_Oficial_Reconstruction_v1(depth_array,rgb_array, depth_cam, rgb_cam, Rdtrgb, Tdtrgb)
%UNTITLED2 Summary of this function goes here
%Detailed explanation goes here

run ('vlfeat-0.9.20\toolbox\vl_setup');

im1g = rgb2gray(image_names.rgb.rgb_image_1);
im2g = rgb2gray(image_names.rgb.rgb_image_2);
im3g = rgb2gray(image_names.rgb.rgb_image_3);
im4g = rgb2gray(image_names.rgb.rgb_image_4);
im5g = rgb2gray(image_names.rgb.rgb_image_5);

xyz1 = get_xyzasus(image_names.depth.depth_1.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);
xyz2 = get_xyzasus(image_names.depth.depth_2.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);
xyz3 = get_xyzasus(image_names.depth.depth_3.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);
xyz4 = get_xyzasus(image_names.depth.depth_4.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);
xyz5 = get_xyzasus(image_names.depth.depth_5.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);

cl1 = reshape(image_names.rgb.rgb_image_1, 480*640, 3);
cl2 = reshape(image_names.rgb.rgb_image_2, 480*640, 3);
cl3 = reshape(image_names.rgb.rgb_image_3, 480*640, 3);
cl4 = reshape(image_names.rgb.rgb_image_4, 480*640, 3);
cl5 = reshape(image_names.rgb.rgb_image_5, 480*640, 3);
cl12 = [cl1; cl2];
cl23 = [cl12; cl3];
cl34 = [cl23; cl4];

%KeyPoints Detection
PeakThresh= 1;%REVER
edge_thresh = 3;%REVER
ws = 2;

[F, D] = vl_sift(single(im1g), 'edgethresh', edge_thresh, 'PeakThresh',PeakThresh ,'WindowSize', ws);

[F2, D2] = vl_sift(single(im2g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);

[F3, D3] = vl_sift(single(im3g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);

[F4, D4] = vl_sift(single(im4g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);

[F5, D5] = vl_sift(single(im5g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);

%Find Matches 1 2 
th = 7.5;%REVER
[MATCHES, ~] = vl_ubcmatch(D, D2, th); 
[~, n] = size(MATCHES);
for i=1:n
u1(i) = F(1,MATCHES(1,i));
v1(i) = F(2,MATCHES(1,i));
u2(i) = F2(1,MATCHES(2,i));
v2(i) = F2(2,MATCHES(2,i));
end

%Find Matches 2 3 
th = 7.5;%REVER
[MATCHES2, ~] = vl_ubcmatch(D2, D3, th); 
[~, n2] = size(MATCHES2);
for i=1:n2
u3(i) = F2(1,MATCHES2(1,i));
v3(i) = F2(2,MATCHES2(1,i));
u4(i) = F3(1,MATCHES2(2,i));
v4(i) = F3(2,MATCHES2(2,i));
end

%Find Matches 3 4 
th = 7.5;%REVER
[MATCHES3, ~] = vl_ubcmatch(D3, D4, th); 
[~, n3] = size(MATCHES3);
for i=1:n3
u5(i) = F3(1,MATCHES3(1,i));
v5(i) = F3(2,MATCHES3(1,i));
u6(i) = F4(1,MATCHES3(2,i));
v6(i) = F4(2,MATCHES3(2,i));
end

%Find Matches 4 5 
th = 7.5;%REVER
[MATCHES4, ~] = vl_ubcmatch(D4, D5, th); 
[~, n4] = size(MATCHES4);
for i=1:n4
u7(i) = F4(1,MATCHES4(1,i));
v7(i) = F4(2,MATCHES4(1,i));
u8(i) = F5(1,MATCHES4(2,i));
v8(i) = F5(2,MATCHES4(2,i));
end

%Print Matches antes do ransac 1 2
ind1 = sub2ind([480 640],uint64(v1'),uint64(u1'));
ind2 = sub2ind([480 640],uint64(v2'),uint64(u2'));
[~, n] = size (u1);

%Print Matches antes do ransac 2 3
ind3 = sub2ind([480 640],uint64(v3),uint64(u3));
ind4 = sub2ind([480 640],uint64(v4),uint64(u4));
[~, n2] = size (u3);

%Print Matches antes do ransac 3 4
ind5 = sub2ind([480 640],uint64(v5),uint64(u5));
ind6 = sub2ind([480 640],uint64(v6),uint64(u6));
[~, n3] = size (u5);

%Print Matches antes do ransac 4 5
ind7 = sub2ind([480 640],uint64(v7),uint64(u7));
ind8 = sub2ind([480 640],uint64(v8),uint64(u8));
[~, n4] = size (u7);

%Calculo Matriz Rotacao 1 2
cent1 = mean(xyz1(ind1,:))';
cent2 = mean(xyz2(ind2,:))';
pc1 = xyz1(ind1,:)'-repmat(cent1,1,n);
pc2 = xyz2(ind2,:)'-repmat(cent2,1,n);
[a , ~, c] = svd(pc2*pc1');
R12 = a*c';

%Calculo Matriz Rotacao 2 3
cent3=mean(xyz2(ind3,:))';
cent4=mean(xyz3(ind4,:))';
pc3=xyz2(ind3,:)'-repmat(cent3,1,n2);
pc4=xyz3(ind4,:)'-repmat(cent4,1,n2);
[a , ~, c]=svd(pc4*pc3');
R23 = a*c';

%Calculo Matriz Rotacao 3 4
cent5=mean(xyz3(ind5,:))';
cent6=mean(xyz4(ind6,:))';
pc5=xyz3(ind5,:)'-repmat(cent5,1,n3);
pc6=xyz4(ind6,:)'-repmat(cent6,1,n3);
[a , ~, c]=svd(pc6*pc5');
R34=a*c';

%Calculo Matriz Rotacao 4 5
cent7=mean(xyz4(ind7,:))';
cent8=mean(xyz5(ind8,:))';
pc7=xyz4(ind7,:)'-repmat(cent7,1,n4);
pc8=xyz5(ind8,:)'-repmat(cent8,1,n4);
[a , ~, c]=svd(pc8*pc7');
R45=a*c';

%Juntar Pointclouds 1 2
xyzt1=R12*(xyz1'-repmat(cent1,1,length(xyz1)));%Estou a juntar a pointCould1 à pointCloud2 no ref 2!
xyzt2=xyz2'-repmat(cent2,1,length(xyz2));
xyzt12 = [xyzt1 xyzt2];

%Tenho de somar cent2 porque estou a aplicar uma rotacao em xyzt12
%em cujo referencial estava no centro de massa dos pontos pc2(cent2)
xyzt3=(R23*(xyzt12-repmat(cent3,1,length(xyzt12))+repmat(cent2,1,length(xyzt12))));%Estou a juntar a pointCould12 à pointCloud3 no ref 3!
xyzt4=xyz3'-repmat(cent4,1,length(xyz3));
xyzt34= [xyzt3 xyzt4];

xyzt5=(R34*(xyzt34-repmat(cent5,1,length(xyzt34))+repmat(cent4,1,length(xyzt34))));%Estou a juntar a pointCould123 à pointCloud4 no ref 4!
xyzt6=xyz4'-repmat(cent6,1,length(xyz4));
xyzt56= [xyzt5 xyzt6];

xyzt7=(R45*(xyzt56-repmat(cent7,1,length(xyzt56))+repmat(cent6,1,length(xyzt56))));
xyzt8=xyz5'-repmat(cent8,1,length(xyz5));

%PointCloud Final 2 3
pcloud = pointCloud([xyzt7';xyzt8'],'Color',[cl34;cl5]);
figure
showPointCloud(pcloud);
transforms = 0;
end

