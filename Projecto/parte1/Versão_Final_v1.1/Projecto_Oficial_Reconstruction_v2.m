function [pcloud, transforms]=Projecto_Oficial_Reconstruction_v2(depth_array,rgb_array, depth_cam, rgb_cam, Rdtrgb, Tdtrgb)
%UNTITLED2 Summary of this function goes here
%Detailed explanation goes here

run ('vlfeat-0.9.20\toolbox\vl_setup');

im1g = rgb2gray(rgb_array(1).rgb);
im2g = rgb2gray(rgb_array(2).rgb);
xyz1 = get_xyzasus(depth_array(1).d.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);
xyz2 = get_xyzasus(depth_array(2).d.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);

rgbd1=get_rgbd(xyz1,rgb_array(1).rgb,Rdtrgb,Tdtrgb,rgb_cam.K);
rgbd2=get_rgbd(xyz2,rgb_array(2).rgb,Rdtrgb,Tdtrgb,rgb_cam.K);

cl1 = reshape(rgbd1, 480*640, 3);
cl2 = reshape(rgbd2, 480*640, 3);
cl12 = [cl1; cl2];
%KeyPoints Detection
PeakThresh= 1;%REVER
edge_thresh = 3;%REVER
ws = 2;
[F, D] = vl_sift(single(im1g), 'edgethresh', edge_thresh, 'PeakThresh',PeakThresh ,'WindowSize', ws);
[F2, D2] = vl_sift(single(im2g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);
%Find Matches 1 2 
th = 4;%REVER
[MATCHES, ~] = vl_ubcmatch(D, D2, th); 
[~, n] = size(MATCHES);
for i=1:n
u1(i) = F(1,MATCHES(1,i));
v1(i) = F(2,MATCHES(1,i));
u2(i) = F2(1,MATCHES(2,i));
v2(i) = F2(2,MATCHES(2,i));
end
%Print Matches antes do ransac 1 2
ind1 = sub2ind([480 640],uint64(v1'),uint64(u1'));
ind2 = sub2ind([480 640],uint64(v2'),uint64(u2'));
[~, n] = size (u1);
%Calculo Matriz Rotacao 1 2
cent1 = mean(xyz1(ind1,:))';
cent2 = mean(xyz2(ind2,:))';
pc1 = xyz1(ind1,:)'-repmat(cent1,1,n);
pc2 = xyz2(ind2,:)'-repmat(cent2,1,n);
[a , ~, c] = svd(pc2*pc1');
R12 = a*c';
%Juntar Pointclouds 1 2
xyzt1=R12*(xyz1'-repmat(cent1,1,length(xyz1)));%Estou a juntar a pointCould1 à pointCloud2 no ref 2!
xyzt2=xyz2'-repmat(cent2,1,length(xyz2));
xyzt12 = [xyzt1 xyzt2];

%PointCloud Final 1 2
pcloud = pointCloud(xyzt12','Color',cl12);
figure
showPointCloud(pcloud);
%
nims = 4;
for i=2:nims
    im2g = rgb2gray(rgb_array(i).rgb);
    im3g = rgb2gray(rgb_array(i+1).rgb);
    xyz2 = get_xyzasus(depth_array(i).d.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);
    xyz3 = get_xyzasus(depth_array(i+1).d.depth_array(:), [480 640], 1:640*480, depth_cam.K, 1, 0);
    
    rgbd2=get_rgbd(xyz1,rgb_array(i).rgb,Rdtrgb,Tdtrgb,rgb_cam.K);
    rgbd3=get_rgbd(xyz2,rgb_array(i+1).rgb,Rdtrgb,Tdtrgb,rgb_cam.K);

    cl2 = reshape(rgbd2, 480*640, 3);
    cl3 = reshape(rgbd3, 480*640, 3);
    cl12 = [cl12; cl3];
    
    [F2, D2] = vl_sift(single(im2g), 'edgethresh', edge_thresh, 'PeakThresh',PeakThresh ,'WindowSize', ws);
    [F3, D3] = vl_sift(single(im3g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);
%    Find Matches 2 3 

    [MATCHES2, ~] = vl_ubcmatch(D2, D3, th); 
    [~, n2] = size(MATCHES2);
    u3=0;v3=0;u4=0;v4=0;
    for j=1:n2
        u3(j) = F2(1,MATCHES2(1,j));
        v3(j) = F2(2,MATCHES2(1,j));
        u4(j) = F3(1,MATCHES2(2,j));
        v4(j) = F3(2,MATCHES2(2,j));
    end
    
    figure;imagesc(im2g);hold on;plot(u3,v3,'*r');hold off;
    figure;imagesc(im3g);hold on;plot(u4,v4,'*r');hold off;
    %Print Matches antes do ransac 2 3
    ind3 = sub2ind([480 640],uint64(v3),uint64(u3));
    ind4 = sub2ind([480 640],uint64(v4),uint64(u4));
    [~, n2] = size (u3);
    %Calculo Matriz Rotacao 2 3
    cent3=mean(xyz2(ind3,:))';
    cent4=mean(xyz3(ind4,:))';
    pc3=xyz2(ind3,:)'-repmat(cent3,1,n2);
    pc4=xyz3(ind4,:)'-repmat(cent4,1,n2);
    [a , ~, c]=svd(pc4*pc3');
    R23 = a*c';
    
    %Tenho de somar cent2 porque estou a aplicar uma rotacao em xyzt12
    %em cujo referencial estava no centro de massa dos pontos pc2(cent2)
    xyzt3=(R23*(xyzt12-repmat(cent3,1,length(xyzt12))+repmat(cent2,1,length(xyzt12))));%Estou a juntar a pointCould12 à pointCloud3 no ref 3!
    xyzt4=xyz3'-repmat(cent4,1,length(xyz3));
    xyzt12= [xyzt3 xyzt4];
    
    %PointCloud Final 2 3
    i;
    size(cl12)
    size(xyzt12);
    
    pcloud = pointCloud(xyzt12','Color',cl12);
    figure
    showPointCloud(pcloud);
    cent2 = cent4;
    transforms = 0;
end