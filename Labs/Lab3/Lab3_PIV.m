%Check this code

 

K=[525 0 319.5;

    0 525 239.5;

    0 0 1];

 

depth1=imread('cardepth2.png');

im1=imread('car2.jpg');

depth2=imread('cardepth393.png');

im2=imread('car393.jpg');

imagesc([im1 im2]);

figure;

imagesc([depth1 depth2]);

xyz1=get_xyzasus(depth1(:),[480 640],1:640*480,K,1,0);

xyz2=get_xyzasus(depth2(:),[480 640],1:640*480,K,1,0);

 

cl1=reshape(im1,480*640,3);

cl2=reshape(im2,480*640,3);

 

p1=pointCloud(xyz1,'Color',cl1);
%p1=showPointCloud(xyz1,cl1);
%p2=pointCloud(xyz2,'Color',cl2);
%p2=showPointCloud(xyz2,cl2);


%%

figure(1)

showPointCloud(p1);

figure(2)

showPointCloud(p2);

%%

m1=depth1>0;

m2=depth2>0;

 

imaux1=double(repmat(m1,[1,1,3])).*double(im1)/255;

imaux2=double(repmat(m2,[1,1,3])).*double(im2)/255;

 

imagesc([imaux1 imaux2]);

% ind1=ginput(5);ind2=ginput(5);

ind1=sub2ind([480 640],uint64(v1),uint64(u1));

ind2=sub2ind([480 640],uint64(v2),uint64(u2));

%%

pc1=xyz1(ind1,:)'-repmat(cent1,1,5);

pc2=xyz2(ind2,:)'-repmat(cent2,1,5);

[a b c]=svd(pc2*pc1')

R12=a*c'

%%

xyzt1=R12*(xyz1'-repmat(cent1,1,length(xyz1)));

xyzt2=xyz2'-repmat(cent2,1,length(xyz2));

T=cent2-R12*cent1;

%%

%p11=pointCloud(xyzt1','Color',cl1);
p11=showPointCloud(xyzt1',cl1);

%p22=pointCloud(xyzt2','Color',cl2);
p22=showPointCloud(xyzt2',cl2);