function bookindex = books( test_image_names, training_image_names )

run ('vlfeat-0.9.20\toolbox\vl_setup');
load calib_asus.mat
a=0;
dummy_index=0;

for i=1:length(training_image_names),
    training_data(i).depths = load( training_image_names(i).depth);
    training_data(i).images = imread(training_image_names(i).rgb);
    %figure();imshow(training_data(i).images);
end

for i=1:length(test_image_names),
    test_data(i).depths = load( test_image_names(i).depth);
    test_data(i).images = imread(test_image_names(i).rgb);
    %figure();imshow(test_data(i).images);
end
% figure();imshow(test_data(1).images);
% figure();imshow(training_data(2).images);

for j=1:length(test_image_names)
    l=7;
    im1g = rgb2gray(training_data(l).images);
    im2g = rgb2gray(test_data(j).images );
%     xyz1 = get_xyzasus(training_data(l).depths.depth_array(:), [480 640], 1:640*480, Depth_cam.K, 1, 0);
%     xyz2 = get_xyzasus(test_data(j).depths.depth_array(:), [480 640], 1:640*480, Depth_cam.K, 1, 0);

    %KeyPoints Detection
    PeakThresh= 8;%REVER
    edge_thresh = 9;%REVER
    ws = 2;
    [F, D] = vl_sift(single(im1g), 'edgethresh', edge_thresh, 'PeakThresh',PeakThresh ,'WindowSize', ws);
    [F2, D2] = vl_sift(single(im2g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);
    %Find Matches 1 2 
    th = 5;%REVER
    [MATCHES, ~] = vl_ubcmatch(D, D2, th);
    %display(MATCHES);
    
%     [F, D] = vl_sift(single(im1g)) ;
%     [F2, D2] = vl_sift(single(im2g)) ;
%     [MATCHES, ~] = vl_ubcmatch(D, D2,th) ;

    if MATCHES > 0
        a = a + 1;
        [~, n] = size(MATCHES);
        
        for i=1:n
        u1(i) = F(1,MATCHES(1,i));
        v1(i) = F(2,MATCHES(1,i));
        u2(i) = F2(1,MATCHES(2,i));
        v2(i) = F2(2,MATCHES(2,i));
        end
        
        figure;imagesc(im1g);hold on;plot(u1,v1,'*r');hold off;
        figure;imagesc(im2g);hold on;plot(u2,v2,'*r');hold off;
        
%         Print Matches antes do ransac 1 2
%         ind1 = sub2ind([480 640],uint64(v1'),uint64(u1'));
%         ind2 = sub2ind([480 640],uint64(v2'),uint64(u2'));
%         [~, n] = size (v1);

%         [u1f,v1f,u2f,v2f] = RANSAC(u1,v1,u2,v2,xyz1,xyz2,ind1,ind2);
%         [~, n] = size (u1f);
%         figure;imagesc(im1g);hold on;plot(u1f,v1f,'*r');hold off;
%         figure;imagesc(im2g);hold on;plot(u2f,v2f,'*r');hold off;
%         Print Matches depois do ransac 1 2
%         ind1 = sub2ind([480 640],uint64(v1f'),uint64(u1f'));
%         ind2 = sub2ind([480 640],uint64(v2f'),uint64(u2f'));
     
        dummy_index(a) = j;
        
    elseif j == length(test_image_names) && isempty(dummy_index) == 1
        bookindex = 0;
    end
end

bookindex = dummy_index;

end