function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
n=size(video_pts,1);
a=[];

for i=1:n
    xv=video_pts(i,:);
    xl=logo_pts(i,:);
    a((i-1)*2+1,:)=[-xv(1) -xv(2) -1 0 0 0 xv(1)*xl(1) xv(2)*xl(1) xl(1)];
    a(i*2,:)=[0 0 0 -xv(1) -xv(2) -1 xv(1)*xl(2) xv(2)*xl(2) xl(2)];
end

[U, S, V] = svd(a);
h=V(:,size(V,2));

H= [h(1) h(2) h(3);h(4) h(5) h(6); h(7) h(8) h(9)];
