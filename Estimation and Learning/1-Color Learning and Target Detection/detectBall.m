% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
thre=200;
mu=0;
sig=[];
load('params.mat');
im=im2double(I);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
z=zeros(size(im,1),size(im,2));
for i=1:size(im,1)
    for j=1:size(im,2)
        p=[im(i,j,1) im(i,j,2) im(i,j,3)];
        v=mvnpdf(p,mu,sig);
        if(v>thre)
         z(i,j)=1;
        end
    end
end

imshow(z);

bw_biggest = false(size(z));

% http://www.mathworks.com/help/images/ref/bwconncomp.html

CC = bwconncomp(z);

numPixels = cellfun(@numel,CC.PixelIdxList);

[biggest,idx] = max(numPixels);

bw_biggest(CC.PixelIdxList{idx}) = true;

figure,

imshow(bw_biggest); hold on;

% show the centroid

% http://www.mathworks.com/help/images/ref/regionprops.html

S = regionprops(CC,'Centroid');

loc = S(idx).Centroid;

plot(loc(1), loc(2),'r+');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI = z;
% loc = 


end
