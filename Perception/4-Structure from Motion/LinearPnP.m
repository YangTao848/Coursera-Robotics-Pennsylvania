function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation

C=zeros(3,1);
R=eye(3);
A=[];
for i=1:size(X,1)
   	 A((i-1)*2+1,:)=[X(i,:) 1 0 0 0 0 -x(i,1)*X(i,:) -x(i,1)];
     A(i*2,:)=[0 0 0 0 X(i,:) 1 -x(i,2)*X(i,:) -x(i,2)];
end

[U S V]=svd(A);
P=reshape(V(:,end),4,3)';
P=[inv(K)*P];
R=P(:,1:3);
t=P(:,4);

[U S V]=svd(R);

if(det(U*V')>0)
    R=U*V';
    t=t/S(1,1);
else
     R=-U*V';
     t=-t/S(1,1);
end

C=-R'*t;

