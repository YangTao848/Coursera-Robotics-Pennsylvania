function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2
best=-1;
y1=x1;
y2=x2;
y1(:,3)=ones();
y2(:,3)=ones();

A=[];
for i=1:size(y1,1)
    A(i,:)=reshape(y1(i,:)'*y2(i,:),1,9);
end

[U S V]=svd(A);
F_temp=V(:,size(V,2));
F_temp=F_temp/norm(F_temp);
F_temp=reshape(F_temp,3,3);

[U S V]=svd(F_temp);
S(size(S))=zeros();
F_temp=U*S*V';

y1=x1;
y2=x2;
y1(:,3)=ones();
y2(:,3)=ones();

s=0;
for j=1:size(x1,1)
    s=s+abs([x1(j,:) 1]*F_temp*[x2(j,:) 1]');
end
if(s<best || best==-1)
    best=s;
    F=F_temp;
end

%F
%best


