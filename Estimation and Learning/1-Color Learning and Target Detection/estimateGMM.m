% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = './train';
clear
load('samples.mat')
% visualize the sample distribution
figure, 
Samples=im2double(Samples);

scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%

D=3;
N=3;

mu = rand(N,D);
sig=zeros(D,D,N);
likelihood=zeros(size(Samples,1),N);

for i=1:N
    sig(:,:,i) = rand(D);
    sig(:,:,i) = sig(:,:,i)*sig(:,:,i)'/2;
    sig(:,:,i) = sig(:,:,i) +D*eye(D);   
end


diff=1;
while diff>0.0000000001
    
for i=1:N    
  likelihood(:,i) = mvnpdf(Samples,mu(i,:),sig(:,:,i));
end
z=likelihood./repmat(sum(likelihood,2),1,N);

old_mu=mu;
old_sig=sig;
for i=1:N
   zi=z(:,i);
   zk=repmat(zi,1,D);
   mu(i,:)=sum(zk.*Samples,1)/sum(zi);
   
   s=0;
   for j=1:size(Samples,1)
       m=mu(i,:);
       x=Samples(j,:);
       s=s+z(j,i).*(x-m)'*(x-m);
  
   end
   sig(:,:,i)=s/sum(zi);
 
end
mu_diff=(old_mu-mu).^2;
sig_diff=(old_sig-sig).^2;
diff=sum(sum(mu_diff,1),2)+sum(sum(sum(sig_diff,1),2),3);

end
save('params.mat','mu','sig');