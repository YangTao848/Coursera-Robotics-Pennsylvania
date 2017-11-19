% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Map Parameters 
% 
% % the number of grids for 1 meter.
% myResolution = param.resol;
% % the origin of the map in pixels
% myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
 
%    imagesc(map); 
% colormap('gray');
% axis equal;
% hold on;
% plot(pose(1,:)*param.resol+param.origin(1), ...
%     pose(2,:)*param.resol+param.origin(2), 'r.-');
% hold on;
% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 400;                            % Please decide a reasonable number of M, 
                             % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
W=zeros(M,1);
V=0.003;
   
   map=map-0.5;

dTheta=0.15;
dR=0.15;
for j = 2:N % start estimating from j=2 

    % 1) Propagate the particles 
    best=1;
    bestW=-10000000000000000;
    
    corr=zeros(M,1);
    for i = 1:M
     P(3,i) = P(3,i) + dTheta*randn;
     r = dR*(randn); 
     P(1,i) = P(1,i) + r*cos(P(3,i)); 
     P(2,i) = P(2,i) - r*sin(P(3,i));
     
     px=P(1,i)+cos(scanAngles+P(3,i)).*ranges(:,j);
     py=P(2,i)-sin(scanAngles+P(3,i)).*ranges(:,j);  
     
     ix=ceil(px*param.resol)+param.origin(1);
     iy=ceil(py*param.resol)+param.origin(2);

%    imagesc(map); 
% colormap('gray');
% axis equal;
% hold on;
% plot(pose(1,:)*param.resol+param.origin(1), ...
%     pose(2,:)*param.resol+param.origin(2), 'r.-');
% hold on;
%  plot(px',py', 'g.');
%    plot(ix',iy', 'y.');  

     s=0;
     
     for k=1:size(scanAngles,1)
        if(iy(k)>0 && iy(k)<=size(map,1) && ix(k)>0 && ix(k)<=size(map,2))
            s=s+map(iy(k),ix(k));
        end
        
     end
      corr(i)=s;
        if(s>bestW)
            bestW=s;
             best=i;
        end

    end
   
    corr(corr<200)=0;
    corr=corr/sum(corr);
   
    
    
      x=P(1,best)+cos(scanAngles+P(3,best)).*ranges(:,j);
     y=P(2,best)-sin(scanAngles+P(3,best)).*ranges(:,j);  
   
    ix=ceil(x*param.resol)+param.origin(1);
     iy=ceil(y*param.resol)+param.origin(2);
    
    px=P(1,:)*param.resol+param.origin(1);
    py=P(2,:)*param.resol+param.origin(2);

    
    
%     
% imagesc(map); 
% colormap('gray');
% axis equal;
% hold on;
% plot(pose(1,:)*param.resol+param.origin(1), ...
%     pose(2,:)*param.resol+param.origin(2), 'r.-');
% hold on;
%  plot(px',py', 'g.');
%    plot(ix',iy', 'y.');
   
   
    P = repmat(P(:,best), [1, M]);  
    myPose(1,j)=P(1,:)*corr;
    myPose(2,j)=P(2,:)*corr;
     myPose(3,j)=P(3,:)*corr;
%    plot(P(1,best)*param.resol+param.origin(1),P(2,best)*param.resol+param.origin(2), 'b.');
   



end

