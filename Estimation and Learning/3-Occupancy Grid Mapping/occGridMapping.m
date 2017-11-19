% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
res = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
origin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
n=size(scanAngles,1);
for j = 1:N % for each time,
    
    x=pose(1,j);
    y=pose(2,j);
    theta=pose(3,j);
    
    xocc=cos(scanAngles+theta).*ranges(:,j)+x;
    yocc=-sin(scanAngles+theta).*ranges(:,j)+y;
    
    ixocc=ceil(res*xocc)+origin(1);
    iyocc=ceil(res*yocc)+origin(2);
   
    ix=ceil(res*x)+origin(1);
    iy=ceil(res*y)+origin(2);
    
    for i=1:n
     
        [freex, freey] = bresenham(ix,iy,ixocc(i),iyocc(i));  

        free = sub2ind(size(myMap),freey,freex);

        myMap(iyocc(i),ixocc(i)) = myMap(iyocc(i),ixocc(i))+ lo_occ;

        myMap(free) = myMap(free)-lo_free;
    end
   
   


%     % Update the log-odds
%   
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
% end

end

 myMap=min(myMap,lo_max);
 myMap=max(myMap,lo_min);
    imagesc(myMap);