function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.





persistent waypoints0 traj_time d0 alpha
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2*sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    
    N=size(waypoints,2)-1;
    A=zeros(8*N,8*N);
    b=zeros(8*N,3);
    
    %segments start (n)
    for i=1:N
        A(i,(i-1)*8+1)=1;
    end
    b(1:N,:)=waypoints(:,1:N)';
    
    %segments end (2n)
    for i=1:N
        A(N+i,(i-1)*8+1:i*8)=1;
    end
    b(N+1:2*N,:)=waypoints(:,2:N+1)';
    
    n=2*N+1;
    %start at rest (2n+3)
    A(n,2)=1/d0(1);
    A(n+1,3)=2/d0(1)^2;
    A(n+2,4)=6/d0(1)^3;
    
    %end at rest (2n+6)

    A(n+3,8*(N-1)+1:8*N)=[0,1/d0(N),2/d0(N),3/d0(N),4/d0(N),5/d0(N),6/d0(N),7/d0(N)];
    A(n+4,8*(N-1)+1:8*N)=[0,0,2/d0(N)^2,6/d0(N)^2,12/d0(N)^2,20/d0(N)^2,30/d0(N)^2,42/d0(N)^2];
    A(n+5,8*(N-1)+1:8*N)=[0,0,0,6/d0(N)^3,24/d0(N)^3,60/d0(N)^3,120/d0(N)^3,210/d0(N)^3];
   
    
    n=n+6;
    %derivatives must be equal at junction points
    for i=[2:N]
     A(n,(i-2)*8+1:(i)*8)=[0,1/d0(i-1),2/d0(i-1),3/d0(i-1),4/d0(i-1),5/d0(i-1),6/d0(i-1),7/d0(i-1),0,-1/d0(i),0,0,0,0,0,0];
     A(n+1,(i-2)*8+1:(i)*8)=[0,0,2/d0(i-1)^2,6/d0(i-1)^2,12/d0(i-1)^2,20/d0(i-1)^2,30/d0(i-1)^2,42/d0(i-1)^2,0,0,-2/d0(i)^2,0,0,0,0,0];
     A(n+2,(i-2)*8+1:(i)*8)=[0,0,0,6/d0(i-1)^3,24/d0(i-1)^3,60/d0(i-1)^3,120/d0(i-1)^3,210/d0(i-1)^3,0,0,0,-6/d0(i)^3,0,0,0,0];  
     A(n+3,(i-2)*8+1:(i)*8)=[0,0,0,0,24/d0(i-1)^4,120/d0(i-1)^4,360/d0(i-1)^4,840/d0(i-1)^4,0,0,0,0,-24/d0(i)^4,0,0,0];   
     A(n+4,(i-2)*8+1:(i)*8)=[0,0,0,0,0,120/d0(i-1)^5,720/d0(i-1)^5,2520/d0(i-1)^5,0,0,0,0,0,-120/d0(i)^5,0,0];   
     A(n+5,(i-2)*8+1:(i)*8)=[0,0,0,0,0,0,720/d0(i-1)^6,5040/d0(i-1)^6,0,0,0,0,0,0,-720/d0(i)^6,0];  
    % A(n+6,(i-2)*8+1:(i)*8)=[0,0,0,0,0,0,0,5040/d0(i-1)^7,0,0,0,0,0,0,0,-5040/d0(i)^7];  
     n=n+6;
    end
    alpha=linsolve(A,b);
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
      desired_state.vel = zeros(3,1);
      desired_state.acc = zeros(3,1);
      desired_state.yaw = 0;
      desired_state.yawdot = 0;
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
  
        else
        scale = (t)/d0(t_index-1);
        desired_state.pos = (alpha((t_index-2)*8+1,:)+alpha((t_index-2)*8+2,:)*scale+alpha((t_index-2)*8+3,:)*scale^2+alpha((t_index-2)*8+4,:)*scale^3+alpha((t_index-2)*8+5,:)*scale^4+alpha((t_index-2)*8+6,:)*scale^5+alpha((t_index-2)*8+7,:)*scale^6+alpha((t_index-2)*8+8,:)*scale^7)';
        desired_state.vel = (alpha((t_index-2)*8+2,:)/d0(t_index-1)+2*alpha((t_index-2)*8+3,:)/d0(t_index-1)*scale+3*alpha((t_index-2)*8+4,:)/d0(t_index-1)*scale^2+4*alpha((t_index-2)*8+5,:)/d0(t_index-1)*scale^3+5*alpha((t_index-2)*8+6,:)/d0(t_index-1)*scale^4+6*alpha((t_index-2)*8+7,:)/d0(t_index-1)*scale^5+7*alpha((t_index-2)*8+8,:)/d0(t_index-1)*scale^6)';
        desired_state.acc =(2*alpha((t_index-2)*8+3,:)/(d0(t_index-1)^2)+6*alpha((t_index-2)*8+4,:)/(d0(t_index-1)^2)*scale+12*alpha((t_index-2)*8+5,:)/(d0(t_index-1)^2)*scale^2+20*alpha((t_index-2)*8+6,:)/(d0(t_index-1)^2)*scale^3+30*alpha((t_index-2)*8+7,:)/(d0(t_index-1)^2)*scale^4+42*alpha((t_index-2)*8+8,:)/(d0(t_index-1)^2)*scale^5)';
      
         
    end
   
end
%

%% Fill in your code here
% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

