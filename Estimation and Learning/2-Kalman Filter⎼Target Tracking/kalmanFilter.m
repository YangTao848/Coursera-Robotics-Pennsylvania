function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
dt=0.0330;
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
       
        predictx = x;
        predicty = y;
        return;
    end

    state=state';
    A=[1 0 dt 0;0 1 0 dt;0 0 1 0;0 0 0 1];
   z=[x;y];
    C=[1 0 0 0;0 1 0 0];
    
    sx=0.003;
    sy=sx;
    svx=0.01;
    svy=svx;
 Q = [dt*dt/4 0 dt/2 0 ;0 dt*dt/4 0 dt/2;dt/2 0 1 0;0 dt/2 0 1]*5.5;
%Q = [sx*sx 0 sx*svx 0 ;0 sy*sy 0 sy*svy ;svx*sx 0 svx*svx 0 ;0 svy*sy 00 svy*svy ]




R=[0.01,0;0,0.01];
    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    state=A*state;
    P=A*param.P*A'+Q;
    
    K=P*C'/((C*P*C')+R);
    state = state + K*(z - (C*state));
   % param.p = P - K * C * P;
    param.P=((eye(4)-(K*C))*P);
   state=state';
    x = state(1), y=state(2), vx = state(3), vy=state(4)

predictx = x + vx*0.330;

predicty = y + vy*0.330;
end
