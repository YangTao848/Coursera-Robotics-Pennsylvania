function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the rostatebot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thurst
kp=150;
kd=2;

t=des_state.vel;
if norm(t)>0
    t=t/norm(t);
end

n=des_state.acc;
if norm(n)>0
    n=n/norm(n);
end

b=cross(t,n);
if norm(b)>0
    b=b/norm(b);
end


ep=((des_state.pos-state.pos)'*n)*n+((des_state.pos-state.pos)'*b)*b;
ev=des_state.vel-state.vel;

r=des_state.acc+kd*ev+kp*ep;

r=kd*(des_state.vel-state.vel)+kp*(des_state.pos-state.pos);

F = params.mass*(params.gravity+r(3));


% Moment
psi=des_state.yaw;
phi=1/params.gravity*(r(1)*sin(psi)-r(2)*cos(psi));
theta=1/params.gravity*(r(1)*cos(psi)+r(2)*sin(psi));


p=0;
q=0;
r=des_state.yawdot;

kpo=150;
kdo=0.3;

kpt=150;
kdt=0.3;

kpp=150;
kdp=0.3;

M = [kpo*(phi-state.rot(1))+kdo*(p-state.omega(1));
    kpt*(theta-state.rot(2))+kdt*(q-state.omega(2));
    kpp*(psi-state.rot(3))+kdp*(r-state.omega(3));];

end
