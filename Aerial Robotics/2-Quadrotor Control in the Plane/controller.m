function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
kvz=10;
kpz=20;
kvo=30;
kpo=300;
kvy=10;
kpy=20;

p=state.pos;
d=state.vel;
u1 = params.mass*(params.gravity+des_state.acc(2)+kvz*(des_state.vel(2)-d(2))+kpz*(des_state.pos(2)-p(2)));
Oc=-1/params.gravity*(des_state.acc(1)+kvy*(des_state.vel(1)-d(1))+kpy*(des_state.pos(1)-p(1)));
u2 =params.Ixx*(kvo*(-state.omega)+kpo*(Oc-state.rot));

% FILL IN YOUR CODE HERE

end

