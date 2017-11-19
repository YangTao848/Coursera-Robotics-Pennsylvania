function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
K=[100; 10];

u = params.mass*(sum(K.*(s_des-s))+params.gravity);


% FILL IN YOUR CODE HERE


end

