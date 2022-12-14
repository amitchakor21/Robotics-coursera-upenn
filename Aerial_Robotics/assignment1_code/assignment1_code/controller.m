function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;


% FILL IN YOUR CODE HERE
Kp = 150;
Kv = 20;
z_des = 0.0;

u = params.mass*(z_des + Kp*(s_des(1)-s(1))+Kv*(s_des(2)-s(2)) + params.gravity);
end

