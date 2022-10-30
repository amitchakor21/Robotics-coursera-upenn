function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], 
%   state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], 
%   state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], 
%   des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], 
%   des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
%F = 0;

mass = params.mass;
I = params.I;
invI = params.invI;
grav = params.gravity;         
L = params.arm_length;
minF = params.minF;
maxF = params.maxF;

%F = minF:0.01:maxF;

% Moment

Kp = [10; 10; 80];
Kd = [10; 10; 35];
Kprot = [200; 200; 200];
Kdrot = [.1;.1;.1];

Ep = des_state.pos -state.pos;
Ev = des_state.vel -state.vel;

ncap = des_state.vel/norm(des_state.vel); % Unit tangent to trajectory 
tcap = des_state.acc/norm(des_state.acc); % Unit normal to trajectory
bcap = cross(tcap, ncap);
%delta_pos = (des_state.pos-state.pos);
if(any(isnan(bcap)))
    %err_p = delta_pos;
else
    Ep = (Ep'*ncap)*ncap + (Ep'*bcap)*bcap;
end


rddot_des = des_state.acc +Kp.*Ep + Kd.*Ev; 

F = mass*(grav+rddot_des(3));

psi_des = des_state.yaw;
phi_des = (rddot_des(1)*sin(psi_des) - rddot_des(2)*cos(psi_des))/grav;
theta_des = (rddot_des(1)*cos(psi_des) + rddot_des(2)*sin(psi_des))/grav;

rot_des =  [phi_des ;theta_des;psi_des ];
omega_des = [0;0;des_state.yawdot];

M = Kprot.*(rot_des - state.rot) + Kdrot.*(omega_des - state.omega);



% =================== Your code ends here ===================

end
