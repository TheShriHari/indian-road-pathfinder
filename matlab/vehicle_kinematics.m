function stateDot = vehicle_kinematics(state, control, params)
% VEHICLE_KINEMATICS Kinematic Bicycle Model for Autonomous Vehicle
%   stateDot = vehicle_kinematics(state, control, params)
%
% Inputs:
%   state   : [x; y; theta; v] (x-pos, y-pos, heading in rad, speed in m/s)
%   control : [delta; a]       (steering angle in rad, acceleration in m/s^2)
%   params  : Struct with vehicle parameters (L_wheelbase, max_steer, etc.)
%
% Output:
%   stateDot: [d_x; d_y; d_theta; d_v]

if nargin < 3 || isempty(params)
    params.L = 2.7;          % Wheelbase (meters)
    params.max_steer = pi/6;  % Max steer angle (30 deg)
    params.max_accel = 3.0;   % Max acceleration (m/s^2)
    params.min_accel = -5.0;  % Max deceleration (m/s^2)
end

% Extract state
x     = state(1);
y     = state(2);
theta = state(3);
v     = state(4);

% Extract control input and apply saturation
delta = min(max(control(1), -params.max_steer), params.max_steer);
a     = min(max(control(2), params.min_accel), params.max_accel);

% Kinematic Bicycle Equations
d_x     = v * cos(theta);
d_y     = v * sin(theta);
d_theta = (v / params.L) * tan(delta);
d_v     = a;

stateDot = [d_x; d_y; d_theta; d_v];
end
