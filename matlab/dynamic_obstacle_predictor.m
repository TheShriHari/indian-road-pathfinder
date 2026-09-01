function predicted_trajectories = dynamic_obstacle_predictor(observations, dt, N_horizon)
% DYNAMIC_OBSTACLE_PREDICTOR Predicts short-term future paths for unstructured agents
%   predicted_trajectories = dynamic_obstacle_predictor(observations, dt, N_horizon)
%
% Inputs:
%   observations: Struct array of detected obstacles with fields:
%                 .id, .type ('cattle', 'auto_rickshaw', 'pedestrian', 'pushcart')
%                 .position [x, y], .velocity [vx, vy]
%   dt          : Time step size (seconds)
%   N_horizon   : Number of prediction steps into the future
%
% Outputs:
%   predicted_trajectories: Struct array with predicted [X, Y] arrays over horizon.

if nargin < 2, dt = 0.1; end
if nargin < 3, N_horizon = 20; end % 2 second horizon

num_obs = length(observations);
predicted_trajectories = struct('id', {}, 'type', {}, 'waypoints', {}, 'covariance', {});

for i = 1:num_obs
    obs = observations(i);
    pred_path = zeros(N_horizon, 2);
    
    curr_pos = obs.position;
    curr_vel = obs.velocity;
    
    % Model dynamic behavior based on agent classification
    switch lower(obs.type)
        case 'cattle'
            % Cattle have low speed, high directional variance (random lateral movement)
            noise_factor = 0.15;
            for t = 1:N_horizon
                % Add slight random lateral wander typical for cows on village roads
                wander = [randn()*noise_factor, randn()*noise_factor];
                curr_pos = curr_pos + curr_vel * dt + wander;
                pred_path(t, :) = curr_pos;
            end
            
        case 'auto_rickshaw'
            % Auto-rickshaws exhibit aggressive weaving / non-lane behavior
            heading = atan2(curr_vel(2), curr_vel(1));
            speed = norm(curr_vel);
            for t = 1:N_horizon
                curr_pos = curr_pos + [speed * cos(heading), speed * sin(heading)] * dt;
                pred_path(t, :) = curr_pos;
            end
            
        case 'pedestrian'
            % Constant velocity model with higher uncertainty expansion
            for t = 1:N_horizon
                curr_pos = curr_pos + curr_vel * dt;
                pred_path(t, :) = curr_pos;
            end
            
        otherwise % Pushcarts, static obstacles, unknown vehicles
            for t = 1:N_horizon
                curr_pos = curr_pos + curr_vel * dt;
                pred_path(t, :) = curr_pos;
            end
    end
    
    predicted_trajectories(i).id = obs.id;
    predicted_trajectories(i).type = obs.type;
    predicted_trajectories(i).waypoints = pred_path;
end
end
