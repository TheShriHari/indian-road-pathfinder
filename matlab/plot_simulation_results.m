% PLOT_SIMULATION_RESULTS Visualizes exported simulation telemetry CSV data
%   Reads 'simulation_telemetry.csv' exported from the Pathfinder Studio
%   and renders multi-variable dynamics subplots (v(t), v(x), a(t), Safety).

clear; clc; close all;

csv_filename = 'simulation_telemetry.csv';

if ~exist(csv_filename, 'file')
    % Check parent or current directory
    if exist('../simulation_telemetry.csv', 'file')
        csv_filename = '../simulation_telemetry.csv';
    else
        error('Telemetry CSV file not found! Please run the web simulation and click "Export Telemetry CSV".');
    end
end

data = readtable(csv_filename);

fprintf('========================================================\n');
fprintf(' PATHFINDER STUDIO - TELEMETRY ANALYTICS REPORT        \n');
fprintf('========================================================\n');
fprintf(' Total Duration    : %.2f seconds\n', data.time(end));
fprintf(' Total Distance    : %.2f meters\n', data.x_pos(end));
fprintf(' Max Velocity      : %.2f m/s (%.1f km/h)\n', max(data.velocity), max(data.velocity)*3.6);
fprintf(' Avg Velocity      : %.2f m/s (%.1f km/h)\n', mean(data.velocity), mean(data.velocity)*3.6);
fprintf(' Peak Accel        : %.2f m/s^2\n', max(data.acceleration));
fprintf(' Max Braking       : %.2f m/s^2\n', min(data.acceleration));
fprintf(' Min Safety Score  : %.1f%%\n', min(data.safety_index));
fprintf('========================================================\n\n');

% Prepare Dark Mode Visualization Figure
fig = figure('Name', 'Pathfinder Telemetry Analytics', 'Color', [0.1 0.1 0.12], 'Position', [100 80 950 750]);

% 1. Velocity vs Time v(t)
subplot(3,1,1);
plot(data.time, data.velocity * 3.6, 'g-', 'LineWidth', 2.0);
grid on; hold on;
set(gca, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.35]);
ylabel('Velocity (km/h)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
title('Vehicle Longitudinal Velocity vs. Time v(t)', 'Color', [0.2 0.8 1], 'FontSize', 11, 'FontWeight', 'bold');

% 2. Velocity vs Longitudinal Position v(x) (Spatial Deceleration Zones)
subplot(3,1,2);
plot(data.x_pos, data.velocity * 3.6, 'c-', 'LineWidth', 2.0);
grid on; hold on;
set(gca, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.35]);
ylabel('Velocity (km/h)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
title('Velocity vs. Position v(x) (Spatial Deceleration Zones)', 'Color', [0.2 0.8 1], 'FontSize', 11, 'FontWeight', 'bold');

% 3. Acceleration & Braking Dynamics Profile a(t)
subplot(3,1,3);
plot(data.time, data.acceleration, 'r-', 'LineWidth', 1.8);
grid on; hold on;
% Draw braking threshold line at -3.5 m/s^2
yline(-3.5, 'w--', 'Hard Braking Limit (-3.5 m/s^2)', 'Color', [1 0.3 0.3], 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
set(gca, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.35]);
ylabel('Accel (m/s^2)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
title('Acceleration & Braking Dynamics Profile a(t)', 'Color', [0.2 0.8 1], 'FontSize', 11, 'FontWeight', 'bold');
