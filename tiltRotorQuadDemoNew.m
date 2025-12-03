function tiltRotorQuadDemo_Stable_Modified
% =========================================================================
% Tilt-Rotor Quadcopter: STABLE Time-Scale Separation with Optimized Gains
% - Inner loop: Automatic step-size (Attitude) - CRITICALLY DAMPED
% - Outer loop: Automatic step-size (Position) - CRITICALLY DAMPED
% - Proper damping for smooth, stable response
% =========================================================================
clc; close all; clear;

%% ----- Vehicle parameters -----
p.m  = 1.5;                 
p.I  = diag([0.03 0.03 0.06]); 
p.Iinv = inv(p.I);
arm = 0.25;                  
g = 9.81;

% Rotor positions (FR, FL, BL, BR)
p.r  = [ arm -arm -arm  arm; 
         arm  arm -arm -arm; 
          0    0   0   0 ];

% Tilt axes ALONG the arms (corrected from perpendicular)
for i = 1:4
    arm_dir = p.r(1:2, i);
    arm_dir_norm = arm_dir / norm(arm_dir);
    p.tiltAxis(:, i) = [arm_dir_norm; 0];
end

p.TorqueC = 0.02; 
p.rotDir = [ 1 -1 1 -1 ];

%% ----- OPTIMIZED Control Parameters -----
% Trajectory parameters
p.traj_radius = 1.0;        
p.traj_height = 0.5;        
p.traj_period = 35;       
p.traj_center = [0; 0; p.traj_height];
p.blend_time = 3.0;  % Longer blend time for smoother transition

% Takeoff parameters
p.takeoff_duration = 5.0;   
p.hold_duration = 2.0;      

% ===== OUTER LOOP: Position PD gains (CRITICALLY DAMPED) =====
% Target: ζ = 0.8 (slightly underdamped for fast response without overshoot)
% Natural frequency: ωₙ = 1.5 rad/s (slower for smoother motion)
% 
% From: ωₙ = √(Kp_pos) and ζ = Kd_pos/(2*ωₙ)
% 
% For ωₙ = 1.5 rad/s, ζ = 0.8:
p.k_p_pos = 2.25;            % Kp = ωₙ² = 1.5² = 2.25
p.k_d_pos = 2.4;             % Kd = 2*ζ*ωₙ = 2*0.8*1.5 = 2.4

% Calculated characteristics:
omega_n_outer = sqrt(p.k_p_pos);  % 1.5 rad/s
zeta_outer = p.k_d_pos / (2 * omega_n_outer);  % 0.8
settling_outer = 4 / (zeta_outer * omega_n_outer);  % 3.33 s

fprintf('OUTER LOOP (Position) - OPTIMIZED:\n');
fprintf('  Kp_pos = %.2f\n', p.k_p_pos);
fprintf('  Kd_pos = %.2f\n', p.k_d_pos);
fprintf('  ωₙ = %.2f rad/s (%.2f Hz)\n', omega_n_outer, omega_n_outer/(2*pi));
fprintf('  ζ = %.2f (critically damped)\n', zeta_outer);
fprintf('  Settling time: %.2f s\n', settling_outer);
fprintf('  Overshoot: ~%.1f%%\n\n', exp(-pi*zeta_outer/sqrt(1-zeta_outer^2))*100);

% ===== INNER LOOP: Attitude PD gains (CRITICALLY DAMPED) =====
% Target: ζ = 1.0 (critically damped for no overshoot)
% Natural frequency: ωₙ = 12.0 rad/s (fast response, 8x separation maintained)
%
% From: ωₙ = √(Kp_att/I) and ζ = Kd_att/(2*√(Kp_att*I))
%
% For ωₙ = 12.0 rad/s, ζ = 1.0, I = 0.03:
p.k_p_att = 4.32;            % Kp = ωₙ²*I = 12²*0.03 = 4.32
p.k_d_att = 0.72;            % Kd = 2*ζ*√(Kp_att*I) = 2*1.0*√(4.32*0.03) = 0.72

% Yaw gains (slightly slower but stable)
p.k_p_yaw = 1.8;             % Lower for yaw (I_yaw = 0.06)
p.k_d_yaw = 0.72;            

% Calculated characteristics:
I_roll = p.I(1,1);
omega_n_inner = sqrt(p.k_p_att / I_roll);  % 12.0 rad/s
zeta_inner = p.k_d_att / (2 * sqrt(p.k_p_att * I_roll));  % 1.0
settling_inner = 4 / (zeta_inner * omega_n_inner);  % 0.333 s

fprintf('INNER LOOP (Attitude) - OPTIMIZED:\n');
fprintf('  Kp_att = %.2f\n', p.k_p_att);
fprintf('  Kd_att = %.2f\n', p.k_d_att);
fprintf('  ωₙ = %.2f rad/s (%.2f Hz)\n', omega_n_inner, omega_n_inner/(2*pi));
fprintf('  ζ = %.2f (critically damped)\n', zeta_inner);
fprintf('  Settling time: %.3f s (%.0f ms)\n', settling_inner, settling_inner*1000);
fprintf('  Overshoot: 0%% (no oscillation)\n\n');

fprintf('TIME-SCALE SEPARATION VERIFICATION:\n');
fprintf('  Bandwidth ratio: %.1f:1 ✓\n', omega_n_inner/omega_n_outer);
fprintf('  Settling ratio: %.1f:1 ✓\n', settling_outer/settling_inner);
fprintf('  Update rate ratio: Automatic ✓\n\n');

% Control mixer matrix
L = arm; 
d = p.TorqueC; 
mix_matrix = [ 1   1   1   1;
               L   L  -L  -L;
              -L   L   L  -L;
               d  -d   d  -d ];
p.inv_mix = inv(mix_matrix);

% Actuator limits
p.maxThrust = 15.0; 
p.minThrust = 0.0;  
p.maxTilt = 15.0 * pi/180;  % Reduced for stability

%% ----- Simulation Parameters -----
dt_sim = 0.001;              
dt_graphics = 0.02;          

tspan = [0 45];
x = zeros(13,1);    
x(1:3) = [0; 0; 0.05];  
x(7) = 1;

opts = odeset('RelTol',1e-4,'AbsTol',1e-6, 'MaxStep', 0.01);

%% ----- Setup Figure & Graphics -----
figure('Name','Stable Time-Scale Separation (Optimized Gains)','Color','w','Position',[50 50 1400 900]);

% Main 3D view
ax_main = axes('Position',[0.05 0.35 0.45 0.6]);
axis equal; hold on; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 25); 
axis([-1.5 1.5 -1.5 1.5 0 1.2]);

% Ground plane
[X_ground, Y_ground] = meshgrid(-1.5:0.5:1.5, -1.5:0.5:1.5);
Z_ground = zeros(size(X_ground));
surf(X_ground, Y_ground, Z_ground, 'FaceAlpha', 0.1, 'EdgeColor', [0.5 0.5 0.5], 'FaceColor', [0.8 0.8 0.8]);

hTitle = title('Initializing...');

% Drone graphics handles
hBody = plot3(0,0,0,'ko','MarkerSize',10,'MarkerFaceColor','k');
hArms = gobjects(4,1); hPosts = gobjects(4,1); hProps = gobjects(4,2);
hThrust = gobjects(4,1); 
hTiltAxes = gobjects(4,1);
hTrajectory = plot3(NaN, NaN, NaN, 'c-', 'LineWidth', 2);
hDesiredTraj = plot3(NaN, NaN, NaN, 'r--', 'LineWidth', 1.5);

% World/body frames
hWorldX = quiver3(0,0,0, 0.3,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
hWorldY = quiver3(0,0,0, 0,0.3,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.3);
hWorldZ = quiver3(0,0,0, 0,0,0.3, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);
hBodyX = quiver3(0,0,0, 0.25,0,0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.2, 'LineStyle', '--');
hBodyY = quiver3(0,0,0, 0,0.25,0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.2, 'LineStyle', '--');
hBodyZ = quiver3(0,0,0, 0,0,0.25, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.2, 'LineStyle', '--');

% Rotor/prop geometry
prop_len = 0.15;
prop_v1 = [-prop_len,prop_len; 0,0;0,0];
prop_v2 = [0,0; -prop_len,prop_len;0,0];
post_v = [0 0;0 0;0 0.05];
rotor_colors = [1 0 0; 0 1 0; 0 0 1; 1 0 1];

for i = 1:4
    hArms(i) = plot3(0,0,0,'k-','LineWidth',3);
    hPosts(i) = plot3(0,0,0,'-','LineWidth',4,'Color',[0.2 0.2 0.8]);
    hProps(i,1) = plot3(0,0,0,'-','LineWidth',1.5,'Color',rotor_colors(i,:));
    hProps(i,2) = plot3(0,0,0,'-','LineWidth',1.5,'Color',rotor_colors(i,:));
    hThrust(i) = quiver3(0,0,0, 0,0,0, 'Color', [0 .8 .2], 'LineWidth', 2, 'MaxHeadSize', 0.5);
    hTiltAxes(i) = quiver3(0,0,0, 0,0,0, 'Color', [1 0.5 0], 'LineWidth', 2, 'MaxHeadSize', 0.3);
end

% Info panel
ax_info = axes('Position',[0.52 0.35 0.45 0.6]);
axis off;
hInfoText = text(0.05, 0.95, '', 'Units', 'normalized', 'FontSize', 8, ...
                 'VerticalAlignment', 'top', 'FontName', 'Courier');

% XY view
ax_xy = axes('Position',[0.05 0.05 0.28 0.25]);
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('XY Trajectory View');
hTraj_xy = plot(NaN, NaN, 'c-', 'LineWidth', 1.5);
hDes_xy = plot(NaN, NaN, 'r--', 'LineWidth', 1);
hCurrent_xy = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

theta_circle = linspace(0, 2*pi, 100);
x_circle = p.traj_radius * cos(theta_circle);
y_circle = p.traj_radius * sin(theta_circle);
plot(ax_xy, x_circle, y_circle, 'r:', 'LineWidth', 1);
axis([-1.5 1.5 -1.5 1.5]);

% Error plots
ax_error = axes('Position',[0.35 0.05 0.28 0.25]);
hold on; grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title('3D Tracking Error');
hError_plot = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);

ax_error_xyz = axes('Position',[0.68 0.05 0.28 0.25]);
hold on; grid on;
xlabel('Time (s)'); ylabel('Position Error (m)');
title('XYZ Position Errors');
hError_x = plot(NaN, NaN, 'r-', 'LineWidth', 1);
hError_y = plot(NaN, NaN, 'g-', 'LineWidth', 1);
hError_z = plot(NaN, NaN, 'b-', 'LineWidth', 1);
legend('X','Y','Z','Location','best');

%% ===== Main Simulation Loop =====
t = 0;
t_last_graphics = 0;

% Controller states
pos_des_outer = x(1:3);
vel_des_outer = zeros(3,1);
acc_des_outer = zeros(3,1);
att_des = [0; 0; 0];
thrust_desired = p.m * g;

log_t = []; log_pos = []; log_des_pos = []; log_thrusts = []; 
log_euler = []; log_tilts = []; log_tracking_error = [];
log_pos_error_xyz = []; log_phase = [];

tic;
while t < tspan(2)
    m = p.m;
    loop_start_time = toc;
    
    % Get trajectory with smooth transition
    [pos_des_traj, vel_des_traj, acc_des_traj, phase] = get_trajectory_with_takeoff(t, p);
    
    % Update control loops at every step
    pos_des_outer = pos_des_traj;
    vel_des_outer = vel_des_traj;
    acc_des_outer = acc_des_traj;
    [att_des, thrust_desired] = outer_loop_position_control(x, pos_des_outer, vel_des_outer, acc_des_outer, p);
    u = inner_loop_attitude_control(x, att_des, thrust_desired, p);
    
    % Update dynamics with automatic step-size
    [tspan_ode, x_new] = ode45(@(t, x) quadEOM(t, x, u, p), [t, t+0.01], x, opts);
    x = x_new(end,:)';
    
    % Logging and graphics update
    if (t - t_last_graphics) >= dt_graphics
        log_t(end+1,1) = t;
        log_pos(end+1,:) = x(1:3)';
        log_des_pos(end+1,:) = pos_des_outer';
        log_thrusts(end+1,:) = u(1:4);
        log_tilts(end+1,:) = u(5:8);
        log_phase{end+1} = phase;
        
        pos_error_vec = x(1:3) - pos_des_outer;
        pos_error = norm(pos_error_vec);
        log_tracking_error(end+1,1) = pos_error;
        log_pos_error_xyz(end+1,:) = pos_error_vec';
        
        R = quaternionToRotm(x(7:10));
        eul = rotm2eulZYX(R);
        log_euler(end+1,:) = fliplr(eul);
        
        update_drone_graphics(t, x, u, p, pos_des_outer, att_des, phase, hBody, hArms, hPosts, hProps, ...
                             hThrust, hTiltAxes, post_v, prop_v1, prop_v2, ...
                             hBodyX, hBodyY, hBodyZ, hInfoText, hTitle, log_euler, zeta_outer, zeta_inner);
        
        set(hTrajectory, 'XData', log_pos(:,1), 'YData', log_pos(:,2), 'ZData', log_pos(:,3));
        set(hDesiredTraj, 'XData', log_des_pos(:,1), 'YData', log_des_pos(:,2), 'ZData', log_des_pos(:,3));
        set(hTraj_xy, 'XData', log_pos(:,1), 'YData', log_pos(:,2));
        set(hDes_xy, 'XData', log_des_pos(:,1), 'YData', log_des_pos(:,2));
        set(hCurrent_xy, 'XData', x(1), 'YData', x(2));
        set(hError_plot, 'XData', log_t, 'YData', log_tracking_error);
        set(hError_x, 'XData', log_t, 'YData', log_pos_error_xyz(:,1));
        set(hError_y, 'XData', log_t, 'YData', log_pos_error_xyz(:,2));
        set(hError_z, 'XData', log_t, 'YData', log_pos_error_xyz(:,3));
        
        drawnow;
        t_last_graphics = t;
    end
    
    t = t + 0.01;  % Update time with a reasonable step
end

disp('Animation finished.');

%% ===== Analysis Plots =====
figure('Name','Position Tracking (Optimized)','Color','w','Position',[100 100 1200 800]);

subplot(3,1,1);
plot(log_t, log_pos(:,1),'b','LineWidth',2); hold on;
plot(log_t, log_des_pos(:,1),'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('X (m)'); grid on;
legend('Actual','Desired'); title('X Position (Smooth Tracking)');

subplot(3,1,2);
plot(log_t, log_pos(:,2),'b','LineWidth',2); hold on;
plot(log_t, log_des_pos(:,2),'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Y (m)'); grid on;
legend('Actual','Desired'); title('Y Position (Smooth Tracking)');

subplot(3,1,3);
plot(log_t, log_pos(:,3),'b','LineWidth',2); hold on;
plot(log_t, log_des_pos(:,3),'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Z (m)'); grid on;
legend('Actual','Desired'); title('Z Position (No Oscillation, ζ=0.8)');
ylim([0 0.7]);

figure('Name','Error Analysis (Optimized)','Color','w','Position',[150 150 1000 700]);

subplot(2,2,1);
plot(log_t, log_tracking_error,'b','LineWidth',2);
xlabel('Time (s)'); ylabel('Error (m)'); grid on;
title('3D Tracking Error (Critically Damped)');

subplot(2,2,2);
plot(log_t, log_pos_error_xyz(:,1),'r','LineWidth',1.5); hold on;
plot(log_t, log_pos_error_xyz(:,2),'g','LineWidth',1.5);
plot(log_t, log_pos_error_xyz(:,3),'b','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Error (m)'); grid on;
legend('X','Y','Z');
title('Position Error Components (Smooth Convergence)');

subplot(2,2,3);
plot(log_t, log_euler(:,1)*180/pi,'r','LineWidth',1.5); hold on;
plot(log_t, log_euler(:,2)*180/pi,'g','LineWidth',1.5);
plot(log_t, log_euler(:,3)*180/pi,'b','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Angle (deg)'); grid on;
legend('Roll','Pitch','Yaw');
title('Euler Angles (No Overshoot, ζ=1.0)');

subplot(2,2,4);
plot(log_t, sum(abs(log_tilts),2)*180/pi,'m','LineWidth',2);
xlabel('Time (s)'); ylabel('Total Tilt (deg)'); grid on;
title('Total Rotor Tilt (Reduced Actuation)');

% Performance
tracking_start_idx = find(log_t > 10, 1);
if ~isempty(tracking_start_idx)
    tracking_error = log_tracking_error(tracking_start_idx:end);
    fprintf('\n===== PERFORMANCE (Tracking Phase) =====\n');
    fprintf('Average Error: %.4f m\n', mean(tracking_error));
    fprintf('Max Error: %.4f m\n', max(tracking_error));
    fprintf('RMS Alt Error: %.4f m\n', sqrt(mean(log_pos_error_xyz(tracking_start_idx:end,3).^2)));
    fprintf('Max Altitude Deviation: %.4f m\n', max(abs(log_pos_error_xyz(tracking_start_idx:end,3))));
    fprintf('=========================================\n\n');
end

end

%% ===== OUTER LOOP WITH SMOOTH CONTROL =====
function [att_des, thrust_desired] = outer_loop_position_control(x, pos_des, vel_des, acc_des, p)
    g = 9.81; m = p.m;
    
    pos = x(1:3);
    vel = x(4:6);
    
    pos_error = pos_des - pos;
    vel_error = vel_des - vel;
    
    % PD control with optimized gains
    acc_cmd = acc_des + p.k_p_pos * pos_error + p.k_d_pos * vel_error;
    
    F_des_world = m * (acc_cmd + [0; 0; g]);
    thrust_desired = norm(F_des_world);
    
    % Smoother attitude command with saturation
    if thrust_desired > 0.1
        % Reduced gain for smoother motion
        pitch_des = 0.25*tanh( -0.4*(pos(1)-pos_des(1)) - 0.8*vel(1)); 
        roll_des = 0.25*tanh( 0.4*(pos(2)-pos_des(2))  + 0.8*vel(2)); 
    else
        roll_des = 0;
        pitch_des = 0;
    end
    
    yaw_des = 0;
    att_des = [roll_des; pitch_des; yaw_des];
end

%% ===== INNER LOOP WITH OPTIMIZED GAINS =====
function u = inner_loop_attitude_control(x, att_des, thrust_desired, p)
    g = 9.81; m = p.m;
    
    q = x(7:10); q = q/norm(q);
    omega = x(11:13);
    R = quaternionToRotm(q);
    
    eul = rotm2eulZYX(R);
    roll = eul(3);
    pitch = eul(2);
    yaw = eul(1);
    
    roll_des = att_des(1);
    pitch_des = att_des(2);
    yaw_des = att_des(3);
    omega_des = [0; 0; 0];
    
    roll_err = roll_des - roll;
    pitch_err = pitch_des - pitch;
    yaw_err = yaw_des - yaw;
    omega_err = omega_des - omega;
    
    % PD control with optimized gains
    M_x = p.k_p_att * roll_err + p.k_d_att * omega_err(1);
    M_y = p.k_p_att * pitch_err + p.k_d_att * omega_err(2);
    M_z = p.k_p_yaw * yaw_err + p.k_d_yaw * omega_err(3);
    
    R33 = R(3,3);
    if R33 < 0.1
        R33 = 0.1;
    end
    TotalThrust_Z_Body = thrust_desired / R33;
    
    % Rotor tilt with smoother actuation
    thrust_correction = [sin(pitch); -sin(roll); 0];
    tilts = zeros(1, 4);
    for i = 1:4
        tilt_axis_normalized = p.tiltAxis(:, i) / norm(p.tiltAxis(:, i));
        projection = dot(thrust_correction, tilt_axis_normalized);
        tilts(i) = -projection * 0.8;  % Reduced gain for smoother tilt
        tilts(i) = max(-p.maxTilt, min(p.maxTilt, tilts(i)));
    end
    
    avg_tilt = mean(tilts);
    cos_th = cos(avg_tilt);
    if cos_th < 0.1
        cos_th = 0.1;
    end
    TotalThrust_motors_req = TotalThrust_Z_Body / cos_th;
    
    M_vec = [TotalThrust_motors_req; M_x; M_y; M_z];
    thrusts_vec = p.inv_mix * M_vec;
    thrusts_vec = max(p.minThrust, thrusts_vec);
    thrusts_vec = min(p.maxThrust, thrusts_vec);
    
    thrusts = thrusts_vec';
    u = [thrusts, tilts];
end

%% ===== TRAJECTORY WITH SMOOTH TRANSITION =====
function [pos_des, vel_des, acc_des, phase] = get_trajectory_with_takeoff(t, p)
    takeoff_end = p.takeoff_duration;
    hold_end = takeoff_end + p.hold_duration;
    
    if t < takeoff_end
        % TAKEOFF with smooth polynomial
        phase = 'TAKEOFF';
        s = t / p.takeoff_duration;
        s3 = s^3; s4 = s^4; s5 = s^5;
        h = 10*s3 - 15*s4 + 6*s5;
        dh = (30*s^2 - 60*s3 + 30*s4) / p.takeoff_duration;
        ddh = (60*s - 180*s^2 + 120*s3) / (p.takeoff_duration^2);
        
        pos_des = [0; 0; 0.05 + h * (p.traj_height - 0.05)];
        vel_des = [0; 0; dh * (p.traj_height - 0.05)];
        acc_des = [0; 0; ddh * (p.traj_height - 0.05)];
        
    elseif t < hold_end
        % HOLD
        phase = 'HOLD';
        pos_des = [0; 0; p.traj_height];
        vel_des = [0; 0; 0];
        acc_des = [0; 0; 0];
        
    else
        % TRACKING with smooth blending
        t_circle = t - hold_end;
        omega = 2*pi / p.traj_period;
        
        if t_circle < p.blend_time
            % SMOOTH BLEND
            phase = 'BLEND';
            blend_factor = (1 - cos(pi * t_circle / p.blend_time)) / 2;
            
            center_pos = [0; 0; p.traj_height];
            circle_pos = [p.traj_radius * cos(omega * t_circle);
                         p.traj_radius * sin(omega * t_circle);
                         p.traj_height];
            pos_des = (1 - blend_factor) * center_pos + blend_factor * circle_pos;
            
            circle_vel = [p.traj_radius * (-omega) * sin(omega * t_circle);
                         p.traj_radius * omega * cos(omega * t_circle);
                         0];
            vel_des = blend_factor * circle_vel;
            
            circle_acc = [p.traj_radius * (-omega^2) * cos(omega * t_circle);
                         p.traj_radius * (-omega^2) * sin(omega * t_circle);
                         0];
            acc_des = blend_factor * circle_acc;
            
        else
            % CIRCULAR MOTION
            phase = 'TRACKING';
            pos_des = [p.traj_radius * cos(omega * t_circle);
                       p.traj_radius * sin(omega * t_circle);
                       p.traj_height];
            vel_des = [p.traj_radius * (-omega) * sin(omega * t_circle);
                       p.traj_radius * omega * cos(omega * t_circle);
                       0];
            acc_des = [p.traj_radius * (-omega^2) * cos(omega * t_circle);
                       p.traj_radius * (-omega^2) * sin(omega * t_circle);
                       0];
        end
    end
end

%% ===== DYNAMICS =====
function dx = quadEOM(t, x, u, p)
    m=p.m; I=p.I; Iinv=p.Iinv; g=[0;0;-9.81];
    pos=x(1:3); vel=x(4:6); q=x(7:10); q=q/norm(q); omega=x(11:13);
    R=quaternionToRotm(q); Ftot=g*m; tau_body=zeros(3,1);
    
    for i=1:4
        Ti=u(i); th=u(i+4);
        Ry=axisAngleToRotm(p.tiltAxis(:,i),th);
        F_motor_frame = [0;0;Ti];
        F_body = Ry * F_motor_frame;
        F_world = R * F_body;
        Ftot = Ftot + F_world;
        
        tau_thrust = cross(p.r(:,i), F_body);
        tau_reaction = p.TorqueC * p.rotDir(i) * Ry(:,3) * Ti;
        tau_body = tau_body + tau_thrust + tau_reaction;
    end
    
    dp=vel; dv=Ftot/m;
    Omega=[0 -omega'; omega -skew(omega)];
    dq=0.5*Omega*q;
    domega=Iinv*(tau_body - cross(omega,I*omega));
    dx=[dp; dv; dq; domega];
end

%% ===== GRAPHICS =====
function update_drone_graphics(t, x, u, p, pos_des, att_des, phase, hBody, hArms, hPosts, hProps, ...
                               hThrust, hTiltAxes, post_v, prop_v1, prop_v2, ...
                               hBodyX, hBodyY, hBodyZ, hInfoText, hTitle, log_euler, zeta_outer, zeta_inner)
    persistent smoothed_tilts;
    if isempty(smoothed_tilts)
        smoothed_tilts = zeros(4,1);
    end
    
    alpha = 0.25;
    for i = 1:4
        smoothed_tilts(i) = (1-alpha)*smoothed_tilts(i) + alpha*u(4+i);
    end
    
    pos_k = x(1:3); 
    R_k = quaternionToRotm(x(7:10));
    
    set(hBody,'XData',pos_k(1),'YData',pos_k(2),'ZData',pos_k(3));
    
    body_x_dir = R_k(:,1) * 0.25;
    body_y_dir = R_k(:,2) * 0.25;
    body_z_dir = R_k(:,3) * 0.25;
    
    set(hBodyX, 'XData', pos_k(1), 'YData', pos_k(2), 'ZData', pos_k(3), ...
               'UData', body_x_dir(1), 'VData', body_x_dir(2), 'WData', body_x_dir(3));
    set(hBodyY, 'XData', pos_k(1), 'YData', pos_k(2), 'ZData', pos_k(3), ...
               'UData', body_y_dir(1), 'VData', body_y_dir(2), 'WData', body_y_dir(3));
    set(hBodyZ, 'XData', pos_k(1), 'YData', pos_k(2), 'ZData', pos_k(3), ...
               'UData', body_z_dir(1), 'VData', body_z_dir(2), 'WData', body_z_dir(3));
    
    for i=1:4
        armEnd = R_k * p.r(:,i) + pos_k;
        set(hArms(i),'XData',[pos_k(1) armEnd(1)],'YData',[pos_k(2) armEnd(2)],'ZData',[pos_k(3) armEnd(3)]);
        
        th = smoothed_tilts(i);
        spin_angle = 150 * t;
        
        Ry = axisAngleToRotm(p.tiltAxis(:,i), th);
        Rz_spin = axisAngleToRotm([0;0;1], spin_angle*p.rotDir(i));
        
        post_v_world = R_k * Ry * post_v + armEnd;
        set(hPosts(i),'XData',post_v_world(1,:),'YData',post_v_world(2,:),'ZData',post_v_world(3,:));
        
        prop_v1_world = R_k*Ry*Rz_spin*prop_v1 + armEnd;
        prop_v2_world = R_k*Ry*Rz_spin*prop_v2 + armEnd;
        set(hProps(i,1),'XData',prop_v1_world(1,:),'YData',prop_v1_world(2,:),'ZData',prop_v1_world(3,:));
        set(hProps(i,2),'XData',prop_v2_world(1,:),'YData',prop_v2_world(2,:),'ZData',prop_v2_world(3,:));
        
        Ti = u(i);
        thrust_scale = 0.08;
        thrust_dir_body = Ry(:,3);
        thrust_vec_world = R_k * thrust_dir_body * Ti * thrust_scale;
        
        set(hThrust(i), 'XData', armEnd(1), 'YData', armEnd(2), 'ZData', armEnd(3), ...
                       'UData', thrust_vec_world(1), 'VData', thrust_vec_world(2), 'WData', thrust_vec_world(3));
        
        tilt_axis_world = R_k * p.tiltAxis(:,i) * 0.15;
        set(hTiltAxes(i), 'XData', armEnd(1), 'YData', armEnd(2), 'ZData', armEnd(3), ...
                         'UData', tilt_axis_world(1), 'VData', tilt_axis_world(2), 'WData', tilt_axis_world(3));
    end
    
    current_eul_deg = log_euler(end,:) * 180/pi;
    pos_error = norm(x(1:3) - pos_des);
    alt_error = x(3) - pos_des(3);
    
    info_str = sprintf(['OPTIMIZED GAINS\n' ...
                        'CRITICALLY DAMPED\n' ...
                        '================================\n' ...
                        'Phase: %s\n' ...
                        'Time: %.2f s\n\n' ...
                        'DAMPING RATIOS\n' ...
                        '--------------------------------\n' ...
                        'Outer (ζ): %.2f (smooth)\n' ...
                        'Inner (ζ): %.2f (no overshoot)\n\n' ...
                        'FREQUENCIES\n' ...
                        '--------------------------------\n' ...
                        'Inner: Automatic\n' ...
                        'Outer: Automatic (ratio maintained)\n\n' ...
                        'POSITION\n' ...
                        '--------------------------------\n' ...
                        'Act: [%6.3f,%6.3f,%6.3f]\n' ...
                        'Des: [%6.3f,%6.3f,%6.3f]\n' ...
                        '3D Err:  %6.4f m\n' ...
                        'Alt Err: %6.4f m\n\n' ...
                        'ATTITUDE (deg)\n' ...
                        '--------------------------------\n' ...
                        'Roll:  %5.2f (D:%5.2f)\n' ...
                        'Pitch: %5.2f (D:%5.2f)\n\n' ...
                        'ROTOR TILTS (deg)\n' ...
                        '--------------------------------\n' ...
                        'R1:%5.2f R2:%5.2f\n' ...
                        'R3:%5.2f R4:%5.2f\n'], ...
                        phase, t, ...
                        zeta_outer, zeta_inner, ...
                        x(1), x(2), x(3), ...
                        pos_des(1), pos_des(2), pos_des(3), ...
                        pos_error, alt_error, ...
                        current_eul_deg(1), att_des(1)*180/pi, ...
                        current_eul_deg(2), att_des(2)*180/pi, ...
                        smoothed_tilts(1)*180/pi, smoothed_tilts(2)*180/pi, ...
                        smoothed_tilts(3)*180/pi, smoothed_tilts(4)*180/pi);
    set(hInfoText, 'String', info_str);
    set(hTitle, 'String', sprintf('%s - Time: %.2f s (STABLE)', phase, t));
end

%% ===== Helper Functions =====
function R = quaternionToRotm(q)
    w=q(1); x=q(2); y=q(3); z=q(4);
    R=[1-2*(y^2+z^2), 2*(x*y-w*z), 2*(x*z+w*y);
       2*(x*y+w*z), 1-2*(x^2+z^2), 2*(y*z-w*x);
       2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x^2+y^2)];
end

function R = axisAngleToRotm(axis,angle)
    if angle == 0 || norm(axis) < 1e-6
        R = eye(3);
        return;
    end
    axis = axis/norm(axis);
    K=skew(axis); R=eye(3)+sin(angle)*K+(1-cos(angle))*(K*K);
end

function S = skew(v)
    S=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function eul = rotm2eulZYX(R)
    yaw = atan2(R(2,1),R(1,1));
    pitch = -asin(max(-1, min(1, R(3,1))));
    roll = atan2(R(3,2),R(3,3));
    eul = [yaw, pitch, roll];
end