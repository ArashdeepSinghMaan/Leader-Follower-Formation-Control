
clear all; clc; close all;

%% Simulation Parameters
dt = 0.08; 
ts = 40;  
t = 0:dt:ts; 
L=0.4;

omega = 0.4; 

%% Initial Conditions
%Leader
x0 = 40;
y0 = -10;
psi0 = pi/4;
eta0 = [x0; y0; psi0];
eta(:, 1) = eta0;

%Follower 1
x0_2 = -2; 
y0_2 = -4;
psi0_2 = pi/2; 
eta0_2 = [x0_2; y0_2; psi0_2];
eta_2(:, 1) = eta0_2;

%Follower 2
x0_3 = 2; 
y0_3 = 4; 
psi0_3 = pi; 
eta0_3 = [x0_3; y0_3; psi0_3];
eta_3(:, 1) = eta0_3;

%Follower 3
x0_4 = 2; 
y0_4 = 6; 
psi0_4 = pi/3; 
eta0_4 = [x0_4; y0_4; psi0_4];
eta_4(:, 1) = eta0_4;

%% Control Law Parameters
k_v = 0.99;

L_dx = 2;  
L_dy = 2;  

L_dx_2 = 4;  
L_dy_2 = 4;

L_dx_3 = 6; 
L_dy_3 = 6;

phi_d=pi/12;


A_curve = 8;  % Amplitude of the sinusoidal curve
A_line = 12;   % Amplitude of the straight-line segment
omega_curve = 0.4;  % Angular frequency of the sinusoidal curve


%% Loop starts here
for i = 1:length(t)
 
  current_time = t(i);
  % Define the leader's trajectory
x_leader = x0 + A_line * current_time + A_curve * sin(omega_curve * current_time);
y_leader = y0 + A_line * current_time + A_curve * cos(omega_curve * current_time);

% Calculate the orientation of the leader
if current_time < ts / 3
    % For the straight-line segment
    psi_leader = atan2(A_line, A_curve * omega_curve * cos(omega_curve * current_time));
elseif current_time < 2*ts / 3
    % For the sinusoidal curve segment
    psi_leader = atan2(A_curve * omega_curve * cos(omega_curve * current_time), ...
                       -A_line + A_curve * omega_curve * sin(omega_curve * current_time));
else
    % For the straight-line segment again
    psi_leader = atan2(A_line, A_curve * omega_curve * cos(omega_curve * current_time));
end



  

  % Follower Robot
  % Calculate distance between robots in x and y directions
  dx = eta(1, i) - eta_2(1, i);
  dy = eta(2, i) - eta_2(2, i);
   
  % second Follower
    dx_2 = eta(1, i) - eta_3(1, i);
    dy_2 = eta(2, i) - eta_3(2, i);

    % third Follower
    dx_3 = eta(1, i) - eta_4(1, i);
    dy_3 = eta(2, i) - eta_4(2, i);


  % Apply control law for follower velocity in x and y directions
  v_i_star = 4;
  v_ix = v_i_star * cos(psi_leader);
  v_iy = v_i_star * sin(psi_leader);
  
  v_jx = -k_v*(L_dx - dx ) + v_ix*cos(psi_leader)
 
  v_jy = -k_v*(L_dy - dy ) + v_iy*sin(psi_leader)
 
v_j_1=sqrt(v_jx^2+v_jy^2);
  v_jx_2 = -k_v*(L_dx_2 - dx_2 ) + v_ix*cos(psi_leader);
  v_jy_2 = -k_v*(L_dy_2 - dy_2 ) + v_iy*sin(psi_leader);
    v_j_2=sqrt(v_jx_2^2+v_jy_2^2);
  v_jx_3 = -k_v*(L_dx_3 - dx_3 ) + v_ix*cos(psi_leader);
  v_jy_3 = -k_v*(L_dy_3 - dy_3 ) + v_iy*sin(psi_leader);
       v_j_3=sqrt(v_jx_3^2+v_jy_3^2);
  % Store velocities
  v_jx_all(i, :) = [v_ix,v_jx, v_jx_2, v_jx_3];
  v_jy_all(i, :) = [v_iy,v_jy,v_jy_2 , v_jy_3];

 

    
  %% position update
  eta(1, i + 1 )= eta(1, i) + dt * v_ix;
  eta(2, i + 1) = eta(2, i) + dt * v_iy;
  eta(3, i + 1) = psi_leader; 

  eta_2(1, i + 1) = eta_2(1, i) + dt * v_jx;
  eta_2(2, i + 1) = eta_2(2, i) + dt * v_jy;
  
  eta_3(1, i + 1) = eta_3(1, i) + dt * v_jx_2;
  eta_3(2, i + 1) = eta_3(2, i) + dt * v_jy_2;
 

  eta_4(1, i + 1) = eta_4(1, i) + dt * v_jx_3;
  eta_4(2, i + 1) = eta_4(2, i) + dt * v_jy_3;
 

%% Updating orientation using angular velocity control
eta_2(3, i + 1) = eta_2(3, i)+ ((-phi_d -eta_2(3,i)+eta(3,i) ) +  (omega ))*dt;
 eta_3(3, i + 1) = eta_3(3, i)+ ((-phi_d - eta_3(3,i)+eta(3,i)) + ( omega))*dt;
 eta_4(3, i + 1) = eta_4(3, i) + ((-phi_d -eta_4(3,i)+eta(3,i))+ ( omega))*dt;


end

%% Animation
figure
for i = 1:length(t)
    
    x_leader = eta(1, i);
    y_leader = eta(2, i);
    psi_leader = eta(3, i);
    
    x_follower1 = eta_2(1, i);
    y_follower1 = eta_2(2, i);
    psi_follower1 = eta_2(3, i);
    
    x_follower2 = eta_3(1, i);
    y_follower2 = eta_3(2, i);
    psi_follower2 = eta_3(3, i);
    
    x_follower3 = eta_4(1, i);
    y_follower3 = eta_4(2, i);
    psi_follower3 = eta_4(3, i);

    % Plot the trajectories
    plot(eta(1, 1:i), eta(2, 1:i), 'r-', 'LineWidth', 1.5); 
    hold on
    plot(eta_2(1, 1:i), eta_2(2, 1:i), 'g-', 'LineWidth', 1.5);
    plot(eta_3(1, 1:i), eta_3(2, 1:i), 'y-', 'LineWidth', 1.5); 
    plot(eta_4(1, 1:i), eta_4(2, 1:i), 'b-', 'LineWidth', 1.5); 
    
    % Plot the leader robot
    plot(x_leader, y_leader, 'ro', 'MarkerSize', 10); 
    plot(x_leader + L/2*cos(psi_leader + pi/2), y_leader + L/2*sin(psi_leader + pi/2), 'b.', 'MarkerSize', 10); 
    plot(x_leader - L/2*cos(psi_leader + pi/2), y_leader - L/2*sin(psi_leader + pi/2), 'b.', 'MarkerSize', 10); 
    
    % Plot the follower robots
    plot(x_follower1, y_follower1, 'go', 'MarkerSize', 10);
    plot(x_follower1 + L/2*cos(psi_follower1 + pi/2), y_follower1 + L/2*sin(psi_follower1 + pi/2), 'b.', 'MarkerSize', 10);
    plot(x_follower1 - L/2*cos(psi_follower1 + pi/2), y_follower1 - L/2*sin(psi_follower1 + pi/2), 'b.', 'MarkerSize', 10); 
    
    plot(x_follower2, y_follower2, 'yo', 'MarkerSize', 10); 
    plot(x_follower2 + L/2*cos(psi_follower2 + pi/2), y_follower2 + L/2*sin(psi_follower2 + pi/2), 'b.', 'MarkerSize', 10); 
    plot(x_follower2 - L/2*cos(psi_follower2 + pi/2), y_follower2 - L/2*sin(psi_follower2 + pi/2), 'b.', 'MarkerSize', 10); 
    
    plot(x_follower3, y_follower3, 'bo', 'MarkerSize', 10); 
    plot(x_follower3 + L/2*cos(psi_follower3 + pi/2), y_follower3 + L/2*sin(psi_follower3 + pi/2), 'b.', 'MarkerSize', 10); 
    plot(x_follower3 - L/2*cos(psi_follower3 + pi/2), y_follower3 - L/2*sin(psi_follower3 + pi/2), 'b.', 'MarkerSize', 10);
    legend('Leader','Follower 1', 'Follower 2', 'Follower 3');
    axis equal;
    title('Differential Drive Wheeled Mobile Robots');
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;
    pause(0.1);
    hold off
end

%% Plot control velocities
figure;
plot(t, v_jx_all(:, 1), 'r-', 'LineWidth', 1.5); 
hold on;
plot(t, v_jx_all(:, 2), 'g-', 'LineWidth', 1.5); 
plot(t, v_jx_all(:, 3), 'b-', 'LineWidth', 1.5);
plot(t, v_jx_all(:, 4), 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Velocity (m/s)');
title('Control Velocities of Followers in X direction');
legend('Leader','Follower 1', 'Follower 2', 'Follower 1-1');
grid on;

figure;
plot(t, v_jy_all(:, 1), 'r-', 'LineWidth', 1.5); 
hold on;
plot(t, v_jy_all(:, 2), 'g-', 'LineWidth', 1.5);
plot(t, v_jy_all(:, 3), 'b-', 'LineWidth', 1.5); 
plot(t, v_jx_all(:, 4), 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Velocity (m/s)');
title('Control Velocities of Followers in Y direction');
legend ('Leader','Follower 1', 'Follower 2', 'Follower 1-1');
grid on;


%% Initialize arrays to store angular velocities and orientations
omega_all = zeros(length(t), 4); % Array to store angular velocities
orientation_all = zeros(length(t), 4); % Array to store orientations

%% Calculate angular velocities and orientations during the simulation
for i = 1:length(t)
    % Calculate angular velocities
    omega_all(i, 1) = omega; % Angular velocity of the leader
    omega_all(i, 2) = (phi_d - eta_2(3, i)+eta(3,i)) + omega; % Angular velocity of Follower 1
    omega_all(i, 3) = (phi_d - eta_3(3, i)+eta(3,i)) + omega; % Angular velocity of Follower 2
    omega_all(i, 4) = (phi_d - eta_4(3, i)+eta(3,i)) + omega; % Angular velocity of Follower 3
    
    % Store orientations
    orientation_all(i, 1) = eta(3, i); % Orientation of the leader
    orientation_all(i, 2) = eta_2(3, i); % Orientation of Follower 1
    orientation_all(i, 3) = eta_3(3, i); % Orientation of Follower 2
    orientation_all(i, 4) = eta_4(3, i); % Orientation of Follower 3
end

%% Plot angular velocities
figure;
plot(t, omega_all(:, 1), 'r-', 'LineWidth', 1.5); 
hold on;
plot(t, omega_all(:, 2), 'g-', 'LineWidth', 1.5); 
plot(t, omega_all(:, 3), 'b-', 'LineWidth', 1.5);
plot(t, omega_all(:, 4), 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocities of Vehicles');
legend('Leader', 'Follower 1', 'Follower 2', 'Follower 3');
grid on;

%% Plot orientations
figure;
plot(t, orientation_all(:, 1), 'r-', 'LineWidth', 1.5); 
hold on;
plot(t, orientation_all(:, 2), 'g-', 'LineWidth', 1.5); 
plot(t, orientation_all(:, 3), 'b-', 'LineWidth', 1.5);
plot(t, orientation_all(:, 4), 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('Orientations of Vehicles');
legend('Leader', 'Follower 1', 'Follower 2', 'Follower 3');
grid on;



