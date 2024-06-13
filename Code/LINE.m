
clear all; clc; close all;

%% Simulation Parameters
dt = 0.09;
ts = 15;  
t = 0:dt:ts; 
L=0.4;


%% Initial Conditions
%Leader
x0 = -13;
y0 = -10;
psi0 = 0; 
eta0 = [x0; y0; psi0];
eta(:, 1) = eta0;


%Follower 1
x0_2 = 6; 
y0_2 = 6; 
psi0_2 = 0; 
eta0_2 = [x0_2; y0_2; psi0_2];
eta_2(:, 1) = eta0_2;

%Follower 2
x0_3 = -9; 
y0_3 = -6; 
psi0_3 = 0;
eta0_3 = [x0_3; y0_3; psi0_3];
eta_3(:, 1) = eta0_3;

%Follower 3
x0_4 = 12; 
y0_4 = 12; 
psi0_4 = 0; 
eta0_4 = [x0_4; y0_4; psi0_4];
eta_4(:, 1) = eta0_4;


%Follower 4
x0_5 = -18; 
y0_5 = -12; 
psi0_5 = 0; 
eta0_5 = [x0_5; y0_5; psi0_5];
eta_5(:, 1) = eta0_5;


%% Control Law Parameters

k_v = 1;  

L_dx = 6;  % Desired distance between leader and follower
L_dy = 0;  % Desired distance between leader and follower

L_dx_2 = 8;  % Desired distance between leader and follower
L_dy_2 = 0;

L_dx_3 = 6;  % Desired distance between leader and follower
L_dy_3 = 0;

L_dx_4 = 8;  % Desired distance between leader and follower
L_dy_4 = 0;


%% Loop starts here
for i = 1:length(t)
  % Current time
  current_time = t(i);

  % Leader Robot (assuming constant velocity)
  v_i_star = 2; 
  v_ix = v_i_star * cos(psi0);
  v_iy = v_i_star * sin(psi0);

  % Follower Robot
  % Calculate distance between robots in x and y directions
  dx = eta(1, i) - eta_2(1, i);
  dy = eta(2, i) - eta_2(2, i);

  % second Follower
    dx_2 = eta(1, i) - eta_3(1, i);
    dy_2 = eta(2, i) - eta_3(2, i);

    % third Follower
    dx_3 = eta_2(1, i) - eta_4(1, i);
    dy_3 = eta_2(2, i) - eta_4(2, i);

       % fourth Follower
    dx_4 = eta_3(1, i) - eta_5(1, i);
    dy_4 = eta_3(2, i) - eta_5(2, i);


  % Apply control law for follower velocity in x and y directions
  v_jx = -k_v*(L_dx - dx ) + v_ix*cos(psi0)
  v_jy = -k_v*(L_dy - dy ) + v_iy*sin(psi0)

   v_jx_2 = -k_v*(L_dx_2 - dx_2 ) + v_ix*cos(psi0)
  v_jy_2 = -k_v*(L_dy_2 - dy_2 ) + v_iy*sin(psi0)


  v_jx_3 = -k_v*(L_dx_3 - dx_3 ) + v_jx*cos(psi0)
  v_jy_3 = -k_v*(L_dy_3 - dy_3 ) + v_jy*sin(psi0)


 v_jx_4 = -k_v*(L_dx_4 - dx_4 ) + v_jx_2*cos(psi0);
  v_jy_4 = -k_v*(L_dy_4 - dy_4 ) + v_jy_2*sin(psi0);

   
  v_jx_all(i, :) = [v_ix,v_jx, v_jx_2, v_jx_3, v_jx_4];
  v_jy_all(i, :) = [v_iy,v_jy, v_jy_2, v_jy_3, v_jy_4];

  % Follower position update

  eta(1, i + 1 )= eta(1, i) + dt * v_ix;
  eta(2, i + 1) = eta(2, i) + dt * v_iy;
  
  eta_2(1, i + 1) = eta_2(1, i) + dt * v_jx;
  eta_2(2, i + 1) = eta_2(2, i) + dt * v_jy;

  eta_3(1, i + 1) = eta_3(1, i) + dt * v_jx_2;
  eta_3(2, i + 1) = eta_3(2, i) + dt * v_jy_2;

  eta_4(1, i + 1) = eta_4(1, i) + dt * v_jx_3;
  eta_4(2, i + 1) = eta_4(2, i) + dt * v_jy_3;

   eta_5(1, i + 1) = eta_5(1, i) + dt * v_jx_4;
  eta_5(2, i + 1) = eta_5(2, i) + dt * v_jy_4;
  
end

%% Animation
figure
for i = 1:length(t)
    % Extract current positions and orientations
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

      x_follower4 = eta_5(1, i);
    y_follower4 = eta_5(2, i);
    psi_follower4 = eta_5(3, i);


    % Plot the trajectories
    plot(eta(1, 1:i), eta(2, 1:i), 'r-', 'LineWidth', 1.5); % Leader trajectory
    hold on
    plot(eta_2(1, 1:i), eta_2(2, 1:i), 'g-', 'LineWidth', 1.5); % Follower 1 trajectory
    plot(eta_3(1, 1:i), eta_3(2, 1:i), 'y-', 'LineWidth', 1.5); % Follower 2 trajectory
    plot(eta_4(1, 1:i), eta_4(2, 1:i), 'b-', 'LineWidth', 1.5); % Follower 3 trajectory
     plot(eta_5(1, 1:i), eta_5(2, 1:i), 'm-', 'LineWidth', 1.5); % Follower 4 trajectory
    
    % Plot the leader robot
    plot(x_leader, y_leader, 'ro', 'MarkerSize', 10); 
    plot(x_leader + L/2*cos(psi_leader + pi/2), y_leader + L/2*sin(psi_leader + pi/2), 'b.', 'MarkerSize', 10); % Plot the left wheel
    plot(x_leader - L/2*cos(psi_leader + pi/2), y_leader - L/2*sin(psi_leader + pi/2), 'b.', 'MarkerSize', 10); % Plot the right wheel
    
    % Plot the follower robots
    plot(x_follower1, y_follower1, 'go', 'MarkerSize', 10);
    plot(x_follower1 + L/2*cos(psi_follower1 + pi/2), y_follower1 + L/2*sin(psi_follower1 + pi/2), 'b.', 'MarkerSize', 10); % Plot the left wheel
    plot(x_follower1 - L/2*cos(psi_follower1 + pi/2), y_follower1 - L/2*sin(psi_follower1 + pi/2), 'b.', 'MarkerSize', 10); % Plot the right wheel
    
    plot(x_follower2, y_follower2, 'yo', 'MarkerSize', 10); 
    plot(x_follower2 + L/2*cos(psi_follower2 + pi/2), y_follower2 + L/2*sin(psi_follower2 + pi/2), 'b.', 'MarkerSize', 10); % Plot the left wheel
    plot(x_follower2 - L/2*cos(psi_follower2 + pi/2), y_follower2 - L/2*sin(psi_follower2 + pi/2), 'b.', 'MarkerSize', 10); % Plot the right wheel
    
    plot(x_follower3, y_follower3, 'bo', 'MarkerSize', 10); 
    plot(x_follower3 + L/2*cos(psi_follower3 + pi/2), y_follower3 + L/2*sin(psi_follower3 + pi/2), 'b.', 'MarkerSize', 10); % Plot the left wheel
    plot(x_follower3 - L/2*cos(psi_follower3 + pi/2), y_follower3 - L/2*sin(psi_follower3 + pi/2), 'b.', 'MarkerSize', 10); % Plot the right wheel

    plot(x_follower4, y_follower4, 'bo', 'MarkerSize', 10);
    plot(x_follower4 + L/2*cos(psi_follower4 + pi/2), y_follower4 + L/2*sin(psi_follower4 + pi/2), 'b.', 'MarkerSize', 10); % Plot the left wheel
    plot(x_follower4 - L/2*cos(psi_follower4 + pi/2), y_follower4 - L/2*sin(psi_follower4 + pi/2), 'b.', 'MarkerSize', 10); % Plot the right wheel
    legend('Leader','Follower 1', 'Follower 2', 'Follower 1-1', 'Follower 2-1');
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
plot(t, v_jx_all(:, 1), 'r-', 'LineWidth', 1.5); % Follower 1 control velocity x
hold on;
plot(t, v_jx_all(:, 2), 'g-', 'LineWidth', 1.5); % Follower 2 control velocity x
plot(t, v_jx_all(:, 3), 'y-', 'LineWidth', 1.5); % Follower 3 control velocity x
plot(t, v_jx_all(:, 4), 'b-', 'LineWidth', 1.5); % Follower 4 control velocity x
plot(t, v_jx_all(:, 5), 'm-', 'LineWidth', 1.5); % Follower 4 control velocity x
xlabel('Time (s)');
ylabel('Control Velocity (m/s)');
title('Control Velocities of Followers in X direction');
legend('Leader','Follower 1', 'Follower 2', 'Follower 1-1', 'Follower 2-1');
grid on;

figure;
plot(t, v_jy_all(:, 1), 'r-', 'LineWidth', 1.5); % Follower 1 control velocity y
hold on;
plot(t, v_jy_all(:, 2), 'g-', 'LineWidth', 1.5); % Follower 2 control velocity y
plot(t, v_jy_all(:, 3), 'y-', 'LineWidth', 1.5); % Follower 3 control velocity y
plot(t, v_jy_all(:, 4), 'b-', 'LineWidth', 1.5); % Follower 4 control velocity y
plot(t, v_jy_all(:, 5), 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Velocity (m/s)');
title('Control Velocities of Followers in Y direction');
legend('Leader','Follower 1', 'Follower 2', 'Follower 1-1', 'Follower 2-1');
grid on;

