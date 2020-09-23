clear all
close all
sample_time = 0.1; % unit: s
velocity = 0.5; % unit: m/s
x0 = 0; % unit: m
y0 = 0; % unit: m
theta0 = 0; % unit: degree
theta0 = theta0/180*pi;
Car_Traj = Create_Trajectory(sample_time,velocity,x0,y0,theta0);

length = 1.7; % unit: m
Car_Traj = Car_Traj.add_line(length);

theta_d = pi; % unit: rad
radius = 0.9; % unit: m
Car_Traj = Car_Traj.add_circle(theta_d,radius);

length = 2; % unit: m
Car_Traj = Car_Traj.add_line(length);

theta_d = -pi/2; % unit: rad
radius = 1.0; % unit: m
Car_Traj = Car_Traj.add_circle(theta_d,radius);

theta_d = -pi/3; % unit: rad
radius = 1.2; % unit: m
Car_Traj = Car_Traj.add_circle(theta_d,radius);

length = 1.2; % unit: m
Car_Traj = Car_Traj.add_line(length);

theta_d = pi/4; % unit: rad
radius = 1.1; % unit: m
Car_Traj = Car_Traj.add_circle(theta_d,radius);

% Plot the trajectory
Car_Traj.plot_traj;

% Play the animation
Car_Traj.ani_traj;

Car_Traj.print_file;

traj_x = Car_Traj.traj_x;
traj_y = Car_Traj.traj_x;
traj_theta = Car_Traj.traj_x;
traj_curvature = Car_Traj.traj_x;
traj_speed = Car_Traj.traj_x;
