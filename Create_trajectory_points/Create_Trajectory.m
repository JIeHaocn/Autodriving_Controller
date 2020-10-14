classdef Create_Trajectory
%   Create_Trajectory    
%   Class definition of creating predefinied trajectory points 
%   The trajectory includes a set of line segments and circle arcs where the parameters (e.g., length, radius) are definitely known
%   add_line(length): import the length of line segment
%   add_circle(theta_d,radius): import the desired central angle and radius of circle arc
%   plot_traj: plot trajectory
%   ani_traj
%   print_file

    properties
        sample_time = 0.1   % unit: s
        velocity = 0.5      % unit: m/s
        x0 = 0               % unit: m
        y0 = 0               % unit: m
        theta0 = 0           % unit: degree    
        traj_x
        traj_y
        traj_theta
        traj_curvature
        traj_speed
    end
    properties
        Lw = 0.256
        Lf = 0.07 
        Lr = 0.05
        W = 0.20
        Tire_length = 0.066
        Tire_width_f = 0.026
        Tire_width_r = 0.03
        Trans_mat
        Tire_Trans_mat  
    end
    
    methods
        function obj = Create_Trajectory(sample_time,velocity,x0,y0,theta0)
            obj.sample_time = sample_time;
            obj.velocity = velocity;
            obj.x0 = x0;
            obj.y0 = y0;
            obj.theta0 = theta0/180*pi;
            obj.Trans_mat = [ obj.Lw + obj.Lf,  obj.W/2, 1; ...
                              obj.Lw + obj.Lf, -obj.W/2, 1; ...
                                      -obj.Lr, -obj.W/2, 1; ...
                                      -obj.Lr,  obj.W/2, 1];
            obj.Tire_Trans_mat =  [ obj.Lw + obj.Tire_length/2,  obj.W/2 + obj.Tire_width_f/2, 1; ...
                                    obj.Lw + obj.Tire_length/2,  obj.W/2 - obj.Tire_width_f/2, 1; ...
                                    obj.Lw - obj.Tire_length/2,  obj.W/2 - obj.Tire_width_f/2, 1; ...
                                    obj.Lw - obj.Tire_length/2,  obj.W/2 + obj.Tire_width_f/2, 1; ...
                                    obj.Lw + obj.Tire_length/2, -obj.W/2 + obj.Tire_width_f/2, 1; ...
                                    obj.Lw + obj.Tire_length/2, -obj.W/2 - obj.Tire_width_f/2, 1; ...
                                    obj.Lw - obj.Tire_length/2, -obj.W/2 - obj.Tire_width_f/2, 1; ...
                                    obj.Lw - obj.Tire_length/2, -obj.W/2 + obj.Tire_width_f/2, 1; ...
                                             obj.Tire_length/2, -obj.W/2 + obj.Tire_width_r/2, 1; ...
                                             obj.Tire_length/2, -obj.W/2 - obj.Tire_width_r/2, 1; ...
                                            -obj.Tire_length/2, -obj.W/2 - obj.Tire_width_r/2, 1; ...
                                            -obj.Tire_length/2, -obj.W/2 + obj.Tire_width_r/2, 1; ...
                                             obj.Tire_length/2,  obj.W/2 + obj.Tire_width_r/2, 1; ...
                                             obj.Tire_length/2,  obj.W/2 - obj.Tire_width_r/2, 1; ...
                                            -obj.Tire_length/2,  obj.W/2 - obj.Tire_width_r/2, 1; ...
                                            -obj.Tire_length/2,  obj.W/2 + obj.Tire_width_r/2, 1];                    
        end
        
        function obj = add_line(obj,length)
            if isempty(obj.traj_x) 
                [traj_x_temp, traj_y_temp, traj_theta_temp] = get_line_points(obj.x0, obj.y0, obj.theta0, length, obj.velocity, obj.sample_time);
            else
                [traj_x_temp, traj_y_temp, traj_theta_temp] = get_line_points(obj.traj_x(end), obj.traj_y(end), obj.traj_theta(end), length, obj.velocity, obj.sample_time);
            end
            traj_curvature_temp = zeros(size(traj_x_temp,1),1);
            traj_speed_temp =  zeros(size(traj_x_temp,1),1);
            traj_speed_temp(:) = obj.velocity;
            obj.traj_x = [obj.traj_x;traj_x_temp];
            obj.traj_y = [obj.traj_y;traj_y_temp];
            obj.traj_theta = [obj.traj_theta;traj_theta_temp];
            obj.traj_curvature = [obj.traj_curvature;traj_curvature_temp];
            obj.traj_speed = [obj.traj_speed;traj_speed_temp];
        end
        
        function obj = add_circle(obj,theta_d,radius)
            if isempty(obj.traj_x) 
                [traj_x_temp, traj_y_temp, traj_theta_temp] = get_circle_points(obj.x0, obj.y0, obj.theta0, theta_d, radius, obj.velocity, obj.sample_time);
            else
                [traj_x_temp, traj_y_temp, traj_theta_temp] = get_circle_points(obj.traj_x(end), obj.traj_y(end), obj.traj_theta(end), theta_d, radius, obj.velocity, obj.sample_time);
            end
            traj_curvature_temp = zeros(size(traj_x_temp,1),1);
            traj_curvature_temp(:) = 1/radius;
            traj_speed_temp =  zeros(size(traj_x_temp,1),1);
            traj_speed_temp(:) = obj.velocity;
            obj.traj_x = [obj.traj_x;traj_x_temp];
            obj.traj_y = [obj.traj_y;traj_y_temp];
            obj.traj_theta = [obj.traj_theta;traj_theta_temp];
            obj.traj_curvature = [obj.traj_curvature;traj_curvature_temp];
            obj.traj_speed = [obj.traj_speed;traj_speed_temp];
        end
        
        function plot_traj(obj)
            % Plot the trajectory
            figure('Name','Desired Trajectory')
            plot(obj.traj_x, obj.traj_y);
        end 
        
        function ani_traj(obj)
            x_left = min(obj.traj_x)-0.5;
            x_right = max(obj.traj_x)+0.5;
            y_left = min(obj.traj_y)-0.5;
            y_right = max(obj.traj_y)+0.5;
            figure('Name','Trajectory Animation ')            
            for i=1:size(obj.traj_x,1)             
                plot(obj.traj_x(1:i), obj.traj_y(1:i),'-b');
                hold on               
                vertex_x = obj.Trans_mat*[cos(obj.traj_theta(i)); -sin(obj.traj_theta(i)); obj.traj_x(i)];
                vertex_y = obj.Trans_mat*[sin(obj.traj_theta(i));  cos(obj.traj_theta(i)); obj.traj_y(i)];
                plot([vertex_x(1) vertex_x(2)], [vertex_y(1) vertex_y(2)],'-k');
                plot([vertex_x(2) vertex_x(3)], [vertex_y(2) vertex_y(3)],'-k');
                plot([vertex_x(3) vertex_x(4)], [vertex_y(3) vertex_y(4)],'-k');
                plot([vertex_x(4) vertex_x(1)], [vertex_y(4) vertex_y(1)],'-k');
                tire_x = obj.Tire_Trans_mat*[cos(obj.traj_theta(i)); -sin(obj.traj_theta(i)); obj.traj_x(i)];
                tire_y = obj.Tire_Trans_mat*[sin(obj.traj_theta(i));  cos(obj.traj_theta(i)); obj.traj_y(i)];  
                for i_tire = 0:3
                    plot([tire_x(i_tire*4+1) tire_x(i_tire*4+2)], [tire_y(i_tire*4+1) tire_y(i_tire*4+2)],'-k');
                    plot([tire_x(i_tire*4+2) tire_x(i_tire*4+3)], [tire_y(i_tire*4+2) tire_y(i_tire*4+3)],'-k');
                    plot([tire_x(i_tire*4+3) tire_x(i_tire*4+4)], [tire_y(i_tire*4+3) tire_y(i_tire*4+4)],'-k');
                    plot([tire_x(i_tire*4+4) tire_x(i_tire*4+1)], [tire_y(i_tire*4+4) tire_y(i_tire*4+1)],'-k');
                end
                axis([x_left x_right y_left y_right])
                pause(0.02);
                hold off
            end        
        end
        
        function print_file(obj,num)
%             traj_theta_deg = obj.traj_theta/pi*180; % unit: degree
            fid=fopen('route.txt','w');
            if num == 3
                for i=1:size(obj.traj_x,1)
                    fprintf(fid,'%.6f %.6f %.6f\n',obj.traj_x(i), obj.traj_y(i), obj.traj_speed(i)); 
                end
            elseif num == 4
                for i=1:size(obj.traj_x,1)
                    fprintf(fid,'%.6f %.6f %.6f %.6f\n',obj.traj_x(i), obj.traj_y(i), obj.traj_theta(i), obj.traj_speed(i));  
                end
            end
        end
        
    end
end

