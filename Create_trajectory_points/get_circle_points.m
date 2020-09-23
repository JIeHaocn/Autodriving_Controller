function [points_x_return, points_y_return, points_theta_return] = get_circle_points(x0, y0, theta0, theta_d, radius, velocity, sample_time)
%   get_line_points  Calculate line segment points
%   x0, y0, theta0: initial state of the car
%   theta_d: desired central angle of the circle arc
%   radius: radius of the circle arc
%   velocity: velocity of the car
%   sample_time

    points_x = zeros(5000,1);
    points_y = zeros(5000,1);
    points_theta = zeros(5000,1);
    delta_theta = velocity/radius*sample_time;
    theta_pass = 0;
    index = 0;
    trans_matrix = [cos(theta0) -sin(theta0) x0; sin(theta0) cos(theta0) y0; 0 0 1];
    
    if(theta_d > 0)
        notarrive = theta_pass <= theta_d+1e-10;
        while(notarrive)
            index = index+1;
            theta_pass = delta_theta*index;
            notarrive = theta_pass <= theta_d+1e-10;
            points_x(index) = radius*sin(theta_pass);
            points_y(index) = radius*(1-cos(theta_pass));
            points_theta(index) = theta0 + theta_pass;
            if(~notarrive)
                points_x(index) = radius*sin(theta_d);
                points_y(index) = radius*(1-cos(theta_d));
                points_theta(index) = theta0 + theta_d;
            end
            temp = trans_matrix*[points_x(index) points_y(index) 1]';
            points_x(index) = temp(1);
            points_y(index) = temp(2);
        end
    else
        notarrive = theta_pass <= abs(theta_d)+1e-10;
        while(notarrive)
            index = index+1;
            theta_pass = delta_theta*index;
            notarrive = theta_pass <= abs(theta_d)+1e-10;
            points_x(index) = radius*sin(theta_pass);
            points_y(index) = -radius*(1-cos(theta_pass));
            points_theta(index) = theta0 - theta_pass;
            if(~notarrive)
                points_x(index) = radius*sin(-theta_d);
                points_y(index) = -radius*(1-cos(-theta_d));
                points_theta(index) = theta0 + theta_d;
            end
            temp = trans_matrix*[points_x(index) points_y(index) 1]';
            points_x(index) = temp(1);
            points_y(index) = temp(2);
        end
    end
    
    points_x_return = points_x(1:index,1);
    points_y_return = points_y(1:index,1);
    points_theta_return = points_theta(1:index,1);
end