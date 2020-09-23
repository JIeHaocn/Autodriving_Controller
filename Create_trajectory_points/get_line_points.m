function [points_x_return, points_y_return, points_theta_return] = get_line_points(x0, y0, theta0, length, velocity, sample_time)
    points_x = zeros(5000,1);
    points_y = zeros(5000,1);
    points_theta = zeros(5000,1);
    distance = 0;
    index = 0;
    trans_matrix = [cos(theta0) -sin(theta0) x0; sin(theta0) cos(theta0) y0; 0 0 1];
    notarrive = distance <= length;
    while(notarrive)
        index = index+1;
        distance = velocity*sample_time*index;
        notarrive = distance <= length;
        points_x(index) = distance;
        points_y(index) = 0;
        points_theta(index) = theta0;
        if(~notarrive)
            points_x(index) = length;
            points_y(index) = 0;
            points_theta(index) = theta0;
        end
        temp = trans_matrix*[points_x(index) points_y(index) 1]';
        points_x(index) = temp(1);
        points_y(index) = temp(2);      
    end
    points_x_return = points_x(1:index,1);
    points_y_return = points_y(1:index,1);
    points_theta_return = points_theta(1:index,1);
end