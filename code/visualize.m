% Visualize the quadcopter simulation as an animation of a 3D quadcopter.
function h = visualize(data,xyHis)
    %3D Visualization of Quadrotor
    [~,n,loop]=size(xyHis);
    xyz=reshape(xyHis,3*n,loop);
    t = 1:2:length(data.t);
    figure; 
    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    [t, ~] = quadcopter;

    % Set axis scale and labels.
    axis([-2 5 -4 5 1 5]);
    zlabel('Height');
    title('Quadcopter Flight Simulation');
    grid on;
    % Animate the quadcopter with data from the simulation.
    plot3(data.x(1,:),data.x(2,:),data.x(3,:),'c--');
    %legend('Actual Path')
    hold on;
    animate(data, t, xyz, n);
end

function animate(data, model, xyz, n)
    for t = 1:5:length(data.t) %skip frames

        dx = data.x(:, t);
        move = makehgtform('translate', dx);

        % Compute rotation to correct angles
        angles = data.theta(:, t);
        rotate = rotation(angles);
        rotate = [rotate zeros(3, 1); zeros(1, 3) 1];

        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(model,'Matrix', move * rotate);
        
        % Update the drawing.      
        xmin = data.x(1,t)-5;
        xmax = data.x(1,t)+5;
        ymin = data.x(2,t)-5;
        ymax = data.x(2,t)+5;
        zmin = data.x(3,t)-1;
        zmax = data.x(3,t)+1;
        axis([xmin xmax ymin ymax zmin zmax]);
        drawnow;

       %Plot planned and actual path of copter
        plot3(xyz(1,1:t),xyz(2,1:t),xyz(3,1:t),'m-')
        for i=2:n
            plot3(xyz(3*i-2,1:t),xyz(3*i-1,1:t),xyz(3*i,1:t),'c-')
        end
    end
end

% Plot three components of a vector in RGB.
function multiplot(data, values, ind)
    % Select the parts of the data to plot.
    times = data.t(:, 1:ind);
    values = values(:, 1:ind);

    % Plot in RGB, with different markers for different components.
    plot(times, values(1, :), 'r-', times, values(2, :), 'g.', times, values(3, :), 'b-.');
    
    % Set axes to remain constant throughout plotting.
    xmin = min(data.t);
    xmax = max(data.t);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values));
    axis([xmin xmax ymin ymax]);
end

% Adapted from previous work for ease of visualization (Andrew Gibliani)
% Drawing a quadcopter
function [h, thrusts] = quadcopter()
    % Draw arms.
    h(1) = prism(-1, -0.1, -0.1, 2, 0.2, 0.2);
    h(2) = prism(-0.1, -1, -0.1, 0.2, 2, 0.2);

    % Draw bulbs representing propellers at the end of each arm.
    [x y z] = sphere;
    x = 0.2 * x;
    y = 0.2 * y;
    z = 0.09 * z;
    h(3) = surf(x - 1, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(4) = surf(x + 1, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - 1, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x, y + 1, z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Draw thrust cylinders.
    [x y z] = cylinder(0.05, 1);
    thrusts(1) = surf(x, y + 1, z, 'EdgeColor', 'none', 'FaceColor', 'w');
    thrusts(2) = surf(x + 1, y, z, 'EdgeColor', 'none', 'FaceColor', 'w');
    thrusts(3) = surf(x, y - 1, z, 'EdgeColor', 'none', 'FaceColor', 'w');
    thrusts(4) = surf(x - 1, y, z, 'EdgeColor', 'none', 'FaceColor', 'w');

    % Create handles for each of the thrust cylinders.
    for i = 1:4
        x = hgtransform;
        set(thrusts(i), 'Parent', x);
        thrusts(i) = x;
    end

    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism(x, y, z, w, l, h)
    [X Y Z] = prism_faces(x, y, z, w, l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X Y Z] = prism_faces(x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w];
    Y = [y y y+l y+l y y y+l y+l];
    Z = [z z+h z z+h z z+h z z+h];
end

%Compute rotation matrix
function R = rotation(angles)
    phi = angles(3);
    theta = angles(2);
    psi = angles(1);

    R = zeros(3);
    R(:, 1) = [
        cos(phi) * cos(theta)
        cos(theta) * sin(phi)
        - sin(theta)
    ];
    R(:, 2) = [
        cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi)
        cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)
        cos(theta) * sin(psi)
    ];
    R(:, 3) = [
        sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)
        cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi)
        cos(theta) * cos(psi)
    ];
end
