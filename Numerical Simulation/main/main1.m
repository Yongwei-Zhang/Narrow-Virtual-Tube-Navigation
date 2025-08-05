%% Program Description
% Under \mathbf{v}_i, this program is used to achieve:
% - Plot the position distribution of the robot swarm in the tube at the termination time;
% - Plot the curve of AMD (Average Minimum Distance) over time;
% - Plot the complete movement trajectory of the robots in the tube.

%% create tube
clc,clear
close all

% paras for creating generating curve
delta_x = 0.01;
slope1 = 0;
slope2 = 1;

% first segment of the generating curve
x1 = [0, 10, 20, 30];
y1 = [0, 1, -1, 0];
pp1 = spline(x1, [slope1, y1, slope1]);
coef1 = pp1.coefs;
xx1 = [];
for i = x1(1) : delta_x : x1(end)
    xx1(end+1) = i;
end
yy1 = ppval(pp1, xx1);

% third segment of the generating curve
x3 = [50, 60, 70];
y3 = [0, 1, 0];
pp3 = spline(x3, [slope1, y3, slope1]);
coef3 = pp3.coefs;
xx3 = [];
for i = x3(1)+delta_x : delta_x : x3(end)
    xx3(end+1) = i;
end
yy3 = ppval(pp3, xx3);

% middle(second) segment of the generating curve
xx2 = [];
for i = x1(end)+delta_x : delta_x : x3(1)
    xx2(end+1) = i;
end
yy2 = yy1(end)*ones(1,size(xx2,2));

% Splice three above curve segments
x = [xx1, xx2, xx3];
y = [yy1, yy2, yy3];
m = size(x,2);

% get the indices of the generating curve
ep1_lIndex = zeros(1,size(x1,2));
for i = 1:size(x1,2)
    ep1_lIndex(i) = 1 + x1(i)/delta_x;
end
ep3_lIndex = zeros(1,size(x3,2));
for i = 1:size(x3,2)
    ep3_lIndex(i) = 1 + x3(i)/delta_x;
end
 
% calculate derivatives of the generating curve
dert = zeros(1,m);
for i = 1:m
    if i <= ep1_lIndex(2)
        dert(i) = 3 * coef1(1,1) * (x(i)-x(ep1_lIndex(1)))^2 + 2 * coef1(1,2) * (x(i)-x(ep1_lIndex(1))) + coef1(1,3);
    elseif i > ep1_lIndex(2) && i <= ep1_lIndex(3)
        dert(i) = 3 * coef1(2,1) * (x(i)-x(ep1_lIndex(2)))^2 + 2 * coef1(2,2) * (x(i)-x(ep1_lIndex(2))) + coef1(2,3);
    elseif i > ep1_lIndex(3) && i <= ep1_lIndex(4)
        dert(i) = 3 * coef1(3,1) * (x(i)-x(ep1_lIndex(3)))^2 + 2 * coef1(3,2) * (x(i)-x(ep1_lIndex(3))) + coef1(3,3);
    elseif i > ep1_lIndex(4) && i <= ep3_lIndex(1)
        dert(i) = 0;
    elseif i > ep3_lIndex(1) && i <= ep3_lIndex(2)
        dert(i) = 3 * coef3(1,1) * (x(i)-x(ep3_lIndex(1)))^2 + 2 * coef3(1,2) * (x(i)-x(ep3_lIndex(1))) + coef3(1,3);
    else
        dert(i) = 3 * coef3(2,1) * (x(i)-x(ep3_lIndex(2)))^2 + 2 * coef3(2,2) * (x(i)-x(ep3_lIndex(2))) + coef3(2,3);
    end
end

% determine the tangent vector based on the derivatives
tangent = cell(1, m);
for i = 1:m
    tangent{i} = zeros(1,2); 
    tangent{i}(1) = 1/sqrt(1+dert(i)^2);
    tangent{i}(2) = dert(i)/sqrt(1+dert(i)^2);
end

% calculate the normal vectors orthogonal to the tangent vectors
normal = cell(1, m);
for i = 1:m
    normal{i} = zeros(1,2);
    normal{i}(1) = -dert(i)/sqrt(1+dert(i)^2);
    normal{i}(2) = 1/sqrt(1+dert(i)^2);
end

% get the arc length paras
l = zeros(1,m);
for i = 2:m
    l(i) = l(i-1) + sqrt((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2);
end

% calculate the arc lengths of endpoints in the 1st and 3rd segments
ep1_arcl = l(ep1_lIndex); % arcl=arclength
ep3_arcl = l(ep3_lIndex);

% set tube radius
radius = zeros(1,m);
for i = 1:m
    radius(i) = cal_radius(ep1_arcl, ep3_arcl, l(i));
end

% calculate derivatives of the radius
radius_dert = zeros(1,m);
for i = 1:m
    radius_dert(i) = cal_radius_dert(ep1_arcl, ep3_arcl, l(i));
end

% generate the upper and lower tubes based on the generating curve
tube_u_x = zeros(1,m);
tube_u_y = zeros(1,m);
tube_d_x = zeros(1,m);
tube_d_y = zeros(1,m);
for i = 1:m
    tube_u_x(i) = x(i) + radius(i)*normal{i}(1); 
    tube_u_y(i) = y(i) + radius(i)*normal{i}(2);
    tube_d_x(i) = x(i) + radius(i)*(-normal{i}(1)); 
    tube_d_y(i) = y(i) + radius(i)*(-normal{i}(2));
end

% save('TubeData.mat')

% plot the generating curve
% figure();
% plot(x, y, 'r--', 'LineWidth', 2)
% axis equal
% hold on;
% % grid on;
% font_size = 25;
% set(gca, 'FontSize', font_size)
% 
% % plot the upper and lower tubes
% plot(tube_u_x, tube_u_y, '-k', 'LineWidth', 2);
% plot(tube_d_x, tube_d_y, '-k', 'LineWidth', 2);
% % axis([x(1)-5, x(end)+5, min(tube_d_y)-10, max(tube_u_y)+10]);
% axis([x(1)-1, x(end)+1, min(tube_d_y)-1, max(tube_u_y)+1]);
% 
% % plot left and right boundaries
% plot(linspace(tube_d_x(1),tube_u_x(1),100),linspace(tube_d_y(1),tube_u_y(1),100), '-k', 'LineWidth', 2);
% plot(linspace(tube_d_x(end),tube_u_x(end),100),linspace(tube_d_y(end),tube_u_y(end),100), '-k', 'LineWidth', 2);
% hold off

%% set robots' initial positions
N = 25; 
N_m = N; % maximum robot number in the tube
position = cell(1,N); % save robots' positions
for i=1:N
    position{i} = zeros(1,2);
end

% set the position of the first robot
rs = 0.5; % safety radius
spacing = 0.01;
spacing_x = spacing*55 + rs*2;
spacing_y = spacing*35 + rs*2;
x_s = 2; % x-coordinate of the first robot
y_s = (coef1(1,1)*x_s^3 + coef1(1,2)*x_s^2 + coef1(1,3)*x_s + coef1(1,4)) - spacing_y*1.5; % y-coordinate
position{1}(1) = x_s;
position{1}(2) = y_s;

% other robots' positions
for i = 2:5
    position{i}(1) = position{1}(1);
    position{i}(2) = position{1}(2) + (i-1)*spacing_y;
end
for i = 6:10
    position{i}(1) = position{1}(1) + spacing_x;
    position{i}(2) = position{1}(2) + (i-6)*spacing_y;
end
for i = 11:15
    position{i}(1) = position{1}(1) + 2*spacing_x;
    position{i}(2) = position{1}(2) + (i-11)*spacing_y;
end
for i = 16:20
    position{i}(1) = position{1}(1) + 3*spacing_x;
    position{i}(2) = position{1}(2) + (i-16)*spacing_y;
end
for i = 21:25
    position{i}(1) = position{1}(1) + 4*spacing_x;
    position{i}(2) = position{1}(2) + (i-21)*spacing_y;
end

% % plot initial robots' postions
% for i = 1:N 
%     viscircles([position{i}(1) position{i}(2)], rs, 'Color', 'r', 'LineWidth', 1);
% end

% save initial robots' postions
position0 = {};
for i = 1:N
    position0{i} = position{i};
end

%% control for robot swarm
velocity = cell(1,N); % save robots' velocities
for i = 1:N
    velocity{i} = zeros(1,2);
end

% four control inputs
u1 = cell(1,N);
u2 = cell(1,N);
u3 = cell(1,N);
u4 = cell(1,N);
for i = 1:N
    u1{i} = zeros(1,2);
    u2{i} = zeros(1,2);
    u3{i} = zeros(1,2);
    u4{i} = zeros(1,2);
end

% set common control efficients
v_max = 3; % saturated control inputs
k1 = 2;
k2 = 1;
k3 = 1;
k4 = 4;
Time = 10; % total simulation time

rs1 = rs; % modified safety radius used in u3
ra = 0.7; % avoidance radius

% set control efficients for u2
d1 = 2*rs; 
d2 = ra + rs;
epsl_m = 1e-6; 
epsl_s = 1e-6;
x2 = 1+1/tan(67.5)*epsl_s; 
x1 = x2-sin(45)*epsl_s;

% set control efficients for u3
epsl_t = 1e-6; 

% set control efficients for u4
h = 2; % bandwidth
delta = 0.01; % time interval for calculating partial derivates in u4
itval = 100; % paras for searching for neighboring positions

% variables for saving data 
time_save = []; % save time instant
Distance1 = cell(1,N); % save minimum distance between robots at time instant t
min_distance_i_robot = zeros(1,N); 
AMD = [];
% Distance2 = zeros(1,N); % save the distance with tube boundary
leaved_num = []; % save robot numbers that exit the tube
exist_num = []; % save robot numbers that are still in the tube
count = 0; % count the number of exited robots
time_pass_save = []; % save time instant when exiting the tube

% save robots' positions and corresponding time, to plot trajectories
position_save_robot = cell(1,N);
time_save_robot = cell(1,N);
for i = 1:N
    position_save_robot{i} = {};
    time_save_robot{i} = [];
end

% variables for dealing with exited robots
position_save_robot_leave = {}; 
time_save_robot_leave = {};

% set simulation time 
delta_T = 0.01; % simulation interval
t = 0; % initial time

while t <= Time 
    all_lIndex = zeros(1,N); 
    for i = 1:N
        all_lIndex(i) = cal_lIndex(position{i},x,y,m);
    end
    min_lIndex = min(all_lIndex);
    max_lIndex = max(all_lIndex);

    % ρ_d(l) = r(l)/∫_{l_b}^{l_f}2*[r(l)]^2dl := r(l)/A
    % calculate A
    A = cal_rho_d_A(radius,min_lIndex,max_lIndex,l);

    if max_lIndex == m
        time_pass_save(end+1) = t;
    end

    for i = 1:N
        lIndex = all_lIndex(i); 

        % calculate u1
        u1{i}(1) = k1 * tangent{lIndex}(1);
        u1{i}(2) = k1 * tangent{lIndex}(2);

        % calculate u2
        Distance1{i} = []; 
        if N ~= 1
            sum1 = zeros(1,2);
            for j = 1:N
                if j ~= i 
                    d_ij = sqrt( (position{i}(1)-position{j}(1))^2 + (position{i}(2)-position{j}(2))^2 );
                    Distance1{i}(end+1) = d_ij; 
                    if d_ij <= ra+rs 
                        p_ij = [position{i}(1)-position{j}(1), position{i}(2)-position{j}(2)]; 
                        sum1 = sum1 + cal_u2(k2,d_ij,d1,d2,p_ij,epsl_m,rs,x1,x2,epsl_s);
                    end             
                end
            end
            u2{i} = sum1;
        else
            u2{i} = zeros(1,2);
        end

        % calculate u3
        rt = radius(lIndex);
        tube_m_x = (tube_u_x(lIndex) + tube_d_x(lIndex))/2;
        tube_m_y = (tube_u_y(lIndex) + tube_d_y(lIndex))/2;
        d_im = sqrt( (position{i}(1)-tube_m_x)^2 + (position{i}(2)-tube_m_y)^2 );
        d_it = rt - d_im;
        % Distance2(i) = d_it; 
        u3{i} = cal_u3(k3,d_it,d_im,tube_m_x,tube_m_y,position,i,epsl_t,rs1,x1,x2,epsl_s,ra);

        % calculate u4
        if N ~= 1 
            rho_d_grad = cal_rho_d_grad(A,radius_dert,lIndex,l,position{i},delta,itval,x,y); 
            [rho_est, rho_est_grad] = cal_rho_est_plus_grad(position, N, h, i);         
            u4{i} = cal_u4(k4, rho_est_grad, rho_d_grad, rho_est, u1{i}, u2{i}, u3{i}); 
        else
            u4{i} = zeros(1,2);
        end
       
        % calculate saturated velocity
        v_x = u1{i}(1)+u2{i}(1)+u3{i}(1)+u4{i}(1); 
        v_y = u1{i}(2)+u2{i}(2)+u3{i}(2)+u4{i}(2);
        saturated_velocity = cal_s_v(v_x, v_y, v_max);

        velocity{i}(1) = saturated_velocity(1);
        velocity{i}(2) = saturated_velocity(2);
    end
    
    % calculate AMD
    for i = 1:N
        min_distance_i_robot(i) = min(Distance1{i});
    end
    AMD(end+1) = mean(min_distance_i_robot);


    % save robots' positions and current time
    for i = 1:N
        position_save_robot{i}{end+1} = position{i};
        time_save_robot{i}(end+1) = t;
    end
    

    % deal with cases when robots achieve tube terminal
    if max_lIndex == m
        [max_lIndex, leaving_index] = max(all_lIndex); 
        if N == N_m 
            leaving_num = leaving_index;
            leaved_num(end+1) = leaving_num; 
            position_save_robot_leave{end+1} = position_save_robot{leaving_num};
            time_save_robot_leave{end+1} = time_save_robot{leaving_num};
            position_save_robot1 = {};
            time_save_robot1 = {};
            for i = 1:N
                if i ~= leaving_num
                    position_save_robot1{end+1} = position_save_robot{i}; 
                    time_save_robot1{end+1} = time_save_robot{i};
                end
            end
            position_save_robot = position_save_robot1;
            time_save_robot = time_save_robot1;
            position1 = {};
            velocity1 = {};
            for i = 1:N
                if i ~= leaving_num
                    exist_num(end+1) = i;
                    position1{end+1} = position{i};
                    velocity1{end+1} = velocity{i};
                end             
            end
            position = position1;
            velocity = velocity1;
        elseif N > 1 && N < N_m 
            position_save_robot_leave{end+1} = position_save_robot{leaving_index};
            time_save_robot_leave{end+1} = time_save_robot{leaving_index};
            position_save_robot1 = {};
            time_save_robot1 = {};
            for i = 1:N
                if i ~= leaving_index
                    position_save_robot1{end+1} = position_save_robot{i};
                    time_save_robot1{end+1} = time_save_robot{i};
                end
            end
            position_save_robot = position_save_robot1;
            time_save_robot = time_save_robot1;

            position1 = {};
            velocity1 = {};
            for i = 1:N
                if i ~= leaving_index
                    position1{end+1} = position{i};
                    velocity1{end+1} = velocity{i};
                else
                    leaved_num(end+1) = exist_num(i);
                    exist_num(i) = 0;
                    nonzeroIndex = (exist_num ~= 0);
                    exist_num = exist_num(nonzeroIndex);
                end
            end
            position = position1;
            velocity = velocity1;
        else
            leaved_num(end+1) = exist_num(1);
            count = count + 1;
            break
        end

        N = N - 1;
        u1 = cell(1,N);
        u2 = cell(1,N);
        u3 = cell(1,N);
        u4 = cell(1,N);
        for i = 1:N 
            u1{i} = zeros(1,2);
            u2{i} = zeros(1,2);
            u3{i} = zeros(1,2);
            u4{i} = zeros(1,2);
        end
        Distance1 = cell(1,N);
        min_distance_i_robot = zeros(1,N);
        count = count + 1;
    end
    
    % save simulation time
    time_save(end+1) = t; 

    % update robots' positions
    for i = 1:N
        position{i}(1) = position{i}(1) + velocity{i}(1)*delta_T;
        position{i}(2) = position{i}(2) + velocity{i}(2)*delta_T;
    end

    % update simulation time
    t = t + delta_T; 
end

%% plot robots' positions at the termination time
figure();
plot(x, y, 'r--', 'LineWidth', 2)
axis equal
hold on;
% grid on;
plot(tube_u_x, tube_u_y, '-k', 'LineWidth', 2);
plot(tube_d_x, tube_d_y, '-k', 'LineWidth', 2);
axis([x(1)-1, x(end)+1, min(tube_d_y)-1, max(tube_u_y)+1]);
plot(linspace(tube_d_x(1),tube_u_x(1),100),linspace(tube_d_y(1),tube_u_y(1),100), '-k', 'LineWidth', 2);
plot(linspace(tube_d_x(end),tube_u_x(end),100),linspace(tube_d_y(end),tube_u_y(end),100), '-k', 'LineWidth', 2);
font_size = 25;
xlabel('x, m', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('y, m', 'Interpreter', 'latex', 'FontSize', font_size);
for i = 1:N
    viscircles([position{i}(1) position{i}(2)], rs, 'Color', 'r', 'LineWidth', 2);
end
set(gca, 'FontSize', font_size);
text(53,5.5,['t = ', num2str(Time),'s,', ' N = ', num2str(N)], 'FontSize', 32, 'Color', 'r');
position_x = [];
position_y = [];
for i = 1:numel(position)
    position_x(end+1) = position{i}(1);
    position_y(end+1) = position{i}(2);
end
scatter(position_x, position_y, 20, 'r', 'filled');
velocity_x = [];
velocity_y = [];
for i = 1:numel(velocity)
    velocity_x(end+1) = velocity{i}(1);
    velocity_y(end+1) = velocity{i}(2);
end
scaleFactor = 0.5;
quiver(position_x, position_y, velocity_x*scaleFactor, velocity_y*scaleFactor, 'Color', 'b', 'LineWidth', 2, 'AutoScale', 'off');
hold off

%% plot AMD
figure()
plot(time_save, AMD, 'k-', 'LineWidth', 2);
hold on;
xlabel('time, s', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('Average Minimum Distance (AMD) of the robot swarm, m', 'Interpreter', 'latex', 'FontSize', font_size);
xlim([0, Time+1]);
grid on
hold off;

%% plot robots trajectories
for i = 1 : numel(position_save_robot_leave)
    position_save_robot{end+1} = position_save_robot_leave{i};
    time_save_robot{end+1} = time_save_robot_leave{i};
end
figure(); 
plot(x, y, 'r--', 'LineWidth', 2)
axis equal
hold on;
% plot([x1,x3], [y1,y3], 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
xlabel('x, m', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('y, m', 'Interpreter', 'latex', 'FontSize', font_size);
set(gca, 'FontSize', font_size)
grid on;
plot(tube_u_x, tube_u_y, '-k', 'LineWidth', 2);
plot(tube_d_x, tube_d_y, '-k', 'LineWidth', 2);
axis([x(1)-1, x(end)+1, min(tube_d_y)-1, max(tube_u_y)+1]);
plot(linspace(tube_d_x(1),tube_u_x(1),100),linspace(tube_d_y(1),tube_u_y(1),100), '-k', 'LineWidth', 2);
plot(linspace(tube_d_x(end),tube_u_x(end),100),linspace(tube_d_y(end),tube_u_y(end),100), '-k', 'LineWidth', 2);
text(53,5.5,['t = ', num2str(Time),'s,', ' N = ', num2str(N)], 'FontSize', 32, 'Color', 'r');
colors = { ...
    [0.7, 0.3, 0.3], ...   % 浅红色
    [0.3, 0.7, 0.3], ...   % 浅绿色
    [0.3, 0.3, 0.7], ...   % 浅蓝色
    [0.6, 0.6, 0.6], ...   % 浅灰色
    [0.8, 0.4, 0.4], ...   % 浅酒红
    [0.4, 0.8, 0.4], ...   % 浅墨绿
    [0.4, 0.4, 0.8], ...   % 浅海蓝
    [0.5, 0.7, 0.8], ...   % 浅青蓝
    [0.8, 0.6, 0.4], ...   % 浅橙色
    [0.7, 0.4, 0.7], ...   % 浅紫色
    [0.7, 0.7, 0.4], ...   % 浅橄榄绿
    [0.6, 0.8, 0.6], ...   % 浅绿色调
    [0.8, 0.4, 0.8], ...   % 浅品红
    [0.7, 0.6, 0.6], ...   % 浅红木色
    [0.7, 0.7, 0.4], ...   % 浅柠檬黄
    [0.5, 0.8, 0.8], ...   % 浅青色
    [0.5, 0.5, 0.8], ...   % 浅蓝紫
    [0.9, 0.6, 0.4], ...   % 浅橙黄
    [0.8, 0.6, 0.5], ...   % 浅棕色
    [0.7, 0.5, 0.7], ...   % 浅紫红
    [0.5, 0.6, 0.7], ...   % 浅水蓝
    [0.8, 0.7, 0.6], ...   % 浅杏色
    [0.5, 0.7, 0.5], ...   % 浅森林绿
    [0.6, 0.4, 0.4], ...   % 浅褐色
    [0.7, 0.5, 0.4]        % 浅咖啡色
};
for i = 1 : numel(position_save_robot)
    viscircles(position_save_robot{i}{1}, rs, 'Color', colors{i}, 'LineWidth', 2);
    scatter(position_save_robot{i}{1}(1), position_save_robot{i}{1}(2), 20, colors{i}, 'filled');
    path_x = [];
    path_y = [];
    for j = 1 : numel(position_save_robot{i})
        path_x(end+1) = position_save_robot{i}{j}(1);
        path_y(end+1) = position_save_robot{i}{j}(2);
    end
    plot(path_x, path_y,'Color', colors{i}, 'LineStyle', '-', 'LineWidth', 1);
    if position_save_robot{i}{end}(1) < x(end)
        viscircles(position_save_robot{i}{end}, rs, 'Color', colors{i}, 'LineWidth', 2);
        scatter(position_save_robot{i}{end}(1), position_save_robot{i}{end}(2), 20, colors{i}, 'filled');
    end
end
hold off