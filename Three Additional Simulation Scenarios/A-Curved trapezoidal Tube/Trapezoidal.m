%% 程序说明
% 本实验用于绘制梯形管道（Trapezoidal Tube）的穿行

%% 创建梯形管道的变形版本——喇叭形管道
clc,clear,close all

% 生成线，还是按照原来的3段式比较方便
% 3 段对应的长度
l_len = 20;
m_len = 2;
r_len = 20;
delta_x = 0.01;

% first segment of the generating curve
x1 = [-l_len-m_len/2, -m_len/2];
y1 = [0, 0];
xx1 = [];
for i = x1(1) : delta_x : x1(end)-delta_x
    xx1(end+1) = i;
end
yy1 = zeros(1,length(xx1));

% third segment of the generating curve
x3 = [m_len/2, m_len/2+r_len];
y3 = [0, 0];
xx3 = [];
for i = x3(1) : delta_x : x3(end)
    xx3(end+1) = i;
end
yy3 = zeros(1,length(xx3));

% middle(second) segment of the generating curve
xx2 = [];
for i = x1(end) : delta_x : x3(1)-delta_x
    xx2(end+1) = i;
end
yy2 = zeros(1,length(xx2));

% Splice three above curve segments
x = [xx1, xx2, xx3];
y = [yy1, yy2, yy3];
m = size(x,2);

% 定义生成线的导数（关于弧长参数l）
dert = zeros(1,m);

% 基于导数确定生成线上每点的切向量
tangent = cell(1, m);
for i = 1:m
    tangent{i} = zeros(1,2); 
    tangent{i}(1) = 1/sqrt(1+dert(i)^2);
    tangent{i}(2) = dert(i)/sqrt(1+dert(i)^2);
end

% 根据切向量计算生成线上的法向量
normal = cell(1, m);
for i = 1:m
    normal{i} = zeros(1,2);
    normal{i}(1) = -dert(i)/sqrt(1+dert(i)^2);
    normal{i}(2) = 1/sqrt(1+dert(i)^2);
end

% 弧长参数
l = zeros(1,m);
for i = 2:m
    l(i) = l(i-1) + sqrt((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2);
end

% 获取第1段和第3段生成线的索引
ep1_lIndex = zeros(1,size(xx1,2));
for i = 1:size(xx1,2)
    ep1_lIndex(i) = 1 + (xx1(i)-x1(1))/delta_x;
end
ep3_lIndex = zeros(1,size(xx3,2));
for i = 1:size(xx3,2)
    ep3_lIndex(i) = 1 + (xx3(i)-x1(1))/delta_x;
end

% 设置每一段的半径
B = 0.8; % cos最下端与x轴的距离
A = 8 - B; % cos这个图像总的幅度
T = 2*l_len;
w = 2*pi/T;
radius = zeros(1,m);
radius_dert = zeros(1,m);
for i = 1:m
    if i <= ep1_lIndex(end)
        radius(i) = A/2*cos(w*l(i)) + (A/2+B);
        radius_dert(i) = -A/2 * sin(w*l(i)) * w;
    elseif i >= ep3_lIndex(1)
        radius(i) = A/2*cos(w*(l(i)-T/2-l(ep3_lIndex(1)))) + (A/2+B);
        radius_dert(i) = -A/2 * sin(w*(l(i)-T/2-l(ep3_lIndex(1)))) * w;
    else
        radius(i) = B;
        radius_dert(i) = 0;
    end
end

% 绘制半径（及其导数）随着弧长参数变化的情况
% figure()
% plot(l, radius, 'LineWidth', 2)
% hold on 
% plot(l, radius_dert, 'LineWidth', 2)
% grid on

% 获取管道的上下边界
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

% % 绘制整个管道
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
% % hold off

%% 初始化Robot的位置
N = 25; 
N_m = N; % maximum robot number in the tube
position = cell(1,N); % save robots' positions
for i=1:N
    position{i} = zeros(1,2);
end

% set the position of the first robot
rs = 0.5; % safety radius
spacing = 0.01;
spacing_x = spacing*35 + rs*2;
spacing_y = spacing*35 + rs*2;
% 确定第1个robot的位置(方形队伍中左下角的位置)
x_s = -l_len -m_len/2 + spacing_x; % x-coordinate of the first robot
y_s = 0 - 2*spacing_y; % y-coordinate
position{1}(1) = x_s;
position{1}(2) = y_s;

% 先从第1个位置向上，然后向下拓展
for i = 2:5
    position{i}(1) = position{1}(1);
    position{i}(2) = position{1}(2) + (i-1)*spacing_y;
end
% 从最下面开始数，第1行
for i = 6:9
    position{i}(1) = position{1}(1) + (i-5)*spacing_x;
    position{i}(2) = position{1}(2);
end
% 第2行，后面就是第3行、第4行、第5行
for i = 10:13
    position{i}(1) = position{2}(1) + (i-9)*spacing_x;
    position{i}(2) = position{2}(2);
end
for i = 14:17
    position{i}(1) = position{3}(1) + (i-13)*spacing_x;
    position{i}(2) = position{3}(2);
end

for i = 18:21
    position{i}(1) = position{4}(1) + (i-17)*spacing_x;
    position{i}(2) = position{4}(2);
end
for i = 22:25
    position{i}(1) = position{5}(1) + (i-21)*spacing_x;
    position{i}(2) = position{5}(2);
end

% for i = 1:N 
%     viscircles([position{i}(1) position{i}(2)], rs, 'Color', 'r', 'LineWidth', 1);
% end

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
% k4 = 4;
k4 = 0;
Time = 30; % total simulation time

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
        if k4 ~= 0
            if N ~= 1 
                rho_d_grad = cal_rho_d_grad(A,radius_dert,lIndex,l,position{i},delta,itval,x,y); 
                [rho_est, rho_est_grad] = cal_rho_est_plus_grad(position, N, h, i);         
                u4{i} = cal_u4(k4, rho_est_grad, rho_d_grad, rho_est, u1{i}, u2{i}, u3{i}); 
            else
                u4{i} = zeros(1,2);
            end
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
text(-5, 5, ['t = ', num2str(Time),'s,', ' N = ', num2str(N)], 'FontSize', 32, 'Color', 'r');
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
ylabel('AMD of the robot swarm, m', 'Interpreter', 'latex', 'FontSize', font_size);
set(gca, 'FontSize', font_size);
xlim([0, Time+1]);
grid on
hold off;

%% 打印额外的信息
disp("当前时刻：" + Time);
disp("Robot的数量：" + N);

