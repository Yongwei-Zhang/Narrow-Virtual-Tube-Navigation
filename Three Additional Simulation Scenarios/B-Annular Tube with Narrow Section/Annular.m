%% 程序说明
% 本实验用于绘制环形管道（Annular Tube）的穿行
% 环形管道的数据可以直接通过Robotarium中给出的Tube_Data.mat获取
% 也可以直接通过原来的创建环形管道的代码获得
% 该实验的仿真部分主要借助Robotarium的中的代码，变量保持一致

%% 环形管道的原代码
clc,clear
% close all
m = 1001;
theta = linspace(0, 2*pi, m); % 生成(m-1)个从0到2*pi的等间距点，用于描述椭圆周
center_x = 0;
center_y = 0;
a = 10;
b = 8;
x = center_x - a*cos(theta); % 椭圆的x坐标，圆心在(0,0)，长半轴为10
y = center_y + b*sin(theta); % 椭圆的y坐标，短半轴为5

% 计算椭圆上每一点对应的弧长
l = zeros(1,m); % 生成线的弧长
for i = 2:m
    l(i) = l(i-1) + norm([x(i)-x(i-1),y(i)-y(i-1)]);
end

% 根据周长分配半径
% 最大半径：A+B，最小半径：B
% 这里设置管道最短的半径为0.8，至多只能容纳1个robot(r_s=0.5)
B = 0.8; % cos最下端与x轴的距离
A = 5 - B; % cos这个图像总的幅度
T = l(m);
w = 2*pi/T;
radius = A/2*cos(w*l) + (A/2+B);

% 计算radius_dert(半径关于弧长参数l的梯度)，在计算ρ_d的梯度的时候需要使用
radius_dert = -A/2 * sin(w*l) * w;

% % 绘制半径（及其导数）随着弧长参数变化的情况
% figure()
% plot(l, radius, 'LineWidth', 2)
% hold on 
% plot(l, radius_dert, 'LineWidth', 2)
% grid on

% 计算生成线上每个点对应的法向量
% 对于平面曲线方程F(x,y)=0,其法向量为：(F_x, F_y)(数分教材P170)
grad_x = (x - center_x) / a^2; % 椭圆方程对x的偏导
grad_y = (y - center_y) / b^2; % 椭圆方程对y的偏导
% 归一化法向量
norm_factor = sqrt(grad_x.^2 + grad_y.^2); % 法向量模长
normal_x = grad_x ./ norm_factor; % 法向量x分量
normal_y = grad_y ./ norm_factor; % 法向量y分量

% 画出所有法向量对应的切向量
% 对于椭圆的参数方程，且切向量为：(x'(theta),y'(theta))=(a*sin(theta),b*cos(theta))
tangent_x = a * sin(theta); % x方向切向量分量
tangent_y = b * cos(theta); % y方向切向量分量
% 归一化切向量
norm_factor = sqrt(tangent_x.^2 + tangent_y.^2); % 切向量模长
tangent_x = tangent_x ./ norm_factor; % 归一化x分量
tangent_y = tangent_y ./ norm_factor; % 归一化y分量

% 切向量存储
tangent = cell(1, m); % 生成一个长度为m的单元数组来存储生成线上每一点的切向量
for i = 1:m
    tangent{i} = zeros(1,2); % tangent是单元数组，每一个元素是一个二维的向量
    tangent{i}(1) = tangent_x(i);
    tangent{i}(2) = tangent_y(i);
end

% 法向量存储
normal = cell(1, m);
for i = 1:m
    normal{i} = zeros(1,2);
    normal{i}(1) = normal_x(i);
    normal{i}(2) = normal_y(i);
end

% 给出上下管道的边界
for i = 1:m
    terminal_u = [x(i),y(i)] + radius(i)*[normal_x(i),normal_y(i)];
    tube_u_x(i) = terminal_u(1);
    tube_u_y(i) = terminal_u(2);
    terminal_d = [x(i),y(i)] - radius(i)*[normal_x(i),normal_y(i)];
    tube_d_x(i) = terminal_d(1);
    tube_d_y(i) = terminal_d(2);
end

% % 绘制椭圆和法向量
% figure;
% plot(x, y, 'r-', 'LineWidth', 2); % 椭圆
% hold on;
% % quiver(x, y, normal_x, normal_y, 0.5, 'r'); % 法向量
% % quiver(x, y, tangent_x, tangent_y, 0.5, 'b'); % 切向量
% axis equal;
% % title('椭圆及其法向量');
% xlabel('x');
% ylabel('y');
% grid on;
% % 绘制管道的上下边界
% plot(tube_u_x,tube_u_y,'k','Linewidth',2);
% plot(tube_d_x,tube_d_y,'k','Linewidth',2);

% 存储数据待使用
% save('Tube_Data.mat', 'x', 'y', 'tube_u_x', 'tube_u_y', 'tube_d_x', 'tube_d_y', 'm', ...
%     'tangent', 'normal', 'radius', 'l', 'radius_dert')

% 将所有数据都存储起来
% save('Tube_Data.mat')

% 保持和Robotarium中的变量一致
tube_m_x = x;
tube_m_y = y;

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
x_s = -10 - 2*spacing_x; % x-coordinate of the first robot
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

%% 开始对Robot控制
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
k4 = 4;
close all
u4_coef = 0.85; % 控制u4的上界，即控制u4的实际大小
Time = 2; % total simulation time

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

% variables for saving data 
% 计算Robot之间的距离，是为了计算AMD
time_save = []; % save time instant
Distance1 = cell(1,N); % save minimum distance between robots at time instant t
min_distance_i_robot = zeros(1,N); 
AMD = [];

% Store the positions of robots
position_save_robot = cell(1,N);
time_save_robot = cell(1,N);
for i = 1:N
    position_save_robot{i} = {};
    time_save_robot{i} = [];
end

% 在环形的管道中，就不设置离开管道的Robot的数量了，因为一直在管道内
% leaved_num = []; % save robot numbers that exit the tube
% exist_num = []; % save robot numbers that are still in the tube
% count = 0; % count the number of exited robots
% time_pass_save = []; % save time instant when exiting the tube

% set simulation time 
delta_T = 0.01; % simulation interval
t = 0; % initial time

% 开始仿真
% 其中调用的函数，后面都加上1，表示这是对于环形管道的函数
while t <= Time

    all_lIndex = zeros(1,N); 
    for i = 1:N
        all_lIndex(i) = cal_lIndex(position{i}, tube_m_x, tube_m_y, m);
    end
    min_lIndex = min(all_lIndex);
    max_lIndex = max(all_lIndex);

    % ρ_d(l) = r(l)/∫_{l_b}^{l_f}2*[r(l)]^2dl := r(l)/A
    % calculate A
    A = cal_rho_d_A(radius,min_lIndex,max_lIndex,l,all_lIndex,m,N);

    for i=1:N

        % calculate lIndex for the i-th robot
        lIndex = all_lIndex(i);

        % calculate u1
        u1{i}(1) = k1 * tangent{lIndex}(1);
        u1{i}(2) = k1 * tangent{lIndex}(2);

        % calculate u2
        % 在环形管道内，不需要像原带代码那样，考虑是否有Robot离开管道
        Distance1{i} = [];
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

        % calculate u3
        rt = radius(lIndex);
        mid_point_x = (tube_u_x(lIndex) + tube_d_x(lIndex))/2;
        mid_point_y = (tube_u_y(lIndex) + tube_d_y(lIndex))/2;
        d_im = sqrt( (position{i}(1)-mid_point_x)^2 + (position{i}(2)-mid_point_y)^2 );
        d_it = rt - d_im;
        % Distance2(i) = d_it;
        u3{i} = cal_u3(k3,d_it,d_im,mid_point_x,mid_point_y,position,i,epsl_t,rs1,x1,x2,epsl_s,ra);

        % 处理u23=u2+u3
        u23 = u2{i} + u3{i};

        % calculate u4
        % 这个要参考Robotarium实验的代码了
        if k4 ~= 0
            rho_d_grad = cal_rho_d_grad(A,radius_dert,lIndex,l,position{i},delta,tube_m_x,tube_m_y,m);
            [rho_est, rho_est_grad] = cal_rho_est_plus_grad(position, N, h, i);
            u4{i} = cal_u4(k4, rho_est_grad, rho_d_grad, rho_est, u1{i}, u23, u4_coef);
        else
            u4{i} = zeros(1,2);
        end

        % calculate the sum velocity
        v_x = u1{i}(1)+u4{i}(1)+u23(1); 
        v_y = u1{i}(2)+u4{i}(2)+u23(2); 
        saturated_velocity = cal_s_v(v_x, v_y, v_max); 

        % calculate the saturated velocity
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
% axis([x(1)-1, x(end)+1, min(tube_d_y)-1, max(tube_u_y)+1]);
% plot(linspace(tube_d_x(1),tube_u_x(1),100),linspace(tube_d_y(1),tube_u_y(1),100), '-k', 'LineWidth', 2);
% plot(linspace(tube_d_x(end),tube_u_x(end),100),linspace(tube_d_y(end),tube_u_y(end),100), '-k', 'LineWidth', 2);
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
% 添加时间说明
text(7.5, 9, ['t = ', num2str(30),'s'], 'FontSize', 32, 'Color', 'r');
% text(6, -9.5, ['coef = ', num2str(u4_coef)], 'FontSize', 32, 'Color', 'r');
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
