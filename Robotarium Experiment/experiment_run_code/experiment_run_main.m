clc,clear
close all
%% Single-integrator Barrier Certificate Algorithm
% by Sean Wilson 
% 07/2019

%% Set up Robotarium object
% Before starting the algorithm, we need to initialize the Robotarium
% object so that we can communicate with the agents
N = 10;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%This iis how many times the main loop will execute.
iterations = 5500;

%% Experiment constants
% Next, we set up some experiment constants

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

% N =  10
x_goal = [-0.75,0.4;-0.5,0.4;
    -1,0.1;-0.75,0.1;-0.5,0.1;
    -1,-0.2;-0.75,-0.2;-0.5,-0.2;
    -0.75,-0.5;-0.5,-0.5];
x_goal=x_goal';
x_goal(1,:) = x_goal(1,:) - 0.05;

% plot x_goal
% scatter(x_goal(1,:),x_goal(2,:),50,'green','filled');

flag = 0; %flag of task completion

%% Retrieve tools for single-integrator -> unicycle mapping

% Let's retrieve some of the tools we'll need.  We would like a
% single-integrator position controller, a single-integrator barrier
% function, and a mapping from single-integrator to unicycle dynamics

position_control = create_si_position_controller();
si_barrier_certificate = create_si_barrier_certificate_with_boundary();
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();

%% Begin the experiment
% This section contains the actual implementation of the barrier
% certificate experiment.

%Iterate for the previously specified number of iterations

load Tube_Data.mat

% plot the tube
plot(tube_m_x,tube_m_y,'k--','Linewidth',3);
plot(tube_u_x,tube_u_y,'r','Linewidth',3);
plot(tube_d_x,tube_d_y,'r','Linewidth',3);

%% Before the loop, set the parameters.
% control gain
v_max = 0.1;
k1 = 1/2*v_max; 
k2 = 1.8;
k3 = 2;
k4 = 2;

% robots' positions
position = cell(1,N);

% robots' velocities
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
velocity = cell(1,N);
for i = 1:N
    velocity{i} = zeros(1,2);
end

% parameter for robots
rs = 0.05;
ra = 0.12;
rs1 = rs; % modified rs
        
% parameters for u2
d1 = 2*rs; 
d2 = ra + rs; 
epsl_m = 1e-6; 
epsl_s = 1e-6; 
x2 = 1+1/tan(67.5)*epsl_s; 
x1 = x2-sin(45)*epsl_s; 

% parameters for u3
epsl_t = 1e-6; 

% parameters for u4
h = 0.15;
delta = 0.01;

% Store the minimum distance between robots
Distance1 = cell(1,N); 
min_distance_i_robot = zeros(1,N);
min_distance_robots = [];

% Store the distance between the robot and the tube boundary
Distance2 = zeros(1,N); 
min_distance_tube = []; 

% Store the positions of robots
position_save_robot = cell(1,N);
time_save_robot = cell(1,N);
for i = 1:N
    position_save_robot{i} = {};
    time_save_robot{i} = [];
end

% store time
t_count = 0;
time_save = [];

% other parameters
test_t_count = 0;

% color for plot robots' trajectories
colors = { ...
    [0.7, 0.3, 0.3], ...   
    [0.3, 0.7, 0.3], ...   
    [0.3, 0.3, 0.7], ...   
    [0.6, 0.6, 0.6], ...   
    [0.8, 0.4, 0.4], ...   
    [0.4, 0.8, 0.4], ...   
    [0.4, 0.4, 0.8], ...   
    [0.5, 0.7, 0.8], ...   
    [0.8, 0.6, 0.4], ...   
    [0.7, 0.4, 0.7], ...   
    [0.7, 0.7, 0.4], ...   
    [0.6, 0.8, 0.6], ...   
};

% plot x_goal
for i = 1:N
    scatter(x_goal(1,i),x_goal(2,i), 50, colors{i}, 'filled');
end

% count iterations
itr_count = 0;

for t = 1:iterations

    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();

    % Retrieve positions
    x_position = x(1:2,:);

    %% Algorithm

    % Let's make sure we're close enough the the goals
    if norm(x_goal-x_position,1)<0.03
        flag = 1-flag;
    end

    if flag == 0
        % Use a single-integrator position controller to drive the agents to
        % the circular formation
        dx = position_control(x(1:2, :), x_goal);

        %% Apply barrier certs. and map to unicycle dynamics
        %Ensure the robots don't collide
        dx = si_barrier_certificate(dx, x);

        % Transform the single-integrator dynamics to unicycle dynamics using a
        % diffeomorphism, which can be found in the utilities
        dx = si_to_uni_dyn(dx, x);

        %% Set the velocities of the agents
        % Set velocities of agents 1,...,N
        r.set_velocities(1:N, dx);
        
        test_t_count = test_t_count + 1;

    else % flag == 1, robots have reached x_goal        

        % Let position{i} store the robots' positions
        for i=1:N
            position{i} = x_position(:,i)';
        end

        % calculate lIndex
        all_lIndex = zeros(1,N); 
        for i = 1:N
            all_lIndex(i) = cal_lIndex(position{i}, tube_m_x, tube_m_y, m);
        end
        min_lIndex = min(all_lIndex); 
        max_lIndex = max(all_lIndex); 

        % calculate parameter for u4
        A = cal_rho_d_A(radius,min_lIndex,max_lIndex,l,all_lIndex,m,N);

        
        for i = 1:N

            % calculate lIndex for the i-th robot
            lIndex = all_lIndex(i);

            % calculate u1
            u1{i}(1) = k1 * tangent{lIndex}(1);
            u1{i}(2) = k1 * tangent{lIndex}(2);

            % calculate u2
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
            Distance2(i) = d_it;
            u3{i} = cal_u3(k3,d_it,d_im,mid_point_x,mid_point_y,position,i,epsl_t,rs1,x1,x2,epsl_s,ra);

            % modify u2 and u3
            u23_x = u2{i}(1)+u3{i}(1);
            u23_y = u2{i}(2)+u3{i}(2);
            sat_u23 = cal_s_v(u23_x, u23_y, 1.62*k1);
            
            % calculate u4
            rho_d_grad = cal_rho_d_grad(A,radius_dert,lIndex,l,position{i},delta,tube_m_x,tube_m_y,m);
            [rho_est, rho_est_grad] = cal_rho_est_plus_grad(position, N, h, i);
            u4{i} = cal_u4(k4, rho_est_grad, rho_d_grad, rho_est, u1{i}, sat_u23, k1);

            % calculate the sum velocity
            v_x = u1{i}(1)+u4{i}(1)+sat_u23(1); 
            v_y = u1{i}(2)+u4{i}(2)+sat_u23(2); 
            saturated_velocity = cal_s_v(v_x, v_y, v_max); 
    
            % calculate the saturated velocity
            velocity{i}(1) = saturated_velocity(1);
            velocity{i}(2) = saturated_velocity(2);

        end
        
        % store the minimum distances
        for i = 1:N
            min_distance_i_robot(i) = min(Distance1{i}); 
        end
        min_distance_robots(end+1) = min(min_distance_i_robot); 
        min_distance_tube(end+1) = min(Distance2);

        % store the robots' positions
        for i = 1:N
            position_save_robot{i}{end+1} = position{i};
            time_save_robot{i}(end+1) = t_count * 1/30;
        end

        % store time
        time_save(end+1) = t_count * 1/30;
        t_count = t_count + 1;

        % plot the robots' trajectories
        itr_count = itr_count + 1;
        if itr_count == 100 % plot the robots' trajectories for every 100 iterations
            for i = 1 : numel(position_save_robot)
                path_x = [];
                path_y = [];
                for j = 1 : numel(position_save_robot{i})
                    path_x(end+1) = position_save_robot{i}{j}(1);
                    path_y(end+1) = position_save_robot{i}{j}(2);
                end
                plot(path_x, path_y,'Color', colors{i}, 'LineStyle', '--', 'LineWidth', 3);
            end
            itr_count = 0;
        end
        

        %% Transform dynamics
        % Transform the single-integrator dynamics to unicycle dynamics using a
        % diffeomorphism, which can be found in the utilities
        VectorField = zeros(2, N);
        for i = 1:N
            VectorField(1,i) = velocity{i}(1);
            VectorField(2,i) = velocity{i}(2);
        end
        dx = si_barrier_certificate(VectorField, x);
        dx = si_to_uni_dyn(dx, x);

        %% Set the velocities of the agents
        % Set velocities of agents 1,...,N
        r.set_velocities(1:N, dx);

    end

    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end

% Print out any simulation problems that will produce implementation
%differences and potential submission rejection.
r.debug()
save('ExperimentData.mat','time_save','min_distance_robots','rs','min_distance_tube','position_save_robot','colors')

