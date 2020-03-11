%% preprocess the thread
% close all
% clear;
% clc
%% parameters definition

%define number of points,the more the better for calculation,but for sampling,the more,the worse.
num_of_pts = 50;
error_factor = 10;

length_of_links = [0 2 3 0.5];                              % link length

q_vec_0 = [ 0 0 length_of_links(1);                          % q vector with no offset
    0 0 length_of_links(1);
    0 0 length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2)]';

w_vec_0 = [ 0 0 1;                                           % omega matrix
    1 0 0;
    1 0 0;
    0 1 0;
    1 0 0;
    0 1 0]';

g_st0 = [eye(3) [0 length_of_links(3)+length_of_links(4) length_of_links(1)+length_of_links(2)]';% g_st0
    0 0 0 1];

% norminal twist_matrix_0; size = 6 x 7
twist_matrix_0 = [[cross(q_vec_0,w_vec_0);w_vec_0],log_my(g_st0)];                              % nominal twist
twist_matrix = twist_matrix_0 + rand(6,7)/error_factor;                                                   % actual twist
twist_matrix(:,1:6) = twist_matrix(:,1:6) ./ vecnorm(twist_matrix(4:6,1:6));                 % normalize omega

for i=1:6
    twist_matrix(1:3,i) = twist_matrix(1:3,i) - twist_matrix(1:3,i)'*twist_matrix(4:6,i)*twist_matrix(4:6,i);       % make v perpendicular to w
end

gst_a = FK_new(twist_matrix,[0 0 0 0 0 0 1]);
gst_n = FK_new(twist_matrix_0,[0 0 0 0 0 0 1]);

% the last joint angle shall be set to 1 in order to formalize a
% exponential form
theta_random_vec = zeros(num_of_pts,7);

% define the maximum and minimum of joint angle.
max_angle_vec = deg2rad(ones(1,6)*130);
min_angle_vec = deg2rad(-ones(1,6)*130);
% define the random angle value for each random point
theta_random_vec(:,1:6) = (max_angle_vec - min_angle_vec) .* rand(num_of_pts,6) + min_angle_vec;
theta_random_vec(:,7) = ones(num_of_pts,1);

% variables declaration
df_f_inv = zeros(num_of_pts*6,1);
dp = zeros(num_of_pts*6,1);
A = zeros(num_of_pts*6,42);
j=0;

norm_dp=[]; % for norm of dp visialization
%% identification
while j<1000
    for i=1:num_of_pts                                                      % repeat num_of_pts times
        [T_n,~,~] = FK_new(twist_matrix_0,theta_random_vec(i,:));           % Tn calculation
        [T_a,~,~] = FK_new(twist_matrix,theta_random_vec(i,:));             % Ta calculation
        A(1+i*6-6:i*6,:) = A_matrix(twist_matrix_0,theta_random_vec(i,:));  % A matrix calculation
        df_f_inv(1+i*6-6:i*6) = log_my(T_a/T_n);                            % solve for log(df_f_inv)
    end
    dp = A\df_f_inv;                                                        % solve for dp(derive of twist)
    % composition of twist
    for i=1:6
        twist_matrix_0(:,i) = twist_matrix_0(:,i) + dp(1+i*6-6:i*6,1);                                                          % composition
        twist_matrix_0(:,i) = twist_matrix_0(:,i)/norm(twist_matrix_0(4:6,i));                                                  % normalization
        twist_matrix_0(1:3,i) = twist_matrix_0(1:3,i) - twist_matrix_0(1:3,i)'*twist_matrix_0(4:6,i)*twist_matrix_0(4:6,i);     % make v perpendicular to w
    end
    twist_matrix_0(:,7) = twist_matrix_0(:,7)+dp(37:42,1);                          % alternate the last rows
    
    j=j+1;                                                                  % counter plus 1
    norm_dp = [norm_dp norm(dp)];                                           % minimization target value calculation
    disp (norm(dp))                                                         % show value of norm of dp
    %disp (j)                                                                % show number of iteration
    if norm(dp) < 10e-6                                                      % judge if it's able to quit the iteration
        converge_flag = 1;
        break;
    elseif (norm(dp)>1e4)||j>100
        converge_flag = 0;
        break;
    end
    %% plot
    clf;
    draw_manipulator(twist_matrix,gst_a,'r',1);                                     % draw robot frame with actual twist
    draw_manipulator(twist_matrix_0,gst_n,'b',1);                                   % draw robot frame with norminal twist
    drawnow;
end
%% plot again
fig2 = figure(2);                                                           % open another window
bar(norm_dp)                                                               % plot discrete figure
%% validation
% validate the result using 100 configurations
number_of_validation_samples = 100;
random_joint_angles = (max_angle_vec - min_angle_vec) .* rand(number_of_validation_samples,6) + min_angle_vec;
orientation_error = zeros(1,100);
position_error = zeros(1,100);
for i = 1:number_of_validation_samples
    [T_n,~,~] = FK(twist_matrix_0,random_joint_angles(i,:));           % Tn calculation
    [T_a,~,~] = FK(twist_matrix,random_joint_angles(i,:));             % Ta calculation
    
    T_n = T_n*gst_a;
    T_a = T_a*gst_n;
    
    deviation = log_my(T_a/T_n);
    orientation_error(i) = norm(deviation);
    position_error(i) = norm(T_a(1:3,4) - T_n(1:3,4));
end
mean_orientation_error = mean(abs(orientation_error))
max_orientation_error = max(abs(orientation_error))
mean_position_error = mean(position_error)
max_position_error = max(position_error)


