%% close all unecessary windows and console and clear variables
close all
clear;
clc
%% parameters definition

measurment_type = 2;    % 1:pose 2:points
joint_error_factor = 50;
EE_Error_gain = 1;

% length of links
length_of_links = [0 1000 2000 400];

% q(point on the rotation axis) vector without offset
q_vec_0 = [ 0 0 length_of_links(1);
    0 0 length_of_links(1);
    0 0 length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2)]';

% omega matrix(axis of rotation)
w_vec_0 = [ 0 0 1;
    1 0 0;
    1 0 0;
    0 1 0;
    1 0 0;
    0 1 0]';

%% norminal twist_matrix_0; size = 6 x 7              
twist_matrix_n = [cross(q_vec_0,w_vec_0);w_vec_0];                          % nominal twist
twist_matrix_copy = twist_matrix_n;                                         % copy of original nominal twist
twist_matrix_0 = twist_matrix_n + rand(6,6)/joint_error_factor;             % actual twist
P_c0_n = [0 2400 1000 1]';                                                  % nominal position of end tip
P_c0_a = [P_c0_n(1:3)+rand(3,1)*EE_Error_gain;1];
gst_a = P_c0_a;

%% calculate normalized omega
vec_norm = vecnorm(twist_matrix_0(4:6,1:6));                                % normalize rotational axis vectors
for i = 1:6
    twist_matrix_0(:,i) = twist_matrix_0(:,i)./vec_norm(i);
end

%% calculate v again(by def,v is always perpendicular to omega)
for i=1:6
    twist_matrix_0(1:3,i) = twist_matrix_0(1:3,i) - twist_matrix_0(1:3,i)'*twist_matrix_0(4:6,i)*twist_matrix_0(4:6,i);       % make v perpendicular to w
end

%% define number of points,the more the better for calculation,but for sampling,the more the worse.
num_of_pts = 50;

%% the last joint angle shall be set to 0
theta_random_vec = zeros(num_of_pts,6);

%% define the maximum and minimum of joint angle.
max_angle_vec = deg2rad(ones(1,6)*130);
min_angle_vec = deg2rad(-ones(1,6)*130);

%% define the random angle value for each random point
theta_random_vec(:,1:6) = (max_angle_vec - min_angle_vec) .* rand(num_of_pts,6) + min_angle_vec;

%% variables declaration
j = 0;                                              % number of iteration
eta_matrix = zeros(6,6);
d_eta = zeros(39,1);
dPc = zeros(3*num_of_pts,1);
A_tilde = zeros(3*num_of_pts,39);
norm_d_eta = [];
%% parameters identification and composition
while j<1000
    %% calculate A matrix and df*f^-1 (parameters identification)
    for i=1:num_of_pts                                                      % repeat num_of_pts times
        [Tn,~,~] = FK(twist_matrix_n,theta_random_vec(i,:));                % nominal transformation matrix
        [Ta,~,~] = FK(twist_matrix_0,theta_random_vec(i,:));                % actual transfomation matrix
        P_n = Tn * P_c0_n;                                                  % nominal position of end point
        P_a = Ta * P_c0_a;                                                  % actual position of end point, simulation of measuring result
        dpc = P_a - P_n;                                                    % deviation of position
        dPc(i*3-2:i*3) = dpc(1:3);
        A_tilde(i*3-2:i*3,:) = A_tilde_matrix(twist_matrix_n,eta_matrix,P_n,theta_random_vec(i,:));
    end
    d_eta = A_tilde\dPc;
    
    %% composition
    for i=1:6
        eta_matrix(:,i) = eta_matrix(:,i)+d_eta(i*6-5:i*6);                 % composition of eta matrix
        twist_matrix_n(:,i) = Adjoint(T_matrix(eta_matrix(:,i)))*twist_matrix_copy(:,i);    % composition of twists
    end
    P_c0_n = [P_c0_n(1:3) + d_eta(37:39);1];                                % composition of end tip position
    gst_n = P_c0_n;                                                         % for visualization
    
    %% data prepration of visialization
    j=j+1;                                                                  % counter plus 1
    %norm_dp = [norm_dp norm(dp)];                                          % minimization target value calculation
    norm_d_eta = [norm_d_eta norm(d_eta)];
    disp (norm(d_eta))                                                      % show value of norm of dp
    disp (j)                                                                % show number of iteration
    if norm(d_eta) < 1e-4                                                   % quit the for loop if deviation is less than 1e-5
        break;
    end
    %% plot
    clf;                                                                    % clear plot
    draw_manipulator(twist_matrix_0,gst_a,'r',measurment_type);             % draw robot frame with actual twist
    draw_manipulator(twist_matrix_0,gst_n,'b',measurment_type);             % draw nominal axis
    drawnow;
end
%% plot again
fig2 = figure(2);                                                           % create another window
stem(norm_d_eta)                                                            % plot discrete data
