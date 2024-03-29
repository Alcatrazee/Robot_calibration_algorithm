function norm_k = cali_2016()
%% statements
% This program is a 6 axis robot calibration program base on paper 
% 'POE-Based Robot Kinematic Calibration Using Axis Configuration Space and the Adjoint Error Model'
% Program simulate a process of robot calibration, we can choose wether
% using points or pose to calibrate a robot, this algorithm is of high
% efficiency,high accuracy,and can eliminate geometric error of a robot.

% Alternatable variables:
%  1.Change measurment type in line 14
%  2.Change Joint Error factor in line 15
%  3.Change End Effector error gain in line 16

%% preprocess the thread
% close all
% clear;
% clc
%% parameters definition

measurment_type = 1;    % 1:pose 2:points
joint_error_factor = 10;
EE_Error_gain = 10;

length_of_links = [0 1000 1000 400];                         % link length

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

% norminal twist_matrix_0; size = 6 x 6
twist_matrix_0 = [cross(q_vec_0,w_vec_0);w_vec_0];                             % nominal twist
twist_matrix = twist_matrix_0 + rand(6,6)/joint_error_factor;                  % actual twist
if(measurment_type==1)                                                         % different gst0 based on measuring types
    g_st0_a = T_matrix(log_my(g_st0)+rand(6,1)/joint_error_factor);
    gst_a = g_st0_a;
elseif measurment_type==2
    Pc_0_a = [g_st0(1:3,4)+rand(3,1)*EE_Error_gain ;1];
    Pc_0_n = [g_st0(1:3,4);1];
    gst_a = Pc_0_a;
end
twist_matrix(:,1:6) = twist_matrix(:,1:6) ./ vecnorm(twist_matrix(4:6,1:6));   % normalize omega

for i=1:6
    twist_matrix(1:3,i) = twist_matrix(1:3,i) - twist_matrix(1:3,i)'*twist_matrix(4:6,i)*twist_matrix(4:6,i);       % make v perpendicular to w
end

%define number of points,the more the better for calculation,but for sampling,the more,the worse.
num_of_pts = 10;

% define the maximum and minimum of joint angle.
max_angle_vec = deg2rad(ones(1,6)*130);
min_angle_vec = deg2rad(-ones(1,6)*130);
% define the random angle value for each random point
theta_random_vec(:,1:6) = (max_angle_vec - min_angle_vec) .* rand(num_of_pts,6) + min_angle_vec;

% variables declaration
if measurment_type == 1
    df_f_inv = zeros(num_of_pts*6,1);
    A = zeros(num_of_pts*6,42);
elseif measurment_type == 2
    df_f_inv = zeros(num_of_pts*3,1);
    A = zeros(num_of_pts*3,42);
end

B = zeros(42,4);
j=0;                                    % times of iteration
Q = zeros(6,42);                        
d_Pc = zeros(num_of_pts*3,1);
norm_k=[];                              % for norm of dp visialization

%% identification
while j<1000
    for i=1:num_of_pts                                                  % repeat num_of_pts times
        % FK
        [T_n,~,~] = FK(twist_matrix_0,theta_random_vec(i,:));           % Tn calculation
        [T_a,~,~] = FK(twist_matrix,theta_random_vec(i,:));             % Ta calculation
        
        Q = Q_matrix(twist_matrix_0,theta_random_vec(i,:));             % Q matrix calculation
        % choose different algorithm base on measurment type
        if(measurment_type==1)
            T_n = T_n*g_st0;                                            % calculate pose of end effector
            T_a = T_a*g_st0_a;
            df_f_inv(1+i*6-6:i*6) = log_my(T_a/T_n);                    % solve for log(df_f_inv)
            A(1+i*6-6:i*6,:) = Q;                                       % calculate [Q1|Q2|Q3|...Qn|Qst]
        elseif(measurment_type==2)
            Pc_a = T_a*Pc_0_a;                                          % position of actual end tip
            Pc_n = T_n*Pc_0_n;                                          % position of nominal end tip
            delta_pc = Pc_a - Pc_n;                                     % deviation 
            df_f_inv(i*3-2:i*3) = delta_pc(1:3);                        
            A(3*i-2:3*i,:) = [eye(3) -hat(Pc_n(1:3))]*Q;
        end
        
    end
    B = B_matrix(twist_matrix_0,measurment_type);                       % amazing matrix
    k = (A*B)\df_f_inv;
    norm(k)
    
    % composition of twist
    for i=1:6
        twist_matrix_0(:,i) = Adjoint(T_matrix(B(i*6-5:i*6,i*4-3:i*4)*k(i*4-3:i*4)))*twist_matrix_0(:,i);
    end
    if(measurment_type==1)
        g_st0 = T_matrix(k(25:30))*g_st0;
        twist_matrix_0(:,7) = log_my(g_st0);
        gst_n = g_st0;
    elseif(measurment_type==2)
        Pc_0_n = [k(25:27)+Pc_0_n(1:3);1];
        gst_n = Pc_0_n;
    end
    
    j=j+1;                                                                  % counter plus 1
    norm_k = [norm_k norm(k)];                                              % minimization target value calculation
    disp (norm(k))                                                          % show value of norm of dp
    %disp (j)                                                               % show number of iteration
    if norm(k) < 10e-6                                                      % judge if it's able to quit the iteration
        break;
    elseif (norm(k)>1e4)||j>100
        break;
    end
    %% plot
    clf;
    draw_manipulator(twist_matrix,gst_a,'r',measurment_type);               % draw robot frame with actual twist
    draw_manipulator(twist_matrix_0,gst_n,'b',measurment_type);             % draw robot frame with norminal twist
    drawnow;
end
%% plot again
% fig2 = figure(2);                                                           % open another window
% bar(norm_k)                                                                 % plot discrete figure

%% validation
% validate the result using 100 configurations
number_of_validation_samples = 100;
random_joint_angles = (max_angle_vec - min_angle_vec) .* rand(number_of_validation_samples,6) + min_angle_vec;
if measurment_type==1
    orientation_error = zeros(1,100);
    position_error = zeros(1,100);
    for i = 1:number_of_validation_samples
       [T_n,~,~] = FK(twist_matrix_0,random_joint_angles(i,:));           % Tn calculation
       [T_a,~,~] = FK(twist_matrix,random_joint_angles(i,:));             % Ta calculation
       
       T_n = T_n*g_st0;
       T_a = T_a*g_st0_a;
       
       deviation = log_my(T_a/T_n);
       orientation_error(i) = norm(deviation);
       position_error(i) = norm(T_a(1:3,4) - T_n(1:3,4));
    end
    mean_orientation_error = mean(abs(orientation_error))
    max_orientation_error = max(abs(orientation_error))
    mean_position_error = mean(position_error)
    max_position_error = max(position_error)
elseif measurment_type==2
    position_error = zeros(1,100);
    for i = 1:number_of_validation_samples
       [T_n,~,~] = FK(twist_matrix_0,random_joint_angles(i,:));           % Tn calculation
       [T_a,~,~] = FK(twist_matrix,random_joint_angles(i,:));             % Ta calculation
       
       Pc_a = T_a*Pc_0_a;
       Pc_n = T_n*Pc_0_n;
       
       deviation = Pc_a - Pc_n;
       position_error(i) = norm(deviation(1:3));
    end
    mean_position_error = mean(position_error)
    max_position_error = max(position_error)
end

end



