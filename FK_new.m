function [T,T_link_incr,T_link_abs] = FK_new( twist_matrix,theta )

% forward kinematic

% link transformation incremental, i.e., T1, T2, T3, ....
T_link_incr = zeros(4,4,7);
% link transformation absolute, i.e., T1, T1*T2, T1*T2*T3, ...
T_link_abs = zeros(4,4,7);
for i = 1:7
    T_link_incr(:,:,i) = T_matrix(twist_matrix(:,i)*theta(i));      % get transformation matrix
    T_link_abs(:,:,i) = eye(4);
    for j = 1:i
        T_link_abs(:,:,i) = T_link_abs(:,:,i) * T_link_incr(:,:,j);
    end
end
    T = T_link_abs(:,:,7);
end