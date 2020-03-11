function T = T_matrix(twist)
% mapping from se(3) to SE(3)
% twist is a vector representing a motion parameter
% value is the radian that rotate or length that moves
% return exponential of a twist 

omega = twist(4:6);
omega = omega(:);
v = twist(1:3);
v = v(:);

if norm(omega) < eps
    T = [eye(3),v;0 0 0 1];
else 
    theta = norm(omega);
    omega = omega/theta;
    v = v/theta;
    R = eye(3)*cos(theta) + hat(omega)*sin(theta) + omega*omega'*(1-cos(theta));
    p = (eye(3) - R)*hat(omega)*v + omega*omega'*v*theta;
    T = [R p; 0 0 0 1];
end
end