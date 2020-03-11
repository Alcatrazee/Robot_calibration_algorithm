function R = R_matrix(omega)
% calculate rotation matrix
omega = omega(:);
if norm(omega) < eps
    R = eye(3);
else 
    theta = norm(omega);
    omega = omega/theta;
    R = eye(3)*cos(theta) + hat(omega)*sin(theta) + omega*omega'*(1-cos(theta));
end