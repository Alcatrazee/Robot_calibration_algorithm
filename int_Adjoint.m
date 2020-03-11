function  intAd = int_Adjoint( twist )
% integration of adjoint matrix
intAd = zeros(6,6);
omega = twist(4:6);
omega = omega(:);
v = twist(1:3);
v = v(:);

if norm(omega) < eps
    intAd(1:3,1:3) = eye(3);
    intAd(4:6,4:6) = eye(3);
    intAd(1:3,4:6) = hat(v/2);
else
    theta = norm(omega);
    v = v/theta;
    omega = omega/theta;
    Q = eye(3)*sin(theta)/theta + hat(omega)*(1-cos(theta))/theta + omega*omega'*(1-sin(theta)/theta);         % ground truth
    %Q = eye(3)*sin(theta)/theta + hat(omega)*(1-cos(theta))/theta^2 + (omega*omega')*(theta - sin(theta))/theta^3;
    R = (R_matrix(omega*theta)-Q)*(omega'*v) + hat(cross(omega,v))*Q - Q*hat(cross(omega,v));
    intAd(1:3,1:3) = Q;
    intAd(4:6,4:6) = Q;
    intAd(1:3,4:6) = R;
end   

end

