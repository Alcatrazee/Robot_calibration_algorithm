function twist = log_my(T)
% log function,calculate inverse function of exponential 
R = T(1:3,1:3);
p = T(1:3,4);
if abs(trace(R)-3) < eps
    twist = [p;zeros(3,1)];
else 
    omega = [R(3,2)-R(2,3),R(1,3)-R(3,1),R(2,1)-R(1,2)]';
    theta = atan2(norm(omega),trace(R)-1);
    omega = omega/norm(omega);
    pitch = omega'*p/theta;
    if norm(cross(omega,p)) < eps
        v = pitch * omega;
    else
        p = (p - omega'*p*omega)/(2*sin(theta/2));
        v = p*cos(theta/2)-cross(omega,p)*sin(theta/2)+pitch*omega;
    end
    twist = [v;omega]*theta;
end
end

