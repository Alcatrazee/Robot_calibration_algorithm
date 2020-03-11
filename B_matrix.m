function B = B_matrix(twist_matrix,measuring_type)

% measuring type: 1:pose 2:point

if measuring_type==1
    B = zeros(42,30);
elseif measuring_type==2
    B = zeros(42,27);
else
    error('Measuring type assert failed. Please check parameter.');
end

for i=1:6
    qn = cross(twist_matrix(4:6,i),twist_matrix(1:3,i));
    [w1,w2] = orthogonal_w_generator(twist_matrix(4:6,i));
    B(6*i-5:6*i,4*i-3:4*i) = [w1,w2,cross(qn,w1),cross(qn,w2);zeros(3,2),w1,w2];
end
if measuring_type==1
    B(37:42,25:30) = eye(6);
elseif measuring_type==2
    B(37:42,25:27) = [eye(3);zeros(3)];
end
end