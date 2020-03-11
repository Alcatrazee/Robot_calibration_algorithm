function A = A_matrix( twist_matrix,theta )
% A matrix calculation
A = zeros(6,42);
[~,~,T_link_abs] = FK_new(twist_matrix,theta);
A(:,1:6) = theta(1)*int_Adjoint(twist_matrix(:,1)*theta(1));
for i = 1:6
    A(:,1+i*6:6+i*6) = theta(i+1)*Adjoint(T_link_abs(:,:,i)) * int_Adjoint(twist_matrix(:,i+1)*theta(i+1));
end
end

