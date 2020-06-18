function A = A_tilde_matrix(twist,eta_matrix,P,theta)

[T,~,T_link_abs] = FK(twist,theta);                 % calculate FK to get T_abs and T
% get Q1
Q_i = theta(1)*int_Adjoint(twist(:,1)*theta(1));        % Q1
temp = Adjoint(T_matrix(eta_matrix(:,1)))*twist(:,1);             % Adexp(eta_1)*twist_1
ad_Adg = [hat(temp(4:6)),hat(temp(1:3));                % ad(Adexp)
    zeros(3)    ,hat(temp(4:6))]*int_Adjoint(eta_matrix(:,1));
Q_tilde_i = -[eye(3) -hat(P(1:3))]*Q_i*ad_Adg;   % Q1_tilde_i

A = zeros(3,39);
A(:,1:6) = Q_tilde_i;
% get the rest of Qi
for i=2:6
    Q_i = theta(i)*Adjoint(T_link_abs(:,:,i-1)) * int_Adjoint(twist(:,i)*theta(i));
    temp = Adjoint(T_matrix(eta_matrix(:,i)))*twist(:,i);
    ad_Adg = [hat(temp(4:6)),hat(temp(1:3));
        zeros(3)    ,hat(temp(4:6))]*int_Adjoint(eta_matrix(:,i));
    Q_tilde_i = -[eye(3) -hat(P(1:3))]*Q_i*ad_Adg;
    A(:,6*i-5:6*i) = Q_tilde_i;
end
Rf = T(1:3,1:3);
A(:,37:39) = Rf;