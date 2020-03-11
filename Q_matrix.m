function Q = Q_matrix(twist_matrix,theta)

Q = zeros(6,42);
Adg = zeros(6,6,7);
Adg(:,:,1) = eye(6);

[~,~,T_abs] = FK( twist_matrix,theta );
for i=2:7
   Adg(:,:,i) = Adjoint(T_abs(:,:,i-1));
end

for i=1:6
    Q(:,6*i-5:6*i) = Adg(:,:,i) - Adg(:,:,i+1);
end
Q(:,37:42) = Adg(:,:,7);
end
