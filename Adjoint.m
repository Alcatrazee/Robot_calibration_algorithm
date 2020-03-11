function adj = Adjoint(g)
% get adjoint matrix
adj = [g(1:3,1:3), hat(g(1:3,4))*g(1:3,1:3);
       zeros(3,3) , g(1:3,1:3)];
end