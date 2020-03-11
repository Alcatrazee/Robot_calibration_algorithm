function hatted = hat(vec)
% get a skew-symmetric matrix
if(length(vec)==3)
    hatted = [0 -vec(3) vec(2);vec(3) 0 -vec(1);-vec(2) vec(1) 0];
elseif (length(vec)==6)
    hatted = [hat(vec(4:6)) vec(1:3);0 0 0 0];
end
end