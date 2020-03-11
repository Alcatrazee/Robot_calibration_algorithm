function [w1,w2] = orthogonal_w_generator(w)

% this function is to generate two orthogonal unit vector that perpendicular to
% the input vector

w1 = randn(3,1);
w1 = w1/norm(w1);
w = w/norm(w);
w1 = w1 - w1'*w*w;

w2 = cross(w1,w);
