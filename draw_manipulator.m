function draw_manipulator( twist_matrix,gst0,color,measurment_type )

% measurment_type : 1:pose  2:point
% if measurment type is points,then input parameter gst0 is a column vector
% of points' coordinates in homogenius form 

hold on;
view([45 35]);
axis equal;
q_vec = cross(twist_matrix(4:6,1:6),twist_matrix(1:3,1:6));
[T,~,~] = FK(twist_matrix,[zeros(1,6),1]);
if(measurment_type==1)
    gst = T*gst0;
elseif(measurment_type==2)
    size_of_points_set = size(gst0);
    num_of_pts = size_of_points_set(2);
    gst = zeros(4,num_of_pts);
    for i=1:num_of_pts
        gst(:,i) = T*gst0(:,i);
    end
end

for i = 1:6
    point_set = [q_vec(:,i) - 500*twist_matrix(4:6,i),q_vec(:,i) + 500*twist_matrix(4:6,i)]';
    plot3(point_set(:,1),point_set(:,2),point_set(:,3),color);
end

if(measurment_type==1)
    color_arrow = ['r' 'g' 'b'];
    for i = 1:3
        point_set = [gst(1:3,4),gst(1:3,4) + 300*gst(1:3,i)]';
        plot3(point_set(:,1),point_set(:,2),point_set(:,3),color_arrow(i));
    end
elseif (measurment_type==2)
    for i=1:num_of_pts
        plot3(gst(1,i),gst(2,i),gst(3,i),color,'marker','*');
    end
end
hold off;
end

