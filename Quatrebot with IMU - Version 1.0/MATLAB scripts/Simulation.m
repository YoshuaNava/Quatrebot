syms q1 q2 x_desired y_desired

NUM_POINTS_TRAJECTORY = 70;
D1 = 130.0;
D2 = 260.0;
origin_robot = [132.0, 200.0];

via_points = [[200, 80]; ...
              [300, 20]; ...
              [380, 320]; ...
              [20, 380]; ...
              [380, 380]];

num_via_points = 5;
trajectory = zeros(NUM_POINTS_TRAJECTORY,2);;
total_distance = 0;
total_points = 0;
points_calculated = 0;

for i=1:num_via_points-1
    distance_segment_via_points(i) = sqrt((via_points(i,1)-via_points(i+1,1))^2 + (via_points(i,2)-via_points(i+1,2))^2);
    total_distance = total_distance + distance_segment_via_points(i);
end

for i=1:num_via_points-1
    points_per_segment(i) = floor((distance_segment_via_points(i) / total_distance) * NUM_POINTS_TRAJECTORY);
    if (((distance_segment_via_points(i) / total_distance) * NUM_POINTS_TRAJECTORY) - floor((distance_segment_via_points(i) / total_distance) * NUM_POINTS_TRAJECTORY) > 0.5)
        points_per_segment(i) = points_per_segment(i) + 1;
    end
    total_points = total_points + points_per_segment(i);
end

for i=1:num_via_points
    via_points_robot(i,:) = via_points(i,:) - origin_robot;
end

for i=1:num_via_points-1
   s = 1.0 / points_per_segment(i);
   for j=1:points_per_segment(i)
      trajectory(points_calculated + j,:) = via_points_robot(i,:)*(1-s*(j-1)) + s*(j-1)*via_points_robot(i+1,:);
   end
   points_calculated = points_calculated + j;
end

points_calculated = points_calculated + 1;
trajectory(points_calculated,:) = via_points_robot(i+1,:);

via_points_robot
trajectory
figure(1)
plot(trajectory(:,1),trajectory(:,2),'r')
axis([0-origin_robot(1),400-origin_robot(1), 0-origin_robot(2),400-origin_robot(2)])
hold on
plot(via_points_robot(:,1),via_points_robot(:,2),'b*')

 tr = trot2(q1*(pi/180))*transl2(D1,0)*trot2(q2*(pi/180))*transl2(D2,0);
 pjoint1 = [D1*cos(q1*(pi/180)), D1*sin(q1*(pi/180))];
 pEF = tr(1:2,3)';
% pose_joint1 = double(subs(pjoint1, [q1], [-133.42]));
% pose_joint2 = double(subs(pEF, [q1, q2], [-133.42, 96]));
% arm = [[0,0]; pose_joint1; pose_joint2];
% plot(arm(:,1),arm(:,2),'g')

for i=1:NUM_POINTS_TRAJECTORY+1
    x_desired = trajectory(i,1);
    y_desired = trajectory(i,2);

    cos_q2 = (x_desired^2 + y_desired^2 - D1^2 - D2^2)/(2*D1*D2);
    temp_q2 = acos(cos_q2)*(180.0)/pi;
    temp1_q1 = (atan2(y_desired, x_desired) - atan2(sin(temp_q2*(pi/180))*D2, cos_q2*D2 + D1))*(180.0)/pi;
    %temp2_q1 = (atan2(y_desired, x_desired) - atan2(sin(temp_q2*(pi/180))*D2, cos_q2*D2 + D1)*(180.0))/pi

    %temp1_q1 = -160
    joint1_desired = double(subs(pjoint1, [q1], [temp1_q1]));
    joint2_desired = double(subs(pEF, [q1, q2], [temp1_q1, temp_q2]));
    arm_desired = [[0,0]; joint1_desired; joint2_desired];
    clf
    plot(trajectory(:,1),trajectory(:,2),'r')
    axis([0-origin_robot(1),400-origin_robot(1), 0-origin_robot(2),400-origin_robot(2)])
    hold on
    plot(via_points_robot(:,1),via_points_robot(:,2),'b*')
    plot(arm_desired(:,1),arm_desired(:,2),'k')
    pause(0.2)
    disp(['Point #',num2str(i)]);
    disp('Angles of joints 1 and 2 = ');
    disp(['q1 = ',num2str(temp1_q1),'; q2 = ',num2str(temp_q2)]);
    disp('Arm end effector position =');
    disp(['x = ',num2str(x_desired),'; y = ',num2str(y_desired)]);
    %x_desired = trajectory(i,1) + origin_robot(1)
    %y_desired = trajectory(i,2) + origin_robot(2)
end