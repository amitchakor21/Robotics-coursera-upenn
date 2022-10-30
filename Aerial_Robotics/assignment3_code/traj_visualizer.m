close all;

clear;

trajhandle = @traj_generator;

waypoints = [0 0 0;

1 1 1;

2 0 2;

3 -1 1;

4 0 0]';

trajhandle([],[],waypoints);

d = waypoints(:,2:end) - waypoints(:,1:end-1);

segmentLength = sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);

waypointTimes = [0, cumsum(segmentLength)];

%% You will need to change the value for total_time if your speed differs from mine

total_time = sum(segmentLength);

wayZeros = zeros(size(waypointTimes,1));

t = 0:0.1:total_time;

n = size(t, 2);

pos = zeros(3, n);

vel = zeros(3, n);

for i = 1 : n

desired_state = trajhandle(t(i), []);

pos(:,i) = desired_state.pos;

vel(:,i) = desired_state.vel;

end

figure('name','Generated Trajectory (Desired State vs Time)','units','normalized','outerposition',[0,0,1,1]);

subplot(3,2,1)

plot(t, pos(1,:), 'r', waypointTimes, waypoints(1,:), 'r+'), ylabel('x'), title('Position');

subplot(3,2,3)

plot(t, pos(2,:), 'g', waypointTimes, waypoints(2,:), 'g+'), ylabel('y');

subplot(3,2,5)

plot(t, pos(3,:), 'b', waypointTimes, waypoints(3,:), 'b+'), ylabel('z');

subplot(3,2,2)

plot(t, vel(1,:), 'r', waypointTimes, wayZeros, 'r+'), ylabel('vx'), title('Velocity');

subplot(3,2,4)

plot(t, vel(2,:), 'g', waypointTimes, wayZeros, 'g+'), ylabel('vy');

subplot(3,2,6)

plot(t, vel(3,:), 'b', waypointTimes, wayZeros, 'b+'), ylabel('vz');

for i = waypointTimes

s = trajhandle(i, -42);

fprintf('At time %6.2f, pos=[%6.4f, %6.4f, %6.4f]\n', i, s.pos(1), s.pos(2), s.pos(3));

end