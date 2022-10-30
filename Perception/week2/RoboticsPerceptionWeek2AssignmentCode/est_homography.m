function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
ax = [ -video_pts(:,1),-video_pts(:,2),-1*ones(4,1),zeros(4,1),zeros(4,1),zeros(4,1),video_pts(:,1).*logo_pts(:,1),video_pts(:,2).*logo_pts(:,1),logo_pts(:,1)];
ay = [ zeros(4,1),zeros(4,1),zeros(4,1),-video_pts(:,1),-video_pts(:,2),-1*ones(4,1),video_pts(:,1).*logo_pts(:,2),video_pts(:,2).*logo_pts(:,2),logo_pts(:,2)];
A = [ax(1,:);ay(1,:);ax(2,:);ay(2,:);ax(3,:);ay(3,:);ax(4,:);ay(4,:)];

[U, S, V] = svd(A);
H = V(:, end);
H = reshape(H, 3, 3);
H = H';
end

