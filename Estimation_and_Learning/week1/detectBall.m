% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
load('mu_sig_params.mat', 'mu', 'sigma');
thre = 0.5


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
hsv = rgb2hsv(I);
hsv = hsv(:,:,1);
G = 1/(2*pi*sigma)^0.5 * exp(-(hsv-mu).^2/2/sigma);
bw = G > (thre*max(G(:)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

bw_biggest = false(size(bw));
CC = bwconncomp(bw);
numPixels = cellfun(@numel, CC.PixelIdxList);
[biggest, idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true;
figure, imshow(bw_biggest); hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI = bw_biggest;

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
plot(loc(1), loc(2),'r+');

% 
% Note: In this assigmament, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
