% In this file the parameters - mu(mean) and sigma(variance) are caclculated
% using MATLAB function - rgb2hsv

load('./step1_samples.mat','Samples');

% Convert the color space to hsv
hsv = rgb2hsv(im2double(Samples));

%visualising the result by plotting a 3D plot
scatter3(hsv(:,1),hsv(:,2),hsv(:,3),'.');

% taking mean and variance of "hue" values
mu = mean(hsv(:,1));
sigma = var(hsv(:,1));

save('mu_sig_params.mat', 'mu', 'sigma');
