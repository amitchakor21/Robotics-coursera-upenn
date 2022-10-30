
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));
  P = 1e3*eye(2); % prior error covariance
  Q = diag([10000, 0.6]); %assumed covariance of process a.k.a process noise
  R = diag([0.001, 0.01, 30]);%measurement noise
  for k=2:length(t)
      dt = t(k)-t(k-1);
      A = [1 dt; 0 1];
      %prediction step
      xhat(:,k) = A*xhat(:, k-1);%newtons laws of motion xt = xt-1+ v*dt+0.5*a*dt^2 & v=vt-1+a*dt
      P = A*P*A'+Q;% 
      H = [cosd(xhat(1,k-1)) 0; -sind(xhat(1,k-1)) 0 ; 0 1]*0.3;% H are scaling matrix?
      h = [sind(xhat(1,k-1)); cosd(xhat(1,k-1)); xhat(2,k-1)]; % h = H*P
      
      %Kalman Gain
      K = P*H'/(H*P*H'+R);% hypothesis = mean*(covariance inverse)
      %if covariance = 1 confidence in previous prediction is high 
      %else vice-versa
      
      %update
      xhat(:,k) = xhat(:,k) + K*(z(:,k) - h);%update
      P = (eye(2)-K*H)*P;% 
end
