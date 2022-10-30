function quat_convention
vecpart = @(q) reshape(q(2:4),[],1);
disp('------------------------------------------------------------------');
disp('------------------------------------------------------------------');
disp(' ');
disp('The following code demonstrates the MATLAB quaternion convention');
disp(' ');
disp('Create an arbitrary unit quaternion');
q = randn(1,4); q = q/norm(q)
disp(' ');
disp('Create an arbitrary vector');
v = randn(3,1)
disp(' ');
disp('Get corresponding direction cosine matrix');
disp('dc = quat2dcm(q)');
dc = quat2dcm(q)
disp(' ');
disp('Rotate the vector using the direction cosine matrix');
disp('dc*v')
disp(dc*v);
disp(' ');
disp('Rotate the vector using the quaternion');
disp('conj(q)*v*q')
disp(vecpart(quatmultiply(quatconj(q),quatmultiply([0 v'],q))));
disp(' ');
disp('Differrence in rotated vectors (should be small)');
dcv = dc*v;
qcvq = vecpart(quatmultiply(quatconj(q),quatmultiply([0 v'],q)));
disp(max(abs(dcv(:)-qcvq(:))));
disp(' ');
disp('Quaternion convention is scalar first and successive rotations are RIGHT multiplies');
disp(' ');
disp('------------------------------------------------------------------');
disp(' ');
disp('Rodrigues Rotation Formula');
disp(' ');
if( q(1) < 0 )
    qc = q;
else
    qc = -q;
end
disp('cos(theta)');
c = cos(acos(qc(1))*2)
disp(' ');
disp('sin(theta)');
s = sin(asin(norm(qc(2:4)))*2)
disp(' ');
disp('theta (deg)');
disp(atan2(s,c)*180/pi);
disp(' ');
disp('Rotation axis');
k = qc(2:4)/norm(qc(2:4))
disp(' ');
disp('Skew symmetric matrix');
K = [0 -k(3)  k(2); k(3) 0 -k(1); -k(2) k(1) 0]
disp(' ');
disp('R = eye(3) + s*K + (1-c)*K^2');
R = eye(3) + s*K + (1-c)*K^2
disp(' ');
disp('Differrence in dc matrices (should be small)');
disp(max(abs(R(:)-dc(:))));
end