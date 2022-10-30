function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

[N,~] = size(X0);
X = zeros(N,3);

for j = 1:10
    for i =1:N
        X_refine = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:)');   
        X(i,:) = X_refine';
    end
 
end 


end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
    
    J = [Jacobian_Triangulation(C1, R1, K, X0);...
        Jacobian_Triangulation(C2, R2, K, X0);...
        Jacobian_Triangulation(C3, R3, K, X0)];

    
uvw1 = K*R1*(X0-C1);
uvw2 = K*R2*(X0-C2);
uvw3 = K*R3*(X0-C3);

u1 = uvw1(1) ;v1 = uvw1(2) ;w1 = uvw1(3) ;
u2 = uvw2(1) ;v2 = uvw2(2) ;w2 = uvw2(3) ;
u3 = uvw3(1) ;v3 = uvw3(2) ;w3 = uvw3(3) ;

fx = [u1/w1 ; v1/w1 ; u2/w2 ; v2/w2 ; u3/w3 ; v3/w3];
b = [x1,x2,x3]';

del = (J'*J)\J'*(b-fx);
X = X0 + del;


end

function J = Jacobian_Triangulation(C, R, K, X)

uvw = K*R*(X-C);
u = uvw(1);
v = uvw(2);
w = uvw(3);

f = K(1,1);
px = K(1,3);
py = K(2,3);


dwdx = R(3,:);
dvdx = [f*R(2,1)+py*R(3,1),f*R(2,2)+py*R(3,2),f*R(2,3)+py*R(3,3)];
dudx = [f*R(1,1)+px*R(3,1),f*R(1,2)+px*R(3,2),f*R(1,3)+px*R(3,3)];

dfdx = [(w*dudx-u*dwdx)/w^2;(w*dvdx-v*dwdx)/w^2];

J = [dfdx']';
end
