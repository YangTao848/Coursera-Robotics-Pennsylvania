function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 
X=X0;

for i=1:size(X0,1)
    x=X(i,:)';
    b=[x1(i,1) x1(i,2) x2(i,1) x2(i,2) x3(i,1) x3(i,2)]'; 
    W1=K*R1*(x-C1);
    W2=K*R2*(x-C2);
    W3=K*R3*(x-C3);
    
    fX=[W1(1)/W1(3) W1(2)/W1(3) W2(1)/W2(3) W2(2)/W2(3) W3(1)/W3(3) W3(2)/W3(3)]';
    
    f=K(1,1);
    px=K(1,3);
    py=K(2,3); 
    
    du1=[f*R1(1,1)+px*R1(3,1) f*R1(1,2)+px*R1(3,2) f*R1(1,3)+px*R1(3,3)];
    dv1=[f*R1(2,1)+py*R1(3,1) f*R1(2,2)+py*R1(3,2) f*R1(2,3)+py*R1(3,3)];
    dw1=[R1(3,1) R1(3,2) R1(3,3)];
    df1=[(W1(3)*du1-W1(1)*dw1)/W1(3)^2 ; (W1(3)*dv1-W1(2)*dw1)/W1(3)^2]';
    
    du2=[f*R2(1,1)+px*R2(3,1) f*R2(1,2)+px*R2(3,2) f*R2(1,3)+px*R2(3,3)];
    dv2=[f*R2(2,1)+py*R2(3,1) f*R2(2,2)+py*R2(3,2) f*R2(2,3)+py*R2(3,3)];
    dw2=[R2(3,1) R2(3,2) R2(3,3)];
    df2=[(W2(3)*du2-W2(1)*dw2)/W2(3)^2 ; (W2(3)*dv2-W2(2)*dw2)/W2(3)^2]';
    
    du3=[f*R3(1,1)+px*R3(3,1) f*R3(1,2)+px*R3(3,2) f*R3(1,3)+px*R3(3,3)];
    dv3=[f*R3(2,1)+py*R3(3,1) f*R3(2,2)+py*R3(3,2) f*R3(2,3)+py*R3(3,3)];
    dw3=[R3(3,1) R3(3,2) R3(3,3)];
    df3=[(W3(3)*du3-W3(1)*dw3)/W3(3)^2 ; (W3(3)*dv3-W3(2)*dw3)/W3(3)^2]';

    J=[df1 df2 df3]';
    
    deltaX=inv(J'*J)*J';
    deltaX=deltaX*(b-fX);
    
    x=x+deltaX;
    X(i,:)=x';
end



end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
end

function J = Jacobian_Triangulation(C, R, K, X)
end
