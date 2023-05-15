% Decomposition of the projection matrices

% Find the center of the projection by (null) P, normalize it so that the last term is 1,
% return in C onlythe first three members.

% you can write P = [M P_4], where M is the left 3x3 block of P.
% M represents The product M = K*R, where K is upper triangular and contains the internal parameters of the camera
% R the rotation matrix of the camera.
% We will use QR decomposition (the qr function of Matlab) to obtain K and R. 
% But there is a problem: qr returns a decomposition M = A*B, where A is square and B Upper triangular, different from what we want.
% Therefore we calculate qr(M^-1) and from it we get the desired:
% M^-1 = Q * L
% (M^-1)^-1 = L^-1 * Q^-1
% M = L^-1 * Q^-1
% K = L^-1
% R = Q^-1 = Q'
% Another problem is that the decomposition is not unique - K is defined up to scale and we may also get negative values
% ​​in the diagonal members of K contrary to what is desired for the internal parameters.
% We will check this and fix it:
% first we will normalize K so that the last term in it (K_3_3) will be 1.
% We will not fix R following this change.
% Second, if the other diagonal members (K_1_1, K_2_2) are negative, we will reverse their sign.
% This entails additional changes that need to be made in K and R so that the product K*R does not change.
% Finally, we will check the determinant of R which should be 1 (because it is a rotation matrix).
% If the determinant is -1, it must be corrected by multiplying the matrix by -1.

function [K, R, C] = DecomposeProjectionMatrix(P)
   % given a projection matrix P, finds its center of projection, its
   % rotation matrix and its inner parameters matrix
   % input:
   %        P  3x4 projection matrix.
   % output:
   %        K  3x3 an upper triangular matrix of inner camera parameters.
   %        R  3x3 the camera rotation matrix.
   %        C  3x1 the camera center of projection.

   
   
    % the camera center should be the null space of P
    % option 1: Using null space 
    % or P*C = 0
    nP = null(P);
    C = nP ./nP(4); % should also check that nP(4) ~= 0

%    % find the center of projection
%    % % option 2:
%    [~,~,V] = svd(P);
%    C = V(:,end);
%    C = C/C(end);
%    C = C(1:3);
   
  
   
   % find the rotation and the inner parameters
   M = P(:,1:3);
   [Q,L] = qr(inv(M));
   K = inv(L);
   R = Q';

   % normalize K so that the last term in it (K_3_3) will be 1
   K = K/K(3,3);

   % fix the sign of K_1_1 and K_2_2 if needed
   if K(1,1) < 0
      K(:,1) = -K(:,1);
      R(1,:) = -R(1,:);
   end
   
   if K(2,2) < 0
      K(:,2) = -K(:,2);
      R(2,:) = -R(2,:);
   end

   % check the determinant of R
   if det(R) < 0
      R = -R;
   end
end
