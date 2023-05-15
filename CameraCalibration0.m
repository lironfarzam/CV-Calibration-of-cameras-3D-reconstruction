function P = CameraCalibration0(pts2d,pts3d)
   % function P = CameraCalibration0(pts2d,pts3d)
   % calculates the dlt coefficients p1..p11 of a camera from a set of
   % matched 2d and 3d points
   % P        is a 3x4 matrix
   % pts2d    is a 2Xn matrix of n 2D points
   % pts3d    is a 3xn matrix of n 3D points
   % Solves A*x = 0, where x is the vector of camera coefficients
   % that is reshaped as a calibration matrix P
   
   
   [n1, n2] = size(pts2d);
   if n1 ~= 2
      error(['pts2d should be an 2xn matrix and not ' num2str(n1) 'xn']);
   end
   
   [n3, n4] = size(pts3d);
   if n3 ~= 3
      error(['pts3d should be an 3xn matrix and not ' num2str(n3) 'xn']);
   end
   
   if n2 ~= n4
      error(['There should be an equal number of 2d and 3d points and not ' num2str(n2) ' and ' num2str(n4)]);
   end
   
   A = zeros(n2*2,12);
   
   for i=1:n2
      a=pts2d(1,i);
      b=pts2d(2,i);
      A(i*2-1,:) = [pts3d(:,i)' 1   0   0   0   0   (-a)*pts3d(:,i)' -a];
      A(i*2,:)   = [0   0   0   0   pts3d(:,i)' 1   (-b)*pts3d(:,i)' -b];
   end
   
   
   
   % % now solve A*x = 0
   % C = A'*A;
   % [V,D] = eig(C);
   % [i,j] = min(abs(diag(D)));
   % X = V(:,j)
   
   % or by SVD
   [~,~,R] = svd(A); 
   X = R(:,end);
   x = X/X(end);
   
   P = reshape(x,4,3)';
   
end