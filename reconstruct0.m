% Use algebraic triangulation to calculate the three-dimensional points from the two-dimensional points and the projection matrices of the two cameras.
function X = reconstruct0(P1,P2,a,b)
   % calculates 3d points from set of matching 2d points
   % and two camera calibration matrices.
   % X       3xn output matrix of 3D points.
   % P1,P2   are the 3x4 calibration matrices of the 2 cameras
   %         cameras.
   % a,b     2Xn matrices of n matching 2d points
   %         uses svd to solve Ax=0 equation.

   % check input
   [n1, n2] = size(a);
   if n1 ~= 2
      error(['a should be an 2xn matrix and not ' num2str(n1) 'xn']);
   end
   [n3, n4] = size(b);
   if n3 ~= 2
      error(['b should be an 2xn matrix and not ' num2str(n3) 'xn']);
   end
   if n2 ~= n4
      error(['a and b should have the same number of columns and not ' num2str(n2) ' and ' num2str(n4)]);
   end

   N = n2;  % Number of points
    
    % Construct the matrix A
    A = zeros(4, 4);
    X = zeros(4, N);
    
    for i = 1:N
        A(1, :) = a(1, i) * P1(3, :) - P1(1, :);
        A(2, :) = a(2, i) * P1(3, :) - P1(2, :);
        A(3, :) = b(1, i) * P2(3, :) - P2(1, :);
        A(4, :) = b(2, i) * P2(3, :) - P2(2, :);
        
        [~, ~, V] = svd(A);
        X(:, i) = V(:, end);
    end
    
    % Dehomogenize the 3D points
    X = X(1:3, :) ./ X(4, :);
end

   
