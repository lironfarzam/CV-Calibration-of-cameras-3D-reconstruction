% The normalization of the data improves the numerical stability and the
% quality of the results of the evaluation of the projection matrices.
% The normalization will be done by ~X = UX
% Where T and U are transformations that change the points so that their
% average change is 0And their average distance from the origin will be 2
% for the two-dimensional points and 3 for the three-dimensional.
% The projection equation for the normalized points is ~x = ~P ~X

function P = CameraCalibration1(pts2d,pts3d)
    % function P = CameraCalibration1(pts2d,pts3d)
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

    % normalize the points
    T = Get2DNormalizationTransform(pts2d);
    pts2d = T*[pts2d; ones(1,size(pts2d,2))];
    U = Get3DNormalizationTransform(pts3d);
    pts3d = U*[pts3d; ones(1,size(pts3d,2))];



    A = zeros(n2*2,12);

    for i=1:n2
        a=pts2d(1,i);
        b=pts2d(2,i);
        A(i*2-1,:) = [pts3d(:,i)'   0   0   0   0   (-a)*pts3d(:,i)'];
        A(i*2,:)   = [0   0   0   0   pts3d(:,i)'   (-b)*pts3d(:,i)'];
    end

    % % now solve A*x =
    % C = A'*A;
    % [V,D] = eig(C);
    % [i,j] = min(abs(diag(D)));
    % X = V(:,j)

    % or by SVD
    [~,~,R] = svd(A);
    x = R(:,end);

    P = reshape(x,4,3)';

    % denormalize the points
    % P = inv(T)*P*U;
    P = T\P*U;
    P = P/P(end,end);
end
