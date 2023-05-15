function U = Get3DNormalizationTransform(X)
    % Computes a similarity transformation U consisting of
    % a translation and scaling, that takes the points X to a new
    % set of points such that the centroid of the new set
    % is the origin (0,0,0)', and their average
    % distance from the origin is sqrt(3).
    %
    % note that U works on homogeneous coordinates
    % Xn = U * Xh
    % where Xh are the 3d homogeneous representation of X.
    %
    % input:
    % X 3 x n matrix of non- homogeneous 3D coordinates:
    % x1 x2 ... xn
    % y1 y2 ... yn
    % z1 z2 ... zn
    % output:
    % U 4x4 transformation matrix
    %
    % see Check3DNormalizationTransform

    % get the centroid
    c = mean(X,2);

    % get the average distance
    d = mean(sqrt(sum((X-c*ones(1,size(X,2))).^2,1)));

    % construct the transformation matrix
    U = [sqrt(3)/d 0 0 -c(1)*sqrt(3)/d ; 0 sqrt(3)/d 0 -c(2)*sqrt(3)/d ; 0 0 sqrt(3)/d -c(3)*sqrt(3)/d ; 0 0 0 1];
    end