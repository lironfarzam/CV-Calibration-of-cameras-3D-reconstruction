function T = Get2DNormalizationTransform(X)
    % Computes a similarity transformation T consisting of
    % a translation and scaling, that takes the points X to a new
    % set of points such that the centroid of the new set
    % is the origin (0,0)', and their average
    % distance from the origin is sqrt(2).
    %
    % note that T works on homogeneous coordinates
    % Xn = T * Xh
    % where Xh are the 2d homogeneous representation of X.
    %
    % input:
    % X 2 x n matrix of non-homogeneous 2D coordinates:
    % x1 x2 ... xn
    % y1 y2 ... yn
    % output:
    % T 3x3 transformation matrix
    %
    % see Check2DNormalizationTransform
    
    % get the centroid
    c = mean(X,2);

    % get the average distance
    d = mean(sqrt(sum((X-c*ones(1,size(X,2))).^2,1)));

    % construct the transformation matrix
    T = [sqrt(2)/d 0 -c(1)*sqrt(2)/d ; 0 sqrt(2)/d -c(2)*sqrt(2)/d ; 0 0 1];
    end