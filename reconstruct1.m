function X = reconstruct1(P1, P2, a, b) 
   % Calculates 3D points from a set of matching 2D points 
   % and two camera calibration matrices. 
   % input:
   %         P1, P2 are the 3x4 calibration matrices of the 2 cameras. 
   %         a, b 2xn matrices of n matching 2D points in the two images in camera coordinates.
   % output:
   %         X 3xn output matrix of 3D points.
   % 
   % For each matrix, the center of the projection is calculated, and the direction of the z-axis is calculated
   % as a direction vector from the center of the projection to a point on the ray, and thus we find the equation
   % of the line represented by a direction vector and an anchor point C.
   % Then the intersection of the 2 rays is calculated in the form of Ax=b 
   % We will solve it using the pseudo-inverse
   % and use the first and second ray equations to find the average of the points.

    % Check input dimensions
    [n1, n2] = size(a);
    if n1 ~= 2
        error(['a should be a 2xn matrix and not ' num2str(n1) 'xn']);
    end
    [n3, n4] = size(b);
    if n3 ~= 2
        error(['b should be a 2xn matrix and not ' num2str(n3) 'xn']);
    end
    if n2 ~= n4
        error(['a and b should have the same number of columns and not ' num2str(n2) ' and ' num2str(n4)]);
    end

    N = n2;  % Number of points

    % Compute the center of projection for camera 1
    C1 = null(P1);
    C1 = C1 ./ C1(end);
    
    % Compute the center of projection for camera 2
    C2 = null(P2);
    C2 = C2 ./ C2(end);
    
    X = zeros(3, N);

    for i = 1:N
        % Compute the 3D point from the matched 2D points

        % Convert 2D points to homogeneous coordinates
        p_a = pinv(P1) * [a(:, i); 1];
        p_b = pinv(P2) * [b(:, i); 1];
        
        % Finding direction vectors 
        V1 = C1 - p_a;
        V1 = V1 / norm(V1);
        V2 = C2 - p_b;
        V2 = V2 / norm(V2);
        
        % Rays equations:
        % r1 = C1 + t1 * V1;
        % r2 = C2 + t2 * V2;
        % t1 and t2 are scalar unknowns, so we have 3 equations and 2 unknowns (3D, not homogenized).
        % We solve the problem Ax = b using the pseudo-inverse.
        A = [V1, -V2];
         
        b_vec = C2 - C1;
        
        % Solve the linear system Ax = b using pseudo-inverse
        t = pinv(A) * b_vec;

        % Calculate the 3D point using the average of the ray intersections
        temp = ((C1 + t(1) * V1) + (C2 + t(2) * V2)) / 2;
        X(:, i) = temp(1:3)/temp(end);
    end
end


    
