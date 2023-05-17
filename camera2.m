% Calibration-of-cameras-3D-reconstruction
%
% In this exercise you will evaluate the casting matrix of cameras from matching points,
% perform a disassemblyof the projection matrix to find the center of the projection
% and the rotation matrix of each camera, show the systemThe coordinates of the cameras
% in space will reconstruct a three-dimensional body from two-dimensional projections
% and displayhim.
%

function camera2()
    % Main function

    % Load the data
    data = load('data.mat');
    Q = data.Xo;
    q1 = data.q1;
    q2 = data.q2;

    format long % Scaled fixed point output format with 15 digits for double
    close all
    home
    h = figure(1); clf
    set(h,'Position',[293   330   418   329]);
    d = 75;   % dimensions are in centimeters
    axis([-1 1 -1 1 -1 1]*d);
    % axis equal
    % axis padded
    hold on
    grid on
    cameratoolbar('ResetCameraAndSceneLight')
    cameratoolbar('Show')
    cameratoolbar('SetMode','orbit')
    cameratoolbar('SetCoordSys','y')
    campos(100*[ 1.109885587469520 , 1.212077444132104, 1.849173147328754]);
    % see
    % https://www.mathworks.com/matlabcentral/answers/427295-how-can-i-keep-the-plot-interaction-options-permanently-visible-in-the-axes-toolbar
    % https://www.mathworks.com/matlabcentral/answers/419036-what-happened-to-the-figure-toolbar-why-is-it-an-axes-toolbar-how-can-i-put-the-buttons-back
    % about the behavior of the annoying grey tooltip that appears with mouse hover % to remove it use this
    ax = gca;
    ax.Toolbar.Visible = 'off'; % Turns off the axes toolbar
    % Show the world coordinate system
    text(0,0,0,'w')
    plot3([0 20],[0 0],[0 0],'r')
    h = text(20,0,0,'x');
    set(h,'Color','r')
    xlabel('x')
    plot3([0 0],[0 20],[0 0],'g')
    h = text(0,20,0,'y');
    set(h,'Color','g')
    ylabel('y')
    plot3([0 0],[0 0],[0 20],'b')
    h = text(0,0,20,'z');
    set(h,'Color','b')
    zlabel('z')

    % drow the world axes
    plot3([0 20],[0 0],[0 0],'r','linewidth',2)
    plot3([0 0],[0 20],[0 0],'g','linewidth',2)
    plot3([0 0],[0 0],[0 20],'b','linewidth',2)
    % add labels to the axes
    text(20,0,0,'X')
    text(0,20,0,'Y')
    text(0,0,20,'Z')
    text(0,0,0,'W')

    h = helpdlg('Press OK to perform calibration');
    set(h,'Position',[282       104       196        76]);
    uiwait(h)

    % Fig 2: 2D image for q1
    figure(2); clf
    % axis([-50 50 -50 50]);
    axis equal; hold on; grid on;
    % name the figure window as 'q1'
    set(gcf,'Name','q1')
    % drow the points from q1 on the image
    plot(q1(1,:),q1(2,:),'*', 'Color', 'magenta')

    % Fig 3: 2D image for q2
    figure(3); clf
    % axis([-50 50 -50 50]);
    axis equal; hold on; grid on;
    % name the figure window as 'q2'
    set(gcf,'Name','q2')
    % drow the points from q2 on the image
    plot(q2(1,:),q2(2,:),'*', 'Color', 'cyan')

    % drow the points from Q on the image
    figure(1);
    plot3(Q(1,:),Q(2,:),Q(3,:),'k.')

    % Calculate the projection matrix for q1
    disp("Calculate the projection matrix ,CameraCalibration0:");
    P01 = CameraCalibration0(q1,Q)
    P02 = CameraCalibration0(q2,Q)

    % normalization:
    % Calculate the projection matrix for q1
    disp("Calculate the projection matrix ,CameraCalibration1:");
    P1 = CameraCalibration1(q1,Q)
    P2 = CameraCalibration1(q2,Q)

    % Decompose the projection matrix:
    disp("projection matrix P1:");
    [K1, R1, C1] = DecomposeProjectionMatrix(P1)
    disp("projection matrix P2:");
    [K2, R2, C2] = DecomposeProjectionMatrix(P2)

    % transform the rotation matrix to be from the world to the camera
    R1 =R1';
    R2 =R2';


    h = helpdlg('Press OK to perform calibration');
    set(h,'Position',[282       104       196        76]);
    uiwait(h)

    figure(1);
    % camera 1 axis
    plot3([C1(1) C1(1)+20*R1(1,3)],[C1(2) C1(2)+20*R1(2,3)],[C1(3) C1(3)+20*R1(3,3)],'r','linewidth',2)
    plot3([C1(1) C1(1)+20*R1(1,2)],[C1(2) C1(2)+20*R1(2,2)],[C1(3) C1(3)+20*R1(3,2)],'g','linewidth',2)
    plot3([C1(1) C1(1)+20*R1(1,1)],[C1(2) C1(2)+20*R1(2,1)],[C1(3) C1(3)+20*R1(3,1)],'b','linewidth',2)
    text(C1(1)+20*R1(1,3),C1(2)+20*R1(2,3),C1(3)+20*R1(3,3),'Z')
    text(C1(1)+20*R1(1,2),C1(2)+20*R1(2,2),C1(3)+20*R1(3,2),'Y')
    text(C1(1)+20*R1(1,1),C1(2)+20*R1(2,1),C1(3)+20*R1(3,1),'X')
    text(C1(1),C1(2),C1(3),'C1')

    % camera 2 axis
    plot3([C2(1) C2(1)+20*R2(1,3)],[C2(2) C2(2)+20*R2(2,3)],[C2(3) C2(3)+20*R2(3,3)],'b','linewidth',2)
    plot3([C2(1) C2(1)+20*R2(1,2)],[C2(2) C2(2)+20*R2(2,2)],[C2(3) C2(3)+20*R2(3,2)],'g','linewidth',2)
    plot3([C2(1) C2(1)+20*R2(1,1)],[C2(2) C2(2)+20*R2(2,1)],[C2(3) C2(3)+20*R2(3,1)],'r','linewidth',2)
    text(C2(1)+20*R2(1,3),C2(2)+20*R2(2,3),C2(3)+20*R2(3,3),'Z')
    text(C2(1)+20*R2(1,2),C2(2)+20*R2(2,2),C2(3)+20*R2(3,2),'Y')
    text(C2(1)+20*R2(1,1),C2(2)+20*R2(2,1),C2(3)+20*R2(3,1),'X')
    text(C2(1),C2(2),C2(3),'C2')

    h = helpdlg('Press OK to perform calibration');
    set(h,'Position',[282       104       196        76]);
    uiwait(h)


    % for each point in the first camera
    for i=1:size(q1,2)
        % find the point X
        X = pinv(P1)*[q1(:,i);1];
        % plot the line between C and X
        plot3([C1(1) X(1)],[C1(2) X(2)],[C1(3) X(3)],'magenta')
    end

    % for each point in the second camera
    for i=1:size(q2,2)
        % find the point X
        X = pinv(P2)*[q2(:,i);1];
        % plot the line between C and X
        plot3([C2(1) X(1)],[C2(2) X(2)],[C2(3) X(3)],'cyan')
    end

    h = helpdlg('Press OK to perform calibration');
    set(h,'Position',[282       104       196        76]);
    uiwait(h)


    % Also present the continuation of the line to the other side - to the throwing plains.
    % For this you need to know f forAny camera. Given that mx = my =100 [pix/cm] for both cameras.
    % Use the K values we found for each camera we can calculate f for each camera.
    mx = 100; % [pix/cm]

    % camera 1
    f1 = K1(1,1)/mx
    % projection matrix from World to Camera 1
    W2C1 = [R1 C1;
        0 0 0 1];
    % the throwing plane is the plane z=f1 in the world coordinate system
    imagwPlane1 = [-10  10  10 -10 -10;
        10  10 -10 -10  10;
        f1  f1  f1  f1  f1;
        1   1   1   1   1];
    imagePlane1 = W2C1*imagwPlane1;
    % add the plane to the figure (1)
    plot3(imagePlane1(1,:),imagePlane1(2,:),imagePlane1(3,:) ,'-', 'Color', 'r')

    % add the line from shape to the image plane:
    % homogenous coordinates
    pts2d = [q1; ones(1, size(q1, 2))];

    % normalize the points
    Zc = univec(det(P1(:,1:3))*P1(3,1:3));

    for i=1:size(pts2d, 2)
        pts = zeros(3,2);
        pts(:,1) = C1;

        X = pinv(P1)*pts2d(:,i);
        X = X / X(4);
        pts(:,2) = X(1:3);

        V = C1 - X(1:3);
        V = univec(V);

        f = f1 / (Zc * V);

        plot3(pts(1,:), pts(2,:), pts(3,:), '-', 'Color', 'magenta');
        x = C1 + f*V;

        pts = [C1, x];
        plot3(pts(1,:), pts(2,:), pts(3,:), '-', 'Color', 'magenta');
        plot3(x(1), x(2), x(3),'*', 'Color', 'magenta');
        plot3(x(1), x(2), x(3),'o', 'Color', 'k');
    end

    % camera 2
    f2 = K2(1,1)/mx
    % projection matrix from World to Camera 2
    W2C2 = [R2 C2; 0 0 0 1];
    % the throwing plane is the plane z=f2 in the world coordinate system
    imagwPlane2 = [-10  10  10 -10 -10;
        10  10 -10 -10  10;
        f2  f2  f2  f2  f2;
        1   1   1   1   1];
    imagePlane2 = W2C2*imagwPlane2;
    % add the plane to the figure (1)
    plot3(imagePlane2(1,:),imagePlane2(2,:),imagePlane2(3,:),'-', 'Color', 'r')

    % add the line from shape to the image plane:
    % homogenous coordinates
    pts2d = [q2; ones(1, size(q2, 2))];

    % normalize the points
    Zc = univec(det(P2(:,1:3))*P2(3,1:3));

    for i=1:size(pts2d, 2)
        pts = zeros(3,2);
        pts(:,1) = C2;

        X = pinv(P2)*pts2d(:,i);
        X = X / X(4);
        pts(:,2) = X(1:3);

        V = C2 - X(1:3);
        V = univec(V);

        f = f2 / (Zc * V);

        plot3(pts(1,:), pts(2,:), pts(3,:), '-', 'Color', 'cyan');
        x = C2 + f*V;

        pts = [C2, x];
        plot3(pts(1,:), pts(2,:), pts(3,:), '-', 'Color', 'cyan');
        plot3(x(1), x(2), x(3),'*', 'Color', 'cyan');
        plot3(x(1), x(2), x(3),'o', 'Color', 'k');
    end


    h = helpdlg('Press OK to perform calibration');
    set(h,'Position',[282       104       196        76]);
    uiwait(h)

    % 3D reconstruction from 2D points:
    shape = load('shape.mat');
    q1 = shape.q1;
    q2 = shape.q2;

    % Fig 2: 2D image for q1
    figure(2); clf
    % axis([-50 50 -50 50]);
    axis equal; hold on; grid on;
    % name the figure window as 'q1'
    set(gcf,'Name','q1')
    % drow the points from q1 on the image
    plot(q1(1,:),q1(2,:),'.', 'Color', 'magenta')

    % Fig 3: 2D image for q2
    figure(3); clf
    % axis([-50 50 -50 50]);
    axis equal; hold on; grid on;
    % name the figure window as 'q2'
    set(gcf,'Name','q2')
    % drow the points from q2 on the image
    plot(q2(1,:),q2(2,:),'.', 'Color', 'cyan')

    % Add the reconstruction of the object in 3D to the display of figure 1, in red according to reconstruct0 function
    figure(1)

    tic
    % Call reconstruct0 to calculate 3D points
    disp("reconstruct0:");
    X = reconstruct0(P1, P2, q1, q2);
    % plot the 3D points in red
    plot3(X(1,:),X(2,:),X(3,:),'.-', 'Color', 'red')

    toc

    h = helpdlg('Press OK to perform calibration');
    set(h,'Position',[282       104       196        76]);
    uiwait(h)

    % Add the reconstruction of the object in 3D to the display of figure 1, in black according to reconstruct1 function
    figure(1)

    tic
    % Call reconstruct1 to calculate 3D points
    disp("reconstruct1:");
    X = reconstruct1(P1, P2, q1, q2);
    % plot the 3D points in red
    plot3(X(1,:),X(2,:),X(3,:),'.-', 'Color', 'black')
    toc

end
