function camera2()
    % Main function

    % Load the data
    data = load('data.mat');
    Q = data.Xo;
    q1 = data.q1;
    q2 = data.q2;

    % Fig 1: 3D world
    figure(1); clf
    axis([-50 50 -50 50 -50 50]);
    % axis equal
    hold on
    grid on
    cameratoolbar('ResetCameraAndSceneLight')
    cameratoolbar('Show')
    cameratoolbar('SetMode','orbit')
    cameratoolbar('SetCoordSys','z')

    % drow the world axes
    plot3([0 20],[0 0],[0 0],'r','linewidth',2)
    plot3([0 0],[0 20],[0 0],'g','linewidth',2)
    plot3([0 0],[0 0],[0 20],'b','linewidth',2)
    % add labels to the axes
    text(20,0,0,'X')
    text(0,20,0,'Y')
    text(0,0,20,'Z')
    text(0,0,0,'W')
    % drow the points from Q on the image
    plot3(Q(1,:),Q(2,:),Q(3,:),'ko')
    plot3(Q(1,:),Q(2,:),Q(3,:),'k*')
    

%     % Fig 2: 2D image for q1
%     figure(2); clf
%     % axis([-50 50 -50 50]);
%     axis equal; hold on; grid on;
%     % name the figure window as 'q1'
%     set(gcf,'Name','q1')
%     % drow the points from q1 on the image
%     plot(q1(1,:),q1(2,:),'r*')
% 
%     % Fig 3: 2D image for q2
%     figure(3); clf
%     % axis([-50 50 -50 50]);
%     axis equal; hold on; grid on;
%     % name the figure window as 'q2'
%     set(gcf,'Name','q2')
%     % drow the points from q2 on the image
%     plot(q2(1,:),q2(2,:),'b*')
    
    
%     % Calculate the projection matrix for q1
%     P1 = CameraCalibration0(q1,Q);
%     % Calculate the projection matrix for q2
%     P2 = CameraCalibration0(q2,Q);

    % normalization: 
    % Calculate the projection matrix for q1
    P01 = CameraCalibration0(q1,Q)
    P02 = CameraCalibration0(q2,Q)
    % Calculate the projection matrix for q1
    P1 = CameraCalibration1(q1,Q)
    P2 = CameraCalibration1(q2,Q)
    
    
   [K1, R1, C1] = DecomposeProjectionMatrix(P1)
   [K2, R2, C2] = DecomposeProjectionMatrix(P2)
   
   figure(1);
   % camera 1
   plot3([C1(1) C1(1)+20*R1(1,3)],[C1(2) C1(2)+20*R1(2,3)],[C1(3) C1(3)+20*R1(3,3)],'r','linewidth',2)
   plot3([C1(1) C1(1)+20*R1(1,2)],[C1(2) C1(2)+20*R1(2,2)],[C1(3) C1(3)+20*R1(3,2)],'g','linewidth',2)
   plot3([C1(1) C1(1)+20*R1(1,1)],[C1(2) C1(2)+20*R1(2,1)],[C1(3) C1(3)+20*R1(3,1)],'b','linewidth',2)
   text(C1(1)+20*R1(1,3),C1(2)+20*R1(2,3),C1(3)+20*R1(3,3),'Z')
   text(C1(1)+20*R1(1,2),C1(2)+20*R1(2,2),C1(3)+20*R1(3,2),'Y')
   text(C1(1)+20*R1(1,1),C1(2)+20*R1(2,1),C1(3)+20*R1(3,1),'X')
   text(C1(1),C1(2),C1(3),'C1')
   
   % camera 2
   plot3([C2(1) C2(1)+20*R2(1,3)],[C2(2) C2(2)+20*R2(2,3)],[C2(3) C2(3)+20*R2(3,3)],'b','linewidth',2)
   plot3([C2(1) C2(1)+20*R2(1,2)],[C2(2) C2(2)+20*R2(2,2)],[C2(3) C2(3)+20*R2(3,2)],'g','linewidth',2)
   plot3([C2(1) C2(1)+20*R2(1,1)],[C2(2) C2(2)+20*R2(2,1)],[C2(3) C2(3)+20*R2(3,1)],'r','linewidth',2)
   text(C2(1)+20*R2(1,3),C2(2)+20*R2(2,3),C2(3)+20*R2(3,3),'Z')
   text(C2(1)+20*R2(1,2),C2(2)+20*R2(2,2),C2(3)+20*R2(3,2),'Y')
   text(C2(1)+20*R2(1,1),C2(2)+20*R2(2,1),C2(3)+20*R2(3,1),'X')
   text(C2(1),C2(2),C2(3),'C2')

       
   % for each point in the first camera
   for i=1:size(q1,2)
       % find the point X
       X = pinv(P1)*[q1(:,i);1];
       % plot the line between C and X
       plot3([C1(1) X(1)],[C1(2) X(2)],[C1(3) X(3)],'r')
   end

    % for each point in the second camera
    for i=1:size(q2,2)
        % find the point X
        X = pinv(P2)*[q2(:,i);1];
        % plot the line between C and X
        plot3([C2(1) X(1)],[C2(2) X(2)],[C2(3) X(3)],'b')
    end

    



