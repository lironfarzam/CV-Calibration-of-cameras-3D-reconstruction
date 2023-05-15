function Check2DNormalizationTransform
   close all
   home
   
   disp('the original point (blue)');
   n=1000; % number of points
   X = randn(2,n)*15 + 119; % get some random points
   figure(1)
   plot(X(1,:),X(2,:),'.');
   axis([-50 250 -50 250]);
   axis square
   pause
   
   disp('the transformation matrix')
   T = Get2DNormalizationTransform(X) %#ok<NOPRT>
   Y = T*[X ; ones(1,n)];
   pause
   
   disp('the transformed points (red) should have zero mean and sqrt(2) average distance')
   disp('check the figure')
   hold on;
   plot(Y(1,:),Y(2,:),'.');
   axis([-50 250 -50 250]);
   disp(' ')
   pause
   
   disp('applying the transform again should result with the identity matrix')
   Ty = Get2DNormalizationTransform(Y(1:2,:)) %#ok<NASGU,NOPRT>
end