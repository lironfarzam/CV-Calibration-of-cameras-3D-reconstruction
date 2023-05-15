function Check3DNormalizationTransform
   close all
   home
   
   disp('the original point (blue)');
   n=1000; % number of points
   X = randn(3,n)*15 + 119; % get some random points
   figure(1)
   plot3(X(1,:),X(2,:),X(3,:),'.');
   axis([-50 250 -50 250 -50 250]);
   axis square
   pause
   
   disp('the transformation matrix')
   T = Get3DNormalizationTransform(X) %#ok<NOPRT>
   Y = T*[X ; ones(1,n)];
   pause
   
   
   disp('the transformed points (red) should have zero mean and sqrt(2) average distance')
   disp('check the figure') 
   hold on;
   plot3(Y(1,:),Y(2,:),Y(3,:),'.');
   axis([-50 250 -50 250 -50 250]);
   disp(' ')
   pause
   
   disp('applying the transform again should result with the identity matrix')
   Ty = Get3DNormalizationTransform(Y(1:3,:)) %#ok<NASGU,NOPRT>
end