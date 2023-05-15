function u = univec(P)
   % returns in u a unit vector in the direction of P
   % returns NaN if norm(p)==0,
   % the calling function should check for NaN (using isnan(u))
   % and decide what to do with it:
   % either use 0 or do not use the value or issue an error
   s = warning('off', 'MATLAB:divideByZero');
   u = P/norm(P);
   warning(s)      
end