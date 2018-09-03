function y=SymReshape(u)
% SymReshape     Reshape dedicated to a symetric matrix
%   if u is a symetric matrix, y will be a vector with n(n+1)/2 components
%   obtained from column of u without repeating symetric values.
%   if u is a vector with n(n+1)/2 components, y will be the corresponding
%   symetric matrix.
[m,n]=size(u);
if m==n,
   S=logical(reshape(tril(ones(m,n)),m*n,1));
   y=u(S);
else
   n=(sqrt(1+8*m)-1)/2;
   y=zeros(n,n);
   S=logical(reshape(tril(ones(n,n)),n^2,1));
   y(S)=u;
   y=y';
   y(S)=u;
end