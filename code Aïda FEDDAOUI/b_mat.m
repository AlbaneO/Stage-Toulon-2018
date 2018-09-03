function out=b_mat(t,z)
global B
   
 ak = AK(z(1),z(2),z(3),B);   
    
 out=[v(t)*sin(z(1))/z(2)-u(t);
     -v(t)*cos(z(1));
      v(t)*sin(z(3))/ak-u(t)];
end
