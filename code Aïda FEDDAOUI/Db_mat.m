function out=Db_mat(t,z)

global B

    xb=B(2);
    xi=z(3)-z(1);

    den = AK(z(1),z(2),z(3),B);

    AA=-v(t)*sin(z(3))*(z(2)*sin(xi)+z(2)^2*sin(xi)*cos(xi)/sqrt(abs(xb^2-(z(2)*sin(xi))^2)))/den^2;

    BB=-v(t)*sin(z(3))*(cos(xi)-z(2)*sin(xi)^2/sqrt(abs(xb^2-(z(2)*sin(xi))^2)))/den^2;
    
    C1=v(t)*cos(z(3))/den ;
    C2=v(t)*sin(z(3))*(z(2)*sin(xi)+z(2)^2*cos(xi)*sin(xi)/sqrt(abs(xb^2-(z(2)*sin(xi))^2)))/den^2;
    
    CC=C1+C2;

    
    out= [v(t)*cos(z(1))/z(2)   -v(t)*sin(z(1))/(z(2)^2)    0  ;
          v(t)*sin(z(1))         0                          0  ;
          AA                     BB                         CC ];
end