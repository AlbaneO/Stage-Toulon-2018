clear all
close all
clc
global B
B=[14;0];
z=[1,2,3];
tend=18;
t=3;
t=0:0.005:tend-0.005;
u=u(t);
[n,m]=size(u);
v=reshape(u,sqrt(m),sqrt(m));
y=u(1:1830)';
v1=SymReshape(v);
y1=SymReshape(v1);
% for t=0:0.005:tend
%    u(t)
%     v(t)
% end
% 
% u(6)
% v(6)
% AK=AK(z(1),z(2),z(3),B)
%b_mat=b_mat(t,z)
%Db_mat=Db_mat(t,z)
Q=2e-2*eye(3,3)
HG = 3 ;  % Grand-gain theta
Delta=diag([1,1,1])
 % Due to very basic normal form
Qt=HG*inv(Delta)*Q*inv(Delta)
X=[1,2,3,4,5,6,7,8,9,10,11]';
Xhat = X((1:3));
[KPS1]=Kalman_Pred_Step(6,X,HG,Qt)
