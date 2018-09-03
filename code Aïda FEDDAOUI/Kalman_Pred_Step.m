function [out] = Kalman_Pred_Step(t,X,theta, Qtheta)
% Prediction step of the continuous-discrete asynchronous K filter
%   
%%% Retrieve state / estimation / Riccati equation
    n = 3;
    nRic = n*(n+1)/2;
% --- % 
    Xhat = X((1:n))
    S = SymReshape(X(n+(1:nRic)))   
% ----- %    
    % No Output related Data
% ----- %
    A_m=A_mat(t)
    b_m=b_mat(t,Xhat)
    Db_m=Db_mat(t,Xhat)
    
    dXhat = A_mat(t)* Xhat + b_mat(t,Xhat);%-P*C'*IR_t*C*(Xhat-mes(:,where));                    %etat estime 
% --- %   
    dS =-(A_mat(t)+Db_mat(t,Xhat))'*S-S*(A_mat(t)+Db_mat(t,Xhat))-S*Qtheta*S %matrice de Riccati
% --- %
   out = [dXhat;SymReshape(dS)];
   %out = dXhat;
   %out2 = SymReshape(dS);
end