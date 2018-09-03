function out = AK(z1,z2,z3,B)
% Determination de rho2 en fonction de la distance AB, de l'angle xsi et de
% rho1
%
% formule obtenue ? patir de la formule d'Al-Kashi, valable uniquement
% lorsque x \in [xA;xB]
%
    radicande = (B(1,1)^2-(z2*sin(z3-z1))^2)*(B(1,1)^2-(z2*sin(z3-z1))^2>0);
    
    out = z2*cos(z3-z1)+sqrt(radicande);
end