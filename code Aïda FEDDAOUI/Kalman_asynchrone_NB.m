function out = Kalman_asynchrone_NB
%
    clear all
    close all
    clc
    global B
    
    %%%      %%%
    % Couleurs %
    %%%      %%%
    C_trajectory = [0 0 0.5]; % Bleu Navy
    C_rotating_Part = [0 100/255 0]; % Vert Sombre
    C_Boat2Amer =[176/255 196/255  222/255]; % bleu acier clair
    C_amer1 = [255/255,20/255,147/255]; % Rose intense
    C_amer2 = [210/255,105/255,30/255]  ;% Chocolat

    %%%        %%%
    % Parametres %
    %%%        %%%
    %
    %si Display=0: la simulation de la trajectoire du bateau n'apparait pas. 
    %Si Display=1: Affichage de la trajectoire
    Display=0; 

    %%%% 
    %  Options de ode
    options = odeset('RelTol',1e-10,'AbsTol',1e-10,'MaxStep',1e-2);
    
    A=[0;0];     % balise A1
    B=[14; 0];   % balise A2
    tend = 18;
    t=[0:0.005:tend];

    %%%                              %%%
    % X=(x,y,theta) et \dot{X}=AX+B(u) %
    %%%                              %%%
    x0=[1 ; 6; 1];
    [TOUT,XOUT] = ode45(@(t,x) [v(t)*cos(x(3)); v(t)*sin(x(3)); u(t)],t,x0,options);

    %XOUT(:,1) correspond a x(t), XOUT(:,2) correspond a y(t),XOUT(:,3) correspond a theta(t)

    XOUT(:,3) = mod(XOUT(:,3),2*pi);

    for i=1:length(t)
        %mesure de l'angle entre l'orientation du bateau et la balise A
         ya(i,1) = mod(atan2((A(2,1)-XOUT(i,2)),(A(1,1)-XOUT(i,1)))-XOUT(i,3),2*pi);

        %mesure de l'angle entre l'orientation du bateau et la balise B
         yb(i,1) = mod(atan2((B(2,1)-XOUT(i,2)),(B(1,1)-XOUT(i,1)))-XOUT(i,3),2*pi);
    end
            
%%%%% ------------------------------------------------------  %%%%%
% Computation Asynchronous mesurements
%
%
%
%  
        w=10; %vitesse de rotation du capteur angulaire 
        Angle_temp=zeros(length(t),1);
        Angle_temp(:,1)=mod(w*t(1,:),2*pi);
        S1=[];
        S2=[];
        ii=[];
  
        for i=1:length(t)
            if (abs(ya(i,1)-Angle_temp(i,1))<1e-1)
                S1=[S1,t(1,i)];
                ii=[ii,i];
            end
            if (abs(yb(i,1)-Angle_temp(i,1))<1e-1)
                S2=[S2,t(1,i)];
            end
        end
        
        %%% Next - Remove Multi-instances -
        for k=1:length(S1)-1
            if (S1(1,k+1)-S1(1,k)<1e-1)
                    S1(1,k+1)=(S1(1,k)+S1(1,k+1))/2;
                    S1(1,k)=0;
            end
        end
        
        S1;
        temp1=find(S1);
        length(temp1);
        
        for i=1:length(temp1)
            temp2(i)=S1(1,temp1(1,i));
        end
        temp2;
        S1=zeros(1,length(temp2));
        S1=temp2


        
        for k=1:length(S2)-1
            if (S2(1,k+1)-S2(1,k)<1e-1)
                    S2(1,k+1)=(S2(1,k)+S2(1,k+1))/2;
                    S2(1,k)=0;
            end
        end
        temp1=find(S2);
         for i=1:length(temp1)
            temp2(i)=S2(1,temp1(1,i));
        end
        S2=zeros(1,length(temp2));
        S2=temp2
        length(S2);
%                                                                   %
% %   END % Computation Asynchronous mesurements                    %
% %%%%% ------------------------------------------------------  %%%%%   
% 
% 
% 
%%%%% ------------------------------------------------------  %%%%%
% si <Display = 1>
% Pour tracer la trajectoire du bateau "en temps reel+ le capteur qui
% tourne+la droite qui relie le bateau -> la balise.

        if Display
            FigHandle=figure(1)
            for i=1:length(t)
                clf
                hold on
                %%%%%%%
                %axis('square')
                axis([-2,12,-5,12])
                plot([A(1,1), B(1,1)],[A(2,1), B(2,1)],'--','Color','k')
                plot(A(1,1), A(2,1),'o','Color',C_amer1)
                plot(B(1,1), B(2,1),'o','Color',C_amer2)

                plot(XOUT(:,1), XOUT(:,2),'Linewidth',1.5,'Color',C_trajectory)
                plot(XOUT(i,1), XOUT(i,2),'or','Linewidth',3)
               % B_graph = TheBoat(XOUT(i,1),XOUT(i,2),XOUT(i,3));
              %  plot(B_graph(1,:),B_graph(2,:),'k','LineWidth',1.5)

        %       Face a l'amer.        
        %       if or((abs(ya(i,1)-Angle_temp(1,i))<1e-2),(abs(yb(i,1)-Angle_temp(1,i))<1e-2))

               if (abs(ya(i,1)-Angle_temp(i,1))<1e-1)
                    plot([XOUT(i,1),XOUT(i,1)+5*cos(XOUT(i,3)+Angle_temp(i,1))], [XOUT(i,2),XOUT(i,2)+5*sin(XOUT(i,3)+Angle_temp(i,1))],'Color',C_amer1,'LineStyle',':','LineWidth',2)
               elseif (abs(yb(i,1)-Angle_temp(i,1))<1e-1)
                   plot([XOUT(i,1),XOUT(i,1)+5*cos(XOUT(i,3)+Angle_temp(i,1))], [XOUT(i,2),XOUT(i,2)+5*sin(XOUT(i,3)+Angle_temp(i,1))],'Color',C_amer2,'LineStyle',':','LineWidth',2)
               else
                    plot([XOUT(i,1),XOUT(i,1)+5*cos(XOUT(i,3)+w*t(1,i))], [XOUT(i,2),XOUT(i,2)+5*sin(XOUT(i,3)+w*t(1,i))],'Color',C_rotating_Part,'LineStyle',':','LineWidth',2)
               end    

                plot([XOUT(i,1),A(1,1)], [XOUT(i,2),A(2,1)],'Linewidth',1,'Color',C_Boat2Amer) % droites bateau - amer

                plot([XOUT(i,1),XOUT(i,1)+cos(XOUT(i,3))], [XOUT(i,2),XOUT(i,2)+sin(XOUT(i,3))],'k','Linewidth',2)

                plot([XOUT(i,1),XOUT(i,1)+cos(XOUT(i,3)+ya(i))],[XOUT(i,2),XOUT(i,2)+sin(XOUT(i,3)+ya(i))],'Color',C_amer1,'Linewidth',2)
                plot([XOUT(i,1),XOUT(i,1)+cos(XOUT(i,3)+yb(i))],[XOUT(i,2),XOUT(i,2)+sin(XOUT(i,3)+yb(i))],'Color',C_amer2,'Linewidth',2)

                plot([XOUT(i,1),B(1,1)], [XOUT(i,2),B(2,1)],'Linewidth',1,'Color',C_Boat2Amer)

                drawnow()
            end
        end
%                                                                       %        
%                                                                       %
%                                                                       %   
%     %%%%% ------------------------------------------------------  %%%%%
%     
%     %%%%%%%%%%%%%%%%%%
%     %%% Passage en coordonnees polaires:
%     %%% (x,y,theta)--> (alpha1, alpha2, rho1,rho2) 
% 
%     alpha1=zeros(length(t),1);
%     alpha2=zeros(length(t),1);
%     rho1=zeros(length(t),1);
%     rho2=zeros(length(t),1);
%     x1=zeros(length(t),1);
%     x2=zeros(length(t),1);
%     y1=zeros(length(t),1);
%     y2=zeros(length(t),1);
% 
%     alpha1(:,1)=atan2(XOUT(:,2)-A(2,1),XOUT(:,1)-A(1,1));%-pi;
%     rho1(:,1)  =sqrt((XOUT(:,2)-A(2,1)).^2+(XOUT(:,1)-A(1,1)).^2);
%     alpha2(:,1)=atan2(XOUT(:,2)-B(2,1),XOUT(:,1)-B(1,1));%-pi;
%     rho2(:,1)  =sqrt((XOUT(:,2)-B(2,1)).^2+(XOUT(:,1)-B(1,1)).^2);
% 
%     for i=1:length(t)   
%         x1(i,1)=rho1(i,1)*cos(alpha1(i,1));
%         y1(i,1)=rho1(i,1)*sin(alpha1(i,1));
%         x2(i,1)=rho2(i,1)*cos(alpha2(i,1))+B(1,1);
%         y2(i,1)=rho2(i,1)*sin(alpha2(i,1))+B(2,1);
%     %     hold on
%     %     plot(alpha2(1,i)*cos(rho2(1,i)), alpha2(1,i)*sin(rho2(1,i)))
%     %     drawnow
%     end
% 
%     figure(2)
%     %axis('square')
%     axis([-2,14,-5,14])
%     hold on
%     plot([A(1,1), B(1,1)],[A(2,1), B(2,1)],'--','Color','k')
%     plot(A(1,1), A(2,1),'o','Color',C_amer1)
%     plot(B(1,1), B(2,1),'o','Color',C_amer2)
% 
%     plot(XOUT(:,1),XOUT(:,2),'Color',C_trajectory,'LineWidth',2);
%     plot(x1(:,1),y1(:,1),'Color','r','LineWidth',1);
%     plot(x2(:,1),y2(:,1),'Color','y','LineWidth',0.5);
%     legend('','','','Trajectoire -coor. originales','trajectoire -- coor. poliares w.r.t A1','trajectoire -- coor. poliares w.r.t A2')
%     
%     hold off
%     %%%
%     %%% FIN FIGURE
%     %%%%%%%%%%
%     
%     
%     %%%%%                         %%%%%
%     % % DYNAMIQUE EN COORD POLAIRES % %
%     %
%     %
%     %
%     
%     % Pol_A_0 = coordonnee initiale en repr?sentation polaire w.r.t. A
%     Pol_A_0 = [sqrt((XOUT(1,2)-A(2,1)).^2+(XOUT(1,1)-A(1,1)).^2);
%                 atan2(XOUT(1,2)-A(2,1),XOUT(1,1)-A(1,1));                 
%                 XOUT(1,3)];               %pourquoi pas alpha1=atan2(XOUT(1,2)-A(2,1),XOUT(1,1)-A(1,1))-pi??
% 
% 
%     % x1 = rho1, x2 = alpha1, x3 = theta        
%     [TOUT,POL1OUT] = ode45(@(t,x) [v(t)*cos(x(3)- x(2)); 
%                                    v(t)*sin(x(3)- x(2))/x(1);
%                                          u(t)             ],TOUT,Pol_A_0,options);
% 
%     % Pol_B_0 = coordonnee initiale en repr?sentation polaire w.r.t. A
%     Pol_B_0 = [sqrt((XOUT(1,2)-B(2,1)).^2+(XOUT(1,1)-B(1,1)).^2);
%                 atan2(XOUT(1,2)-B(2,1),XOUT(1,1)-B(1,1));    %idem
%                 XOUT(1,3)];             %pourquoi pas alpha2=atan2(XOUT(1,2)-B(2,1),XOUT(1,1)-B(1,1))-pi ??
% 
%     % x1 = rho2, x2 = alpha2, x3 theta        
%     [TOUT,POL2OUT] = ode45(@(t,x) [v(t)*cos(x(3)- x(2)); 
%                                    v(t)*sin(x(3)- x(2))/x(1);
%                                           u(t)              ],TOUT,Pol_B_0,options);                               
% 
%                                    
%     figure(3)
%     title('trajectoire donnee par la dynamique en coord. polaires')
%     hold on
% %    axis([-2,30,-5,12])
%     plot([A(1,1), B(1,1)],[A(2,1), B(2,1)],'--','Color','k')
%     plot(A(1,1), A(2,1),'o','Color',C_amer1)
%     plot(B(1,1), B(2,1),'o','Color',C_amer2)
%     plot(XOUT(:,1),XOUT(:,2),'Color',C_trajectory,'LineWidth',3);
% 
%     plot(POL1OUT(:,1).*cos(POL1OUT(:,2)),POL1OUT(:,1).*sin(POL1OUT(:,2)),'r','LineWidth',1.5)
% 
%     plot(POL2OUT(:,1).*cos(POL2OUT(:,2))+B(1,1),POL2OUT(:,1).*sin(POL2OUT(:,2))+B(2,1),'g','LineWidth',0.5)
% 
%     hold off
% 
%     Measure1_polA = mod(POL1OUT(:,2)-POL1OUT(:,3)+pi,2*pi); % Xi_1 = alpha1 + pi -Theta
% 
%                                       %
%                                       %
%                                       % 
%     % % DYNAMIQUE EN COORD POLAIRES % %
%     %%%%%       -------------     %%%%%
%    
%      
%     %%%%%                 %%%%%
%     % CALCUL DES SORTIES
%     %
%     %
%     % 
%     theta = XOUT(:,3);
%     mes_1 = pi - theta+alpha1 ;
%     mes_2 = rho1 ;
%     mes_3 = pi - theta+alpha2 ;
%         
%     %%%%%%                           %%%
%     % Mesures d'angle sans modulo 2*pi %
%     mes_1_V2 = mes_1; 
%     indexes = find(diff(mes_1)<-0.5);
%     if length(indexes)>0
%         for i = 1:length(indexes)
%             mes_1_V2(indexes(i)+1:end)=(2*pi)+mes_1_V2(indexes(i)+1:end);
%         end
%     end
%     
%     indexes = find(diff(mes_1_V2)>0.5);
%     if length(indexes)>0
%         for i = 1:length(indexes)
%             mes_1_V2(indexes(i)+1:end)=-(2*pi)+mes_1_V2(indexes(i)+1:end);
%         end
%     end
%     
%     
%     mes_3_V2 = mes_3; 
%     indexes = find(diff(mes_3)<-0.5);
%     if length(indexes)>0
%         for i = 1:length(indexes)
%             mes_3_V2(indexes(i)+1:end)=(2*pi)+mes_3_V2(indexes(i)+1:end);
%         end
%     end
%     
%     indexes = find(diff(mes_3_V2)>0.5);
%     if length(indexes)>0
%         for i = 1:length(indexes)
%             mes_3_V2(indexes(i)+1:end)=-(2*pi)+mes_3_V2(indexes(i)+1:end);
%         end
%     end
%    
%     
%     Measures = [mes_1_V2';
%                 mes_2';
%                 mes_3_V2'];    % Format : [mes_1;mes_2;mes_3]
%   
%                 
%   Noisy = 1 ;
%   St_dev1 =0.1 ;
%   St_dev2 =1 ; 
%   St_dev3 =0.1 ;
%     
%   AA = [0.2; 
%         0.4; 
%         0.3];
%     % 0 : kills the noise
%     % 1: keeps the noise
%     
%     
%   if Noisy 
%         Noise =randn(size(Measures));
%       for i = 1:3
%         name = ['St_dev',num2str(i)];
%         Noise(i,:)=Noise(i,:)*eval(name); 
%       end
%       
%       
%       for i = 2 : length(Noise) 
%         Noise(:,i) = (ones(3,1)-AA).*Noise(:,i-1)+AA.*Noise(:,i);
%         % Noise i <- nouveau signal
%         % Noise i-1 <- ancien signal
%       end
%       
%       Measures = Measures + Noise;
%       figure(44)
%      
%       
%       for i = 1: 3
%           subplot(3,1,i); plot(TOUT,Measures(i,:));
%       end
%       
%   end
%             
%                                       %
%                                       %
%                                       %
%     %%%%%%%        --------         %%%
%     
%     
%     %%%%%%%%%%%%%%%%%%%%%%          %%%
%     %%% Forme normale d'observabilite %
%     %
%     %
%     %
%     zed=zeros(length(t),3);
%     zed(:,1)= pi-XOUT(:,3)+alpha1(:,1);
%     zed(:,2)= rho1 ;
%     zed(:,3)=pi-XOUT(:,3)+alpha2(:,1);
% 
%     %%%% ???    phi_exact(:,1)=(sin(z(:,1)).*z(:,4)).^2+(sin(z(:,3)).*z(:,2)).^2-2*sin(z(:,1)).*sin(z(:,3)).*cos(abs(z(:,1)-z(:,3))).*z(:,2).*z(:,4)-(B(1)*z(:,2).*z(:,4)).^2;
% 
%     %%%                %%% 
%     % Dynamique Normale  % 
%     %%%                %%%
%     [TOUT,NORM_OUT] = ode45(@(t,z) [v(t)*sin(z(1))/z(2)-u(t);
%                                 -v(t)*cos(z(1));
%                                 v(t)*sin(z(3))/AK(z(1),z(2),z(3),B)-u(t)],TOUT,zed(1,:),options);
% 
% 
%     figure(4)
%     title('Simulation forme normale VS changement de variables');
%     % --- %
%     subplot(3,1,1)
%     plot(TOUT,mod(zed(:,1),2*pi),'k')
%     hold on
%     plot(TOUT,mod(NORM_OUT(:,1),2*pi),'r')
%     hold off
%     % --- %
%     subplot(3,1,2)
%     plot(TOUT,zed(:,2),'k')
%     hold on
%     plot(TOUT,NORM_OUT(:,2),'r')
%     hold off
%     % --- %
%     subplot(3,1,3)
%     plot(TOUT,mod(zed(:,3),2*pi),'k')
%     hold on
%     plot(TOUT,mod(NORM_OUT(:,3),2*pi),'r')
%     hold off
% 
%                                     
% %    figure(4)
% %    plot(phi_exact)
% %    title('\phi=\rho_1^2+\rho_2^2-2*\rho_1*\rho_2cos(\xi)-xb avec z exact' )
%     % % % for i=1:length(t)
%     % % %     z(i,1)=alpha1(1,i)-XOUT(i,3);
%     % % %     z(i,2)=sin(XOUT(i,3)-alpha1(1,i))/rho1(1,i);
%     % % %     z(i,3)=alpha2(1,i)-XOUT(i,3);
%     % % %     z(i,4)=sin(XOUT(i,3)-alpha2(1,i))/rho2(1,i);
%     % % % end
% 
% 
%     
%     %%% ------------------------------------ %%%
%     %%% changement de variable inverse.
%     %
%     %
%     %
%     %
%     %
%     
%     z = NORM_OUT;
%     
%     %%% Assurons nous qu'il est possible de reconstituer (x,y,theta) a partir de
%     %%% (z1, z2, z3,z4)
% 
%     %%% A un instant t donne, le bateau est sur une courbe de niveau (un arc de
%     %%% cercle circonstrit au triangle A-B-Bateau. Le centre C de ce cercle est
%     %%% le point de concours des mediatrices de ce triangle et peut donc etre
%     %%% calcule a la main. 
% 
%     %%% En utilisant cela ainsi que le fait que rho1 et rho2 soient les rayons
%     %%% de cercles centres resp. en A et B et de rayon A-Bateau, resp.
%     %%% B-Bateau, on peut trouver une expression pour y et donc pour x et
%     %%% theta. On exploite notamment la connaissance de yA=theta-alpha1,
%     %%% yB=theta-alpha2 et rho1=sin(z1)/z2 (idem pour rho2)
% 
%     %diff(rho1,t)=v*cos(theta-alpha1)=v*cos(z1) (idem pour rho2)
% 
%     xi=zeros(length(t),1);
%     C=zeros(length(t),2);
%     R=zeros(length(t),1);
%     r1=zeros(length(t),1);      %calcul de rho1 a partir de z1 et z2
%     r2=zeros(length(t),1);      %calcul de rho1 a partir de z3 et z4
%     y=zeros(length(t),1);
%     x=zeros(length(t),1);
%     a1=zeros(length(t),1);      %calcul de alpha1 et alpha2 a partir de x et y
%     a2=zeros(length(t),1);
%     tht=zeros(length(t),1);     %calcul de theta a partir de z1 et alpha1 (ou z3 et alpha2)
% 
%     xi(:,1)=abs(z(:,3)-z(:,1));
%     C(:,1)=(B(2,1)-A(2,1)+tan(xi(:,1)).*(A(1,1)+B(1,1)))./(2*tan(xi(:,1))); %pourquoi C(1) est il constant?
%     C(:,2)=(B(1,1)-A(1,1)+tan(xi(:,1)).*(A(2,1)+B(2,1)))./(2*tan(xi(:,1)));
% 
%     for i = 1:length(C)
%        Ray(i,1) = sqrt(C(i,:)*C(i,:)');
%     end
%     %  Ray(:,1)=sqrt((B(1,1)-A(1,1))^2+(B(2,1)-A(2,1))^2)./(2*sin(xi(:,1)));
%     %r1(:,1)=sin(pi-z(:,1))./z(:,2);           
%     r1 = z(:,2);
% %    r2(:,1)=sin(pi-z(:,3))./z(:,4);        
%     for i=1:length(r1)
%         r2(i,1) = AK(z(i,1),z(i,2),z(i,3),B);
%     end
% 
% 
%     yNUM1 = (C(:,1).*(r2(:,1).^2-r1(:,1).^2)+B(1,1).*r1(:,1).^2);
% 
%     yDEN1=(2*(B(1,1).*C(:,2)-C(:,1)*B(2,1)));
% 
%     yNUM2 = (B(1,1).*C(:,1).*(C(:,1)-B(1,1))-B(1,1).*Ray(:,1).^2+B(1,1).*C(:,2).^2-C(:,1)*B(2,1)^2);
% 
%     yDEN2=(2*(B(1,1).*C(:,2)-C(:,1)*B(2,1)));
% 
%     y =yNUM1./yDEN1 + yNUM2./yDEN2;
% 
%     x(:,1)=sqrt(r1(:,1).^2-y(:,1).^2);
%     a1(:,1)=mod(atan2(A(2,1)-y(:,1),A(1,1)-x(:,1))-pi,2*pi);
%     a2(:,1)=mod(atan2(B(2,1)-y(:,1),B(1,1)-x(:,1))-pi,2*pi);
%     tht(:,1)=pi-z(:,1)+a1(:,1);
% 
% 
%     figure(5)
%     title('Trajectoire reconstruite a partir de la forme normale')
%     hold on
%     plot(x(:,1),y(:,1),'r')
%     plot(XOUT(:,1),XOUT(:,2),'Color',C_trajectory)
%     hold off
% 
%     %%% Calcul erreur 
%     for i=1:length(t)
%     erreur_rho(i,1)=r1(i,1)-rho1(i,1);
%     erreur_alpha1(i,1)=alpha1(i,1)-a1(i,1);
%     erreur_alpha2(i,1)=alpha2(i,1)-a2(i,1);
% 
%     erreur_x(i,1)=XOUT(i,1)-x(i,1);
%     erreur_y(i,1)=XOUT(i,2)-y(i,1);
%     erreur_theta(i,1)=XOUT(i,3)-tht(i,1);
%     end
% 
%     figure(6)
%     hold on
%     title('erreur de reconstruction')
%     plot(erreur_alpha2)
%     plot(erreur_alpha1)
%     plot(erreur_rho )
%     plot(erreur_theta)
%     %plot(erreur_x)
%     %plot(erreur_y)
%     hold off
%                                                %
%                                                %
%                                                %
%                                                %
%                                                %
%     %%% changement de variable inverse.        %
%     %%% ------------------------------------ %%%
%     
%     
%     
%     %%%%%                ---------------------               %%%%
%     %  Asynchronous continuous-discrete High-Gain Kalman Filter %
%     %                                                           %
%     %%%%%                ---------------------               %%%%
%     
    
     TimeStamps = union(S1,S2);
     if TimeStamps(1)==S1(1)
         FirstIndex = 1; 
     elseif TimeStamps(1)==S2(1)
         FirstIndex = 2;
     else
         error('Problem With first index');
     end
%    
%     %%%%                          %%%
%     % Test d'alternance des indices %
%     %
%     %
%     
     First_indexes  = 1:2:length(TimeStamps);
     Second_indexes = 2:2:length(TimeStamps);
%     
%     if FirstIndex == 1
%         if min(S1==TimeStamps(First_indexes))&&min(S2==TimeStamps(Second_indexes))
%             disp('Alternance OK');
%         else
%             disp('Alternance NOT OK');
%         end
%     else
%         if min(S1==TimeStamps(Second_indexes))&&min(S2==TimeStamps(First_indexes))
%             disp('Alternance OK');
%         else
%             disp('Alternance NOT OK');
%         end
%     end
%     
%                                     %
%                                     %   
%     % Test d'alternance des indices %
%     %%%%                          %%%
%     
%     
%     %%%%%%%%% -------------------------
%     %%% Parametres de l'observateur    
%     %
%     %
%     %
%     %
%     %%% --------------
%     % HIGH-GAIN PARAMETER
%     %
%     %
%     HG = 3 ;  % Grand-gain theta
%     Delta=diag([1,1,1]); % Due to very basic normal form
%         
%     Q = 1e-2*diag([1,10,1]);   % ???
% 
% %    Q=2e-2*eye(3,3);
% 
%     Qtheta=HG*inv(Delta)*Q*inv(Delta);
%     
%     R = diag([0.1,1,0.1]);
%     
% %    R = 1e1*eye(3,3);
%     
% 
%     % -----
%     % Initial Guess
%  %   xhat0 = [0.7; 6.3;1.3]; % Initial Guess In original coodinates
%  %    ya = mod(atan2((A(2,1)-xhat0(2)),(A(1,1)-xhat0(1)))-xhat0(3),2*pi);
% 
%     %mesure de l'angle entre l'orientation du bateau et la balise B
%  %   yb = mod(atan2((B(2,1)-xhat0(2)),(B(1,1)-xhat0(1)))-xhat0(3),2*pi);  
%  %    RHO1 = sqrt(0.7^2+6.3^2);   
%  %    xhat0 =[ya;RHO1;yb];
%       
% 
%     xhat0=zed(1,:)'+[1;-1;-0.5];   % Initial Guess in Normal coordinates
%     
%     % Riccati Initial State 
%     As = (A_mat(0)+Db_mat(0,xhat0));
%     P0=care(As',eye(3,3),Qtheta,R,zeros(3,3),eye(3,3)); % cf. cahier de Nico
%     
%     %S0=diag([1,1,1]);       
%     S0 = inv(P0)
%     
%     S0=SymReshape(S0);
%     %
%     %
%     %%%
%                                       %
%                                       %
%     % Parametres de l'observateur     %
%     %%%%%%%%% -------------------------    
% 
%     
%     
%     sensor = FirstIndex;
%     Big_TOUT = [];
%     KALOUT2 = []
%     for i=1:length(TimeStamps)
%         if i==1 
%             TSPAN = [0 TimeStamps(1)];
%             [TOUT2,KALOUT] = ode45(@(t,Z) Kalman_Pred_Step(t,Z,HG, Qtheta),TSPAN,[xhat0;S0],options);
%             KALOUT2 = [KALOUT2;KALOUT(1:end,:)];
%             Big_TOUT = [Big_TOUT;TOUT2(1:end,:)];
%            
%         else
%             TSPAN = [TimeStamps(i-1) TimeStamps(i)];
%             [TOUT2,KALOUT] = ode45(@(t,Z) Kalman_Pred_Step(t,Z,HG, Qtheta),TSPAN,KALOUT2(end,:),options);
%             Big_TOUT = [Big_TOUT;TOUT2(2:end,:)];
%             KALOUT2 = [KALOUT2;KALOUT(2:end,:)];
%         end
%         
%         if or(i == 1,i == 2)
%             DT = TimeStamps(i);
%         else
%             DT = TimeStamps(i)-TimeStamps(i-2);
%         end
%         
%         %%%             ----------                   %%%
%         % In order to know which measure is considered %
%            Where = max(find(TOUT<=TimeStamps(i)));      %
%         %%%             ----------                   %%%
%         
%         if FirstIndex==1
%            % -- MODEL RELATED -%
%             C = [1 0 0; 
%                  0 1 0 ];
% 
%             Red = R(1:2,1:2);
%             
%             % -- HIGH-GAIN CONSTRUCTION -%  
%             
%             IR_t=HG*inv(Red);
%            
%             % -- Measures -- %
%             Output_Signal = Measures(1:2,Where);
%             
%             % ----- %
%             FirstIndex= mod(FirstIndex,2)+1;
%         else
%              % -- MODEL RELATED -%
%             C = [0 0 1];
% 
%             Red = R(3,3);
%             
%             % -- HIGH-GAIN CONSTRUCTION -%  
%             
%             IR_t=HG*inv(Red);
% 
%             % -- Measures -- %
%             Output_Signal = Measures(3,Where);
%             
%             % ----- %
%             FirstIndex= mod(FirstIndex,2)+1;
%         end
%         
%         %%% Some vectors need to be transposes in order to meet the format
%         %%% of KALOUT2.
%         
%         Zminus = KALOUT2(end,1:3)';
%         Sminus = SymReshape(KALOUT2(end,4:end)');
%         
%         Splus = Sminus + C'*IR_t*C*DT;
%          
%         Zplus = Zminus-inv(Splus)*C'*IR_t*(C*Zminus-Output_Signal)*DT;
%         
%         KALOUT2(end,1:3) = Zplus';
%         KALOUT2(end,4:end) = SymReshape(Splus)';          
%     end
%     
%        
%     figure(7) % Convergence dans les coordonnees normales
%     
%     %title('Simulation forme normale VS changement de variables');
%     % --- %
%     subplot(3,1,1)
%     plot(TOUT,mod(NORM_OUT(:,1),2*pi),'k')
%     hold on
%     plot(TOUT,mod(zed(:,1),2*pi),'b')
%     plot(Big_TOUT,mod(KALOUT2(:,1),2*pi),'r')
%     hold off
%     legend({'$z_1$','$\hat z_1$'},'Interpreter','latex')
%     
%     % --- %
%     subplot(3,1,2)
%     plot(TOUT,NORM_OUT(:,2),'k')
%     hold on
%     plot(TOUT,zed(:,2),'b')
%     plot(Big_TOUT,KALOUT2(:,2),'r')
%     hold off
%     legend({'$z_2$','$\hat z_2$'},'Interpreter','latex')
%     
%     % --- %
%     subplot(3,1,3)
%     plot(TOUT,mod(NORM_OUT(:,3),2*pi),'k')
%     hold on
%     plot(TOUT,mod(zed(:,3),2*pi),'b')
%     plot(Big_TOUT,mod(KALOUT2(:,3),2*pi),'r')
%     hold off
%     legend({'$z_3$','$\hat z_3$'},'Interpreter','latex')
%     
% 
%     
%     
%     %%% ------------------------------------ %%%
%     %%% changement de variable inverse (de l'etat estime)
%     %
%     %
%     %
%     %
%     %
%     
%     z = KALOUT2(:,1:3);
%     
%     %%% Assurons nous qu'il est possible de reconstituer (x,y,theta) a partir de
%     %%% (z1, z2, z3,z4)
% 
%     %%% A un instant t donne, le bateau est sur une courbe de niveau (un arc de
%     %%% cercle circonstrit au triangle A-B-Bateau. Le centre C de ce cercle est
%     %%% le point de concours des mediatrices de ce triangle et peut donc etre
%     %%% calcule a la main. 
% 
%     %%% En utilisant cela ainsi que le fait que rho1 et rho2 soient les rayons
%     %%% de cercles centres resp. en A et B et de rayon A-Bateau, resp.
%     %%% B-Bateau, on peut trouver une expression pour y et donc pour x et
%     %%% theta. On exploite notamment la connaissance de yA=theta-alpha1,
%     %%% yB=theta-alpha2 et rho1=sin(z1)/z2 (idem pour rho2)
% 
%     %diff(rho1,t)=v*cos(theta-alpha1)=v*cos(z1) (idem pour rho2)
% 
%     
%     l = length(KALOUT2);
%     
%     xi=zeros(l,1);
%     C=zeros(l,2);
%     R=zeros(l,1);
%     r1=zeros(l,1);      %calcul de rho1 a partir de z1 et z2
%     r2=zeros(l,1);      %calcul de rho1 a partir de z3 et z4
%     y=zeros(l,1);
%     x=zeros(l,1);
%     a1=zeros(l,1);      %calcul de alpha1 et alpha2 a partir de x et y
%     a2=zeros(l,1);
%     tht=zeros(l,1);     %calcul de theta a partir de z1 et alpha1 (ou z3 et alpha2)
% 
%     xi(:,1)=abs(z(:,3)-z(:,1));
%     C(:,1)=(B(2,1)-A(2,1)+tan(xi(:,1)).*(A(1,1)+B(1,1)))./(2*tan(xi(:,1))); %pourquoi C(1) est il constant?
%     C(:,2)=(B(1,1)-A(1,1)+tan(xi(:,1)).*(A(2,1)+B(2,1)))./(2*tan(xi(:,1)));
% 
%     for i = 1:length(C)
%        Ray(i,1) = sqrt(C(i,:)*C(i,:)');
%     end
%     %  Ray(:,1)=sqrt((B(1,1)-A(1,1))^2+(B(2,1)-A(2,1))^2)./(2*sin(xi(:,1)));
%     %r1(:,1)=sin(pi-z(:,1))./z(:,2);           
%     r1 = z(:,2);
% %    r2(:,1)=sin(pi-z(:,3))./z(:,4);        
%     for i=1:length(r1)
%         r2(i,1) = AK(z(i,1),z(i,2),z(i,3),B);
%     end
% 
% 
%     yNUM1 = (C(:,1).*(r2(:,1).^2-r1(:,1).^2)+B(1,1).*r1(:,1).^2);
% 
%     yDEN1=(2*(B(1,1).*C(:,2)-C(:,1)*B(2,1)));
% 
%     yNUM2 = (B(1,1).*C(:,1).*(C(:,1)-B(1,1))-B(1,1).*Ray(:,1).^2+B(1,1).*C(:,2).^2-C(:,1)*B(2,1)^2);
% 
%     yDEN2=(2*(B(1,1).*C(:,2)-C(:,1)*B(2,1)));
% 
%     y =yNUM1./yDEN1 + yNUM2./yDEN2;
% 
%     x(:,1)=sqrt(r1(:,1).^2-y(:,1).^2);
%     a1(:,1)=mod(atan2(A(2,1)-y(:,1),A(1,1)-x(:,1))-pi,2*pi);
%     a2(:,1)=mod(atan2(B(2,1)-y(:,1),B(1,1)-x(:,1))-pi,2*pi);
%     tht(:,1)=pi-z(:,1)+a1(:,1);
% 
% 
%     figure(8)
% %    title('Trajectoire estimee par l''observateur')
%     hold on
%     plot(x(:,1),y(:,1),'r')
%     plot(XOUT(:,1),XOUT(:,2),'Color',C_trajectory)
%     hold off
% 
%     legend('Estimated state','Actual Trajectory')
%                                                %
%                                                %                                               
%                                                %
%                                                %
%     %%% changement de variable inverse.        %
%     %%% ------------------------------------ %%%    
%     
% 
end


% ------------------------------------------------------------------------ %  
% ------------------------------------------------------------------------ %
% ------------------------------------------------------------------------ %
% ------------------------------------------------------------------------ %
%%%%%
%%% CONTROLES
%%%%%

function out = v(t)
    out = 2 + (t>5)*(t<10)  ;
end

function out = u(t)
   out = -0.22.*t+(0.44.*t).*(t>5)-(0.5.*(t-5)).*(t>9)+(0.5.*(t-13)).*(t>13)+(0.2.*(t-8)).*(t>15) ;
end

%%%%%
%%% CALCUL DE RHO_2
%%%%%

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

%%%%%
%%% KALMAN ETAPE DE PREDICTION
%%%%%


function out = Kalman_Pred_Step(t,X,theta, Qtheta)
% Prediction step of the continuous-discrete asynchronous K filter
%   
%%% Retrieve state / estimation / Riccati equation
    n = 3;
    nRic = n*(n+1)/2;
% --- % 
    Xhat = X((1:n));
    S = SymReshape(X(n+(1:nRic)));    
% ----- %    
    % No Output related Data
% ----- %
    dXhat = A_mat(t)* Xhat + b_mat(t,Xhat);%-P*C'*IR_t*C*(Xhat-mes(:,where));                    %etat estime 
% --- %   
    dS =-(A_mat(t)+Db_mat(t,Xhat))'*S-S*(A_mat(t)+Db_mat(t,Xhat))'-S*Qtheta*S; %matrice de Riccati
% --- %
    out = [dXhat;SymReshape(dS)];
end

%
%
%%%%%
%%% MODELE SOUS FORME NORMALE
%%%%%
%
%

function out = A_mat(t)
out = [0           0          0; 
       0           0          0;
       0           0          0];
end

function out=b_mat(t,z)
global B
   
 AK = AK(z(1),z(2),z(3),B);   
    
 out=[v(t)*sin(z(1))/z(2)-u(t);
     -v(t)*cos(z(1));
      v(t)*sin(z(3))/AK-u(t)];
end


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

