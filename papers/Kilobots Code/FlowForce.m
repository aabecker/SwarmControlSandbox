
function FlowForce 

%Flow Around Variables
eta=50;
zeta=1;
rhoNot=7.5;
alphaWant=0;

 alphaWant=atan2(movesX(indOY,indOX),movesY(indOY,indOX));

    repPointX=ObjectCentroidX/scale;
    repPointY=ObjectCentroidY/scale;

    theta = atan2((M(1,2)/scale - repPointY),(M(1,1)/scale - repPointX));
    angdiff = alphaWant-theta;
    rho=sqrt((M(1,1)/scale-repPointX)^2 + (M(1,2)/scale-repPointY)^2);
    if(angdiff > pi) 
        angdiff = angdiff - 2*pi;
    end
    if(angdiff < -pi) 
        angdiff = angdiff + 2*pi;
    end
    
    if ((rho<rhoNot) && (abs(angdiff)<pi*4/8))
        epsilon = 0.2 * scale;

        %disp('In Flow Control Goal System')
        FrepX=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(repPointX-M(1,1)/scale);
        FrepY=eta*((rho^(-1))-(rhoNot^(-1)))*(rho^(-1))^2*(repPointY-M(1,2)/scale);
        
        attPointX=ObjectCentroidX/scale - ObjectRadius/scale * cos(alphaWant);
        attPointY=ObjectCentroidY/scale - ObjectRadius/scale * sin(alphaWant);
        rho=sqrt((M(1,1)/scale-attPointX)^2 + (M(1,2)/scale-attPointY)^2);
        FattX=zeta*(M(1,1)/scale-attPointX)/rho;
        FattY=zeta*(M(1,2)/scale-attPointY)/rho;
        
         currgoalX=M(1,1)/scale*scale+cos(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;
         currgoalY=M(1,2)/scale*scale+sin(atan2((-FrepY-FattY),(-FattX-FrepX)))*scale;
             else 
%% Old Goal Algorithm 
epsilon = 1*scale;
        if M(1,1) > ObjectCentroidX- minDistance && M(1,1) < ObjectCentroidX+minDistance && M(1,2) < ObjectCentroidY+minDistance && M(1,2) > ObjectCentroidY-minDistance
            r = 0.1;
     %else if M(1,1) > ObjectCentroidX- (corners(corInd,1))*scale && M(1,1) < ObjectCentroidX+minDistance && M(1,2) < ObjectCentroidY+minDistance && M(1,2) > ObjectCentroidY-minDistance
     %        r = 2.5;
        else
            r = 2.5;
        end
      
        currgoalX = ObjectCentroidX - r*scale * movesY(indOY,indOX);
        currgoalY = ObjectCentroidY - r*scale * movesX(indOY,indOX);
    end