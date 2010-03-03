
function p = cal()


distanceFromServoCenterToCloserControlArmBall = 1.475;
distanceFromServoCenterToFartherControlArmBall = 1.926;

servoArmRadiList = [ 0.275; 0.375; 0.475; 0.575; 0.675 ];

bellCrankCenterRadiList = [];
for i = 1:10,
    radius = 1.6 + (i * 0.050);
    if( (cos(45) * radius) < 1.8 ) 
        bellCrankCenterRadiList(i) = radius;
    end
end

%bellCrankCenterRadiList = [1.75]; %we choose 1.75 inches

controlArmRadiList = [0.65, 0.8, 0.95, 1.1, 1.25, 1.4, 1.55, 1.7 ]; 



finalResultIndex = 1;

for p = 1:1:length(bellCrankCenterRadiList),
   
   % This is the radius from the center of the base plate to the pivot
   % point of the bell crank.
   bellCrankCenterRadi =  bellCrankCenterRadiList(p);

   for servoRadiusIndex=1:1:length(servoArmRadiList),
       
       % This is the radius of the ball from the center of the servo
       servoRadius = servoArmRadiList(servoRadiusIndex);
   
       % This is the radius of the first hole of the arm of the bell crank
       % that links to the arm of the servo
       primaryBaseBellCrankRadius = calculatePrimaryBellCrankRadius(bellCrankCenterRadi);
       
       % This is the radius of the hole that is on the same linear axis as
       % the current servo radius hole, this link joint must be at a right
       % angle.
       primaryBellCrankRadius = primaryBaseBellCrankRadius + ((length(servoArmRadiList) - servoRadiusIndex) * 0.1);
       
       % This is the radius of the ball/hole on the bell crank that ties to the
       % control arm.
       secondaryBellCrankRadius = calculateSecondaryBellCrankRadius(bellCrankCenterRadi);
       
       if primaryBellCrankRadius > 0 && secondaryBellCrankRadius > 0 
         
           % This is the linear distance that the ball travels on the
           % server as it rotates 90 degrees
           servoLinearTravelDistance = calculateLinearTravel(servoRadius, 90);
           
           % Given how far the servo moves linearly, this is how many
           % degrees the bell crank will rotate
           bellCrankAngularRotation = calculateAngularRotation(primaryBellCrankRadius, servoLinearTravelDistance);
           
           % This is the linear distance that the secondary ball on the
           % bell crank will travel given the known degrees of rotation.
           bellCrankSecondaryLinearTravel = calculateLinearTravel(secondaryBellCrankRadius, bellCrankAngularRotation);
           
           
           
           for k=1:1:length(controlArmRadiList),
               
               %this is the radius from the center of the control arm
               %rotation to the ball on the control arm.
              controlArmRadius = controlArmRadiList(k);
              
              % This is the number of degrees that the control arm will
              % rotate given the choosen control arm radius and the linear
              % travel of the 2nd ball on the bell crank.
              controlArmAngleRotation = calculateAngularRotation(controlArmRadius, bellCrankSecondaryLinearTravel);
              
              % This is the lengh of the linkage rod going from the 2nd
              % ball on the bell crank to the ball on the control arm.
              linkageRodLength = cosd(45) * bellCrankCenterRadi;
              
              if( controlArmAngleRotation >= 15 && controlArmAngleRotation <= 45 && linkageRodLength >= 1.3 && primaryBellCrankRadius < 0.734 )
                 %This is a useful combonation
                 
                
                 % This is the trigometric ration of A/B for the bell crank
                 % with the control arm.
                 primaryLinkageRatio = (cosd(45) * bellCrankCenterRadi) / (distanceFromServoCenterToFartherControlArmBall - (cosd(45) * bellCrankCenterRadi));

                 betaBellCrankRadius = (primaryLinkageRatio * distanceFromServoCenterToCloserControlArmBall)/((primaryLinkageRatio + 1) * cosd(45));
                 betaBellCrankBallRadius = distanceFromServoCenterToCloserControlArmBall - (cosd(45) * betaBellCrankRadius);

                 betaBellCrankRodLength = cosd(45) * betaBellCrankRadius;
                 
                 
                 servoOunceInchesOfTorque = 128;
                 servoBallForceOunces = servoOunceInchesOfTorque / servoRadius;
                 primaryBellCrankOunceInchesOfTorque = primaryBaseBellCrankRadius * servoBallForceOunces;
                 secondaryBellCrankBallForceOunces = primaryBellCrankOunceInchesOfTorque / secondaryBellCrankRadius;
                 controlArmTorqueOunceInches = secondaryBellCrankBallForceOunces * controlArmRadius;
                 
                 controlArmForceNewtons = controlArmTorqueOunceInches * 0.27801385;
                 
                 
                 finalResultList(finalResultIndex) = struct('bellCrankCenterRadi', bellCrankCenterRadi, 'servoRadius', servoRadius, 'primaryBaseBellCrankRadius', primaryBaseBellCrankRadius, 'primaryBellCrankRadius', primaryBellCrankRadius, 'bellCrankAngularRotation', bellCrankAngularRotation, 'secondaryBellCrankRadius', secondaryBellCrankRadius, 'controlArmRadius', controlArmRadius, 'controlArmAngleRotation', controlArmAngleRotation, 'linkageRodLength', linkageRodLength, 'betaBellCrankRadius', betaBellCrankRadius, 'betaBellCrankBallRadius', betaBellCrankBallRadius, 'betaBellCrankRodLength', betaBellCrankRodLength, 'servoOunceInchesOfTorque', servoOunceInchesOfTorque, 'servoBallForceOunces', servoBallForceOunces, 'primaryBellCrankOunceInchesOfTorque', primaryBellCrankOunceInchesOfTorque, 'secondaryBellCrankBallForceOunces', secondaryBellCrankBallForceOunces, 'controlArmTorqueOunceInches', controlArmTorqueOunceInches, 'controlArmForceNewtons', controlArmForceNewtons)
                 
                 finalResultIndex = finalResultIndex + 1;

              end
           end 
       end
   end
end

for i=1:1:length(finalResultList)
   finalResultList(i) 
end





function r = calculateAngularRotation(radius, linearTravel)
r = acosd((linearTravel^2 - 2*radius^2)/(-2*radius^2));

function r = calculateLinearTravel(radius, degrees)
r = sqrt(2 * radius^2 - 2 * radius^2 * cosd(degrees));


function r = calculatePrimaryBellCrankRadius(bellCrankCenterRadi)
r = cosd(45) * bellCrankCenterRadi - 0.275 - 0.4;


function r = calculateSecondaryBellCrankRadius(bellCrankCenterRadi)
r = 1.89 - cosd(45) * bellCrankCenterRadi;









%CALCULATE_RADI Summary of this function goes here
%   Detailed explanation goes here
