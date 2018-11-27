% The main script to simulate the trajectory of a three-arm manipulator.
% Final version completed in Octobebr 2017
% Written by Avinash  Soor
% Git: Avinasho
% Written for the MEng Individual Project

function J = IK_Jacobian_func(q_curr, L)
       
L1 = L(1);
L2 = L(2);
L3 = L(3);

theta1 = q_curr(1);
theta2 = q_curr(2);
theta3 = q_curr(3);
    
J = [-L1*sin(theta1)-L2*sin(theta1+theta2)-L3*sin(theta1+theta2+theta3) -L2*sin(theta1+theta2)-L3*sin(theta1+theta2+theta3) -L3*sin(theta1+theta2+theta3);
L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3) L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3) L3*cos(theta1+theta2+theta3);
      1 1 1];

end
