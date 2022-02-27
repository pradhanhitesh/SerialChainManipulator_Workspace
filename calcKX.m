%Title: Calculating end-point stiffness matrix
%Author: Hitesh Pradhan
%Institute: Indian Institute of Technology, Gandhinagar, India
%Course: ES 656 Human Robot Interaction, Spring Semester 2022
%Date: 26-02-2022
%
%
%ONLY RUNS FOR 2 JOINT 1 REDUNDANT SYSTEM
%
%This function calculates the K_X end-point stiffness matrix. Along with
%other matrices:- Kappa_Q and Kappa_F.
%
%The input for the function are P(x,y) desired end point, K_Q matrix that
%is joint stiffness matrix, link lengths and force matrix(2*1).
%The output for the function are K_X end point stiffness matrix, Kappa_Q
%and Kappa_F.
%
function [K_X,Kappa_Q,Kappa_F]=calcK_X(x,y,K_Q,l1,l2,F)


[q11,~,q21,~]=calcIKangles(x,y,l1,l2); %Joint Angles

J=[-l1*sind(q11)-l2*sind(q11+q21) -l2*sind(q11+q21);
    l1*cosd(q11)+l2*cosd(q11+q21) l2*cosd(q11+q21)]; %Jacobian for end position

J_t=transpose(J);

del_Jt_q1=[-l1*cosd(q11)-l2*cosd(q11+q21) -l1*sind(q11)-l2*sind(q11+q21);
            -l2*cosd(q11+q21) -l2*sind(q11+q21)];
del_Jt_q2=[-l2*cosd(q11+q21) -l2*sind(q11+q21);
            -l2*cosd(q11+q21) -l2*sind(q11+q21)];

del_Jt_F=[del_Jt_q1*F del_Jt_q2*F];

Kappa_F=-(inv(J_t)).*del_Jt_F.*inv(J);
Kappa_Q=inv(J_t).*K_Q.*(inv(J));
K_X = Kappa_Q+Kappa_F;


function [q11,q12,q21,q22]=calcIKangles(x,y,l1,l2)
r = (x^2 + y^2);
D = (r - l1^2 - l2^2)/(2*l1*l2);

q21= rad2deg(atan2(real(sqrt(1-D^2)),D));
q22= rad2deg(atan2(real(-sqrt(1-D^2)),D));
q11=rad2deg(atan2(y,x)-atan2(l2*sind(q21),l1+l2*cosd(q21)));
q12=rad2deg(atan2(y,x)-atan2(l2*sind(q22),l1+l2*cosd(q22)));

end 


end
