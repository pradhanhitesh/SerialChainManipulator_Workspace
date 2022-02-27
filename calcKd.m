%Title: Calculating Kappa_D for K_Q matrix
%Author: Hitesh Pradhan
%Institute: Indian Institute of Technology, Gandhinagar, India
%Course: ES 656 Human Robot Interaction, Spring Semester 2022
%Date: 26-02-2022
%
%
%
%
%
%
%
function [Kappa_D]=calcKappa_D(x,y,T,p)
l1=0.52; l2=0.48;



[q11,~,q21,~]=calcIKangles(x,y,l1,l2);
r1=[l1*p*cosd(q11) l1*p*sind(q11)]; %position vector for link 1
r2=[l1*cosd(q11)+(l2*p*cosd(q11+q21)) l1*sind(q11)+(l2*p*sind(q11+q21))];%position vector for link 2 
r3=[l1*cosd(q11)+(l2*p*cosd(q11+q21)) l1*sind(q11)+(l2*p*sind(q11+q21))];%position vector for link 3

OM1=[0.5 0]; %vector array for motor1
OM2=[1 0];   %vector array for motor2
OM3=[-1 0];  %vector array for motor3


l1_cap = (OM1-r1)/(sqrt(sum((OM1-r1).^2))); %unit vector for cable 1
l2_cap = (OM2-r2)/(sqrt(sum((OM2-r2).^2))); %unit vector for cable 2
l3_cap = (OM3-r3)/(sqrt(sum((OM3-r3).^2))); %unit vector for cable 3

eq1=[-l1*p*cosd(q11) -l1*p*sind(q11)];
eq2=[-l1*cosd(q11)-l2*066*cosd(q11+q21) -l1*sind(q11)-l2*066*sind(q11+q21)];
eq3=[-l1*cosd(q11)-l2*066*cosd(q11+q21) -l1*sind(q11)-l2*066*sind(q11+q21)];
eq4=[0 0];
eq5=[-l2*p*sind(q11+q21) -l2*p*sind(q11+q21)];
eq6=[-l2*p*sind(q11+q21) -l2*p-sind(q11+q21)];

eq7=[0 0];
eq8=[-l2*p*cosd(q11+q21) -l2*p*sind(q11+q21)];
eq9=[-l2*p*cosd(q11+q21) -l2*p*sind(q11+q21)];
eq10=[0 0];
eq11=[-l2*p*cosd(q11+q21) -l2*p*sind(q11+q21)];
eq12=[-l2*p*cosd(q11+q21) -l2*p*sind(q11+q21)];

pd_Afinal_q11 = [dot(l1_cap,eq1) dot(l2_cap,eq2) dot(l3_cap,eq3); 
                    dot(l1_cap,eq4) dot(l2_cap,eq5) dot(l2_cap,eq6)];
pd_Afinal_q21 = [dot(l1_cap,eq7) dot(l2_cap,eq8) dot(l2_cap,eq9);
                    dot(l1_cap,eq10) dot(l2_cap,eq11) dot(l2_cap,eq12)];
dA_T = [pd_Afinal_q11*T pd_Afinal_q21*T];
Kappa_D = dA_T;

function [q11,q12,q21,q22]=calcIKangles(x,y,l1,l2)
r = (x^2 + y^2);
D = (r - l1^2 - l2^2)/(2*l1*l2);

q21= rad2deg(atan2(real(sqrt(1-D^2)),D));
q22= rad2deg(atan2(real(-sqrt(1-D^2)),D));
q11=rad2deg(atan2(y,x)-atan2(l2*sind(q21),l1+l2*cosd(q21)));
q12=rad2deg(atan2(y,x)-atan2(l2*sind(q22),l1+l2*cosd(q22)));

end 

end