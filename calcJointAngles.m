%Title: Calculating joint configuration through inverse kinematics
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
%The outputs are q21,q22,q11,q12 joint angles
%The inputs are (point(x),point(y),linklength1,linklength2)
%Point(x,y) define the end effector location
%The outputs are in degrees
%So use functions cosd and sind in future 
%IK stands for Inverses Kinematics in calcIKangles

function [q11,q12,q21,q22]=calcIKangles(x,y,l1,l2)
r = (x^2 + y^2);
D = (r - l1^2 - l2^2)/(2*l1*l2);

q21= rad2deg(atan2(sqrt(1-D^2),D));
q22= rad2deg(atan2(-sqrt(1-D^2),D));
q11=rad2deg(atan2(y,x)-atan2(l2*sind(q21),l1+l2*cosd(q21)));
q12=rad2deg(atan2(y,x)-atan2(l2*sind(q22),l1+l2*cosd(q22)));

end 

