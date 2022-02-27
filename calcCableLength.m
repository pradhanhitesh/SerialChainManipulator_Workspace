%Title: Calculating cable lengths
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
%CALCULATE YOUR CABLE LENGTH 
%DISTANCE FROM MOTOR POSITION TO ATTACHMENT AT THE LINKS
%DEFINE YOUR MOTOR POSITIONS INSIDE THE FUNCTION (M)
%link1cable is the coordinate of attachment at link1
%link2cable is the coordinate of attachment at link2

function [cable1,cable2,cable3]=calcCableLength(link1cable,link2cable)
M1=[-0.5,0];
M2=[1,0];
M3=[-1,0];

cable1=sqrt(sum((link1cable-M1).^2));
cable2=sqrt(sum((link2cable-M2).^2));
cable3=sqrt(sum((link2cable-M3).^2));


end
