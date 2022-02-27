%Title: Calculating structure A matrix
%Author: Hitesh Pradhan
%Institute: Indian Institute of Technology, Gandhinagar, India
%Course: ES 656 Human Robot Interaction, Spring Semester 2022
%Date: 26-02-2022
%
%
%
%What is structure A matrix?
%Structure A matrix arises as result of relation between joint space and
%muslce space. Joint space is defined as number of joints (in this case,2)
%and its relating parameters such as link lengths, direction vectors.
%
%Structure A matrix is purely a function of joint angles. 
%So as the limb configuration changes, structure A matrix will change.
%To determine if a point exists in the limb-reachable workspace, this 
%matrix plays an important role.
%
%The inputs are joint angles generated from calcIKAngles function in MATLAB
%The output is a 2*3 matrix for 2 link arms driven by 3 cables. 
%For n-links and m-cables, the size of A matrix will be n*m.
%
%OM1 defines the motor position that controls cable 1 attached to link1.
%OM2 defines the motor position that controls cable 2 attached to link2.
%OM3 defines the motor position that controls cable 3 attached to link2. 
%The length of cables are determined from OM1 to site of attachment of 
%cable at the links.
%l1_cap,l2_cap,l3_cap are the unit length vectors of cables respectively.
%
%The value p that can be seen in the function can be replaced by the
%user. The value represents the ratio (2:1).
%
%2:1 ratio means that, for example, cable1 at link1 divides the link1 in
%a ratio of 2:1 and hence 2/3 of link1 length or 2/3 of link2 length. So
%if you want to attach the cables at 1/4 the length of links, use p value
%of 1/(1+4) that is 1/5, which is 0.2.
%
%The inputs are joint angle pairs, (q11 and q21) or (q12 and q22) and link
%lengths, and ration p, which are user pre-defined
%The output is a strucute matrix A.
%

function [A]=strucutrematrix(q11,q21,l1,l2,p)

r1=[l1*p*cosd(q11) l1*p*sind(q11)]; %position vector for link 1
r2=[l1*cosd(q11)+(l2*p*cosd(q11+q21)) l1*sind(q11)+(l2*p*sind(q11+q21))];%position vector for link 2 
r3=[l1*cosd(q11)+(l2*p*cosd(q11+q21)) l1*sind(q11)+(l2*p*sind(q11+q21))];%position vector for link 3

OM1=[0.5 0]; %vector array for motor1, CHANGE THIS IF REQUIRED
OM2=[1 0];   %vector array for motor2, CHANGE THIS IF REQUIRED
OM3=[-1 0];  %vector array for motor3, CHANGE THIS IF REQUIRED

l1_cap = (OM1-r1)/(sqrt(sum((OM1-r1).^2))); %unit vector for cable 1
l2_cap = (OM2-r2)/(sqrt(sum((OM2-r2).^2))); %unit vector for cable 2
l3_cap = (OM3-r3)/(sqrt(sum((OM3-r3).^2))); %unit vector for cable 3

pd_r1q1 = [-l1*p*sind(q11)  l1*p*cosd(q11)]; %partial derivate of r1/q1
pd_r2q1 = [(-l1*sind(q11)-l2*p*sind(q11+q21)) (l1*cosd(q11)+l2*p*cosd(q11+q21))];%partial derivate of r2/q1
pd_r3q1 = [(-l1*sind(q11)-l2*p*sind(q11+q21)) (l1*cosd(q11)+l2*p*cosd(q11+q21))];%partial derivate of r3/q1

pd_r1q2 = [0 0]; %partial derivate of r1/q2
pd_r2q2 = [-l2*p*sind(q11+q21) l2*p*cosd(q11+q21)];%partial derivate of r2/q2
pd_r3q2 = [-l2*p*sind(q11+q21) l2*p*cosd(q11+q21)]; %partial derivate of r3/q2

A = [dot(l1_cap,pd_r1q1) dot(l2_cap,pd_r2q1) dot(l3_cap,pd_r3q1)
    dot(l1_cap,pd_r1q2) dot(l2_cap,pd_r2q2) dot(l3_cap,pd_r3q2)]; %Structure matrix A
end 
