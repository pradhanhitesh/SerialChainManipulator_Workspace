tic
clc, format compact
%% GENERATING THE WORKSPACE (WILL TAKE AROUND 26 SECONDS TO RUN)

%Defining the link lengths of SCM
l1=0.52; l2=0.48;

%Plotting the general workspace of SCM i.e, a circle with following
%dimensions. Here, we assume there are no cables.
r=1;
origin=[0,0];
viscircles(origin,r,'Color',"black"); 
axis equal
grid on
hold on

%Setting the link2cable attachment ratio
%The position on the link where the cable will be attached
%The cable will be attached at 2/3rd the length of the link
p=0.66;
%
%%IMPORTANT%%
%It will take time to generate the workspace when you run the FOR loop.
%Do not think that MATLAB stopped working.

for x=-2:0.02:2
    for y=-2:0.02:2
        
        if x^2 + y^2 >= r^2
            continue
        end
        point = [x,y];
        %Calculating the joint angles
        [q11,q12,q21,q22]=calcIKangles(x,y,l1,l2); %Joint angles calculated
        %Calculating the structure matrix
        [A]=strucutrematrix(q11,q21,l1,l2,p); %A(q) is generated
        A(isnan(A))=0; %if there are any Nan values, convert them to zero
        rankA = rank(A);
        %rankA = 2; otherwise the point do not exist in workspace
        if rankA~=2
            continue 
        end
        
        %Checking for null
        nullofA = null(A);
        if all(nullofA(:,1)>0) || all(nullofA(:,1)<0)
            valid=true;
        else
            valid=false; 
        end 
        if valid
           axis equal
           plot(point(1),point(2),'.m');
           %hold on
           %plotConfigs(point(1),point(2));
           %xlabel('X-axis');ylabel('Y-axis');
           %title('Workspace for 2-Joint SCM with 1 redundancy');
           disp('computing WORKSPACE')
        else
           disp('computing WORKSPACE')
        end      
        
    end    
end 


disp('STATUS: COMPLETED')
%% PLOTTING THE LINKS AND CABLES

%Define link lengths
l1=0.52; l2=0.48;

%Define attachment of Link1 at origin
origin=[0,0];

%Provide the coordinates for the end-effector position
%Make sure to input (x,y) that exists in the workspace
%If you want to know what points exists in workspace, generate the 
%'workspace' plot from the FOR loop above and choose your points
x=0.55; y=-0.71;

%Throught inverse kinematics, calculate the joint configuration.
%The i below, denotes initial. The initial configuration will serve as
%baseline for measuring the change in cable length and change in angle.
[q11_i,~,q21_i,~]=calcIKangles(x,y,l1,l2);

%Defining the coordinates of end of Link1
%End of Link1 will be starting for Link2
x2=l1*cosd(q11_i);
y2=l1*sind(q11_i);
%link1end=[x2,y2];

%Defining the coordinates of end of Link2
x3=l1*cosd(q11_i)+l2*cosd(q11_i+q21_i);
y3=l1*sind(q11_i)+l2*sind(q11_i+q21_i);
%link2end=[x3,y3];
axis square
%Plotting Link1 and Link2
hold on
grid on
plot([0 x2],[0 y2],'b','linewidth',4)%Link 1
plot([x2 x3],[y2 y3],'b','linewidth',4)%Link 2
plot(x2,y2,'.k','MarkerSize',20)%Joint 1
plot(origin(1),origin(2),'.k','MarkerSize',20)%Origin
plot(x3,y3,'.k','MarkerSize',20)%Joint 2

%Plotting the cables attached to Link1 and Link2
%The cables are attached to 2/3 length of Link1 and Link2
%The cable attachment to links divide the links in a ratio of 2:1
x4=(2*x2)/3;
y4=(2*y2)/3;
link1cable_i=[x4,y4]; %Cable1 from motor position (0.5,0) attached here
M1=[0.5,0]; %Motor Position 1
plot([x4 M1(1)],[y4 M1(2)],'k')%Cable1

x5=((2*x3)+x2)/3;
y5=((2*y3)+y2)/3;
link2cable_i=[x5,y5]; %Cable2 and 3 from motor position (1,0) and (-1,0) attached here
M2=[1,0];%Motor Position 2
M3=[-1,0];%Motor Position 3
plot([x5 M2(1)],[y5 M2(2)],'k')%Cable2
plot([x5 M3(1)],[y5 M3(2)],'k')%Cable3
title('2 JOINT-1 REDUNDANCY SYSTEM WORKSPACE') 
xlabel('X-axis');ylabel('Y-axis');

toc           



































function [q11,q12,q21,q22]=calcIKangles(x,y,l1,l2)
r = (x^2 + y^2);
D = (r - l1^2 - l2^2)/(2*l1*l2);

q21= rad2deg(atan2(real(sqrt(1-D^2)),D));
q22= rad2deg(atan2(real(-sqrt(1-D^2)),D));
q11=rad2deg(atan2(y,x)-atan2(l2*sind(q21),l1+l2*cosd(q21)));
q12=rad2deg(atan2(y,x)-atan2(l2*sind(q22),l1+l2*cosd(q22)));

end 


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

function plotConfigs(x,y)
r=1;
origin=[0,0];
viscircles(origin,r,'Color',"black"); 
axis ([-1.5 1.5 -1.5 1.5])
grid on
hold on

%Define link lengths
l1=0.52; l2=0.48;

%Define attachment of Link1 at origin
origin=[0,0];

[q11,~,q21,~]=calcIKangles(x,y,l1,l2);
x2=l1*cosd(q11);
y2=l1*sind(q11);
%link1end=[x2,y2];

%Defining the coordinates of end of Link2
x3=l1*cosd(q11)+l2*cosd(q11+q21);
y3=l1*sind(q11)+l2*sind(q11+q21);
%link2end=[x3,y3];


%Plotting Link1 and Link2
hold on
grid on
plot([0 x2],[0 y2],'linewidth',3)
plot([x2 x3],[y2 y3],'linewidth',3)
plot(x2,y2,'.k','MarkerSize',20)
plot(origin(1),origin(2),'.k','MarkerSize',20)
plot(x3,y3,'.k','MarkerSize',20)

%Plotting the cables attached to Link1 and Link2
%The cables are attached to 2/3 length of Link1 and Link2
%The cable attachment to links divide the links in a ratio of 2:1
x4=(2*x2)/3;
y4=(2*y2)/3;
M1=[0.5,0];
plot([x4 M1(1)],[y4 M1(2)])

x5=((2*x3)+x2)/3;
y5=((2*y3)+y2)/3;
M2=[1,0];
M3=[-1,0];
plot([x5 M2(1)],[y5 M2(2)])
plot([x5 M3(1)],[y5 M3(2)])





function [q11,q12,q21,q22]=calcIKangles(x,y,l1,l2)
r = (x^2 + y^2);
D = (r - l1^2 - l2^2)/(2*l1*l2);

q21= rad2deg(atan2(real(sqrt(1-D^2)),D));
q22= rad2deg(atan2(real(-sqrt(1-D^2)),D));
q11=rad2deg(atan2(y,x)-atan2(l2*sind(q21),l1+l2*cosd(q21)));
q12=rad2deg(atan2(y,x)-atan2(l2*sind(q22),l1+l2*cosd(q22)));

end 





end