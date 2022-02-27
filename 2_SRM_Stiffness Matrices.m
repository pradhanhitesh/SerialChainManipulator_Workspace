tic
clc, format compact
%Owner: Hitesh Pradhan
%Insititute: IIT Gandhinagar
%Course: ES656-Human Robot Interaction
%
%Objective: 1. Generate workspace for 2-joint 3-cable SCM 
%           2. Generate stiffness matrix for SCM 
%           3. Generate corresponding ellipses for task space and muslce
%              space striffness matrices
%
%
%
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

%%IMPORTANT%%
%It will take time to generate the workspace when you run the FOR loop.
%Do not think that MATLAB stopped working.

for x=-2:0.03:2
    for y=-2:0.03:2
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
           plot(point(1),point(2),'.m');
           %xlabel('X-axis');ylabel('Y-axis');
           %title('Workspace for 2-Joint SCM with 1 redundancy');
           hold on
           disp('computing WORKSPACE')
        else
           disp('computing WORKSPACE')
        end                 
    end    
end 


figure(1);
%% INTIAL CONFIGURATION
%Initial Configuration is almost straight-like and from initial config
%we'll measure the changes in angle and cable lengths as we move on to
%3-5 different limb configuration.
%At intial configuration, we assume there is no active net tension. However
%passive tension may exist.
%We do not calculate K_Q,K_X for initial configuration. 



%Define link lengths
l1=0.52; l2=0.48;

%Define attachment of Link1 at origin
origin=[0,0];

%Provide the coordinates for the end-effector position at initial config
%That will almost result in straight-like configuration
%Make sure to input (x,y) that exists in the workspace
%If you want to know what points exists in workspace, generate the 
%'workspace' plot from the FOR loop above and choose your points
x=-0.02; y=-0.98;

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
figure(2); 
axis square
%Plotting Link1 and Link2
hold on
grid on
plot([0 x2],[0 y2],'b','linewidth',4)
plot([x2 x3],[y2 y3],'b','linewidth',4)
plot(x2,y2,'.k','MarkerSize',20)
plot(origin(1),origin(2),'.k','MarkerSize',20)
plot(x3,y3,'.k','MarkerSize',20)

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


%Cable lengths are calculated using the defined function calcCableLength
%Cable lengths for initial configuration is calculated
%Change in cable lengths is important for K_Q,K_X calculation
cable1_i=sqrt(sum((link1cable_i-M1).^2)); %Length of cable1 for congif-i
cable2_i=sqrt(sum((link2cable_i-M2).^2)); %Length of cable1 for congif-i
cable3_i=sqrt(sum((link2cable_i-M3).^2)); %Length of cable1 for congif-i

title('2 JOINT-1 REDUNDANCY SYSTEM - Initial Configuration') 

%% CONFIGURATION - A
%I have choosen a random position P(ax,ay)
%The 'a' signifies, configuration number, that is a. 
%Subsequently, you can have (bx,by),(cx,cy) and so on. 
%Depending on the configuration type, I have added a to parameters such as
%joint angles. For example, q11 is now q11_a and so on. 

%Define link lengths
l1=0.52; l2=0.48;

%Define attachment of Link1 at origin
origin=[0,0];

%Provide the coordinates for the end-effector position at initial config
%That will almost result in straight-like configuration
ax=0.55; ay=-0.71; %<--------------------------------------------USER INPUT

%Throught inverse kinematics, calculate the joint configuration.
[q11_a,~,q21_a,~]=calcIKangles(ax,ay,l1,l2);
final_q11 = q11_a;
final_q21 = q21_a;

%Defining the coordinates of end of Link1
%End of Link1 will be starting for Link2
ax2=l1*cosd(q11_a);
ay2=l1*sind(q11_a);
%link1end=[x2,y2];

%Defining the coordinates of end of Link2
ax3=l1*cosd(q11_a)+l2*cosd(q11_a+q21_a);
ay3=l1*sind(q11_a)+l2*sind(q11_a+q21_a);
%link2end=[x3,y3];

r=1;
viscircles(origin,r,'Color',"black"); 
axis ([-2.5 2.5 -2.5 2.5])
grid on
hold on

%Plotting Link1 and Link2
hold on
grid on
plot([0 ax2],[0 ay2],'g','linewidth',3)
plot([ax2 ax3],[ay2 ay3],'g','linewidth',3)
plot(ax2,ay2,'.k','MarkerSize',20)
plot(origin(1),origin(2),'.k','MarkerSize',20)
plot(ax3,ay3,'.k','MarkerSize',20)

%Plotting the cables attached to Link1 and Link2
%The cables are attached to 2/3 length of Link1 and Link2
%The cable attachment to links divide the links in a ratio of 2:1
ax4=(2*ax2)/3;
ay4=(2*ay2)/3;
link1cable_a=[ax4,ay4]; %Cable1 from motor position (0.5,0) attached here
M1=[0.5,0];%Motor Position 1
plot([ax4 M1(1)],[ay4 M1(2)],'k')%Cable 1

ax5=((2*ax3)+ax2)/3;
ay5=((2*ay3)+ay2)/3;
link2cable_a=[ax5,ay5]; %Cable2 and 3 from motor position (1,0) and (-1,0) attached here
M2=[1,0];%Motor Position 2
M3=[-1,0];%Motor Position 3
plot([ax5 M2(1)],[ay5 M2(2)],'k')%Cable 1
plot([ax5 M3(1)],[ay5 M3(2)],'k')%Cable 1

%Generating structure matrix for Congif-a
[Aa]=strucutrematrix(q11_a,q21_a,l1,l2,0.66);
A_a=Aa;

%Cable lengths are calculated using the defined function calcCableLength
%Cable lengths for initial configuration is calculated
%Change in cable lengths is important for K_Q,K_X calculation
cable1_a=sqrt(sum((link1cable_a-M1).^2)); %Length of cable1 for congif-a
cable2_a=sqrt(sum((link2cable_a-M2).^2)); %Length of cable2 for congif-a
cable3_a=sqrt(sum((link2cable_a-M3).^2)); %Length of cable3 for congif-a

title('2 JOINT-1 REDUNDANCY SYSTEM - Random Configuration-A') 

%CALCULATING STIFNESS MATRIX FOR CONFIG-A
%First, we need to set a diagonal matrix K_C which is stiffness coeffs of
%cables, or practicaly, the muscles. 
%The K_C is purely user-defined.
%From K_C, we calculate K_Q and K_X, which are the joint-stifness matrix
%and the end-effector stifness matrix respectively.
%
%
%K_Q=Kappa_D+Kappa_C 
%K_X=Kappa_Q+Kappa_F
%Kappa_D, Kappa_C, Kappa_Q and Kappa_F will be calculated for configs
%except the initial configuration.
%Kappa_D, Kappa_C, Kappa_Q and Kappa_F are calculated using pre-defined
%functions.
%
%
%
%

%Generate/Set random K_C values
K_C=[0.06 0 0; 0 0.04 0;0 0 0.02]
F=[1;1];

%We can calculate the tension in the cables after the config-a has been
%achieved from initial configuration.
%Tension will be equal to stifness coeff times change in cable length.
%For each configuration, there will be unique T matrix.
%T_a here means, Tension matrix for config-a
T1_a=K_C(1,1)*(cable1_a-cable1_i) %Tension in the cable1 of config-a
T2_a=K_C(2,2)*(cable2_a-cable2_i) %Tension in the cable2 of config-a
T3_a=K_C(3,3)*(cable3_a-cable3_i) %Tension in the cable3 of config-a
T_a = [T1_a;T2_a;T3_a];            %Tension matrix for config-a
T_a = max(T_a,0);
    
%KD value calculated for final positio
[Kappa_D_a]=calcKappa_D(ax,ay,T_a,0.66); 
%Kappa_C value calculated for final position
Kappa_C_a=(A_a*K_C)*transpose(A_a); 
%K_Q joint stiffness calculated for final position
K_Q_a = Kappa_D_a + Kappa_C_a

%Calculating K_X
[K_X_a,~,~]=calcK_X(ax,ay,K_Q_a,l1,l2,F)
%trans_KX = transpose(K_X_a);


syms x y
Eqn = [x y]*(K_X_a)'*(K_X_a)*[x y]'-1;
Eqn_1 = [x y]*(K_Q_a*30)'*(K_Q_a*30)*[x y]'-1;
hold on
fimplicit(Eqn,'--r','LineWidth',1)
fimplicit(Eqn_1,'r','LineWidth',1)
xlabel('X-axis');ylabel('Y-axis');
title('Task Space-Joint Space stiffness Ellipse for Config-A');
axis square
disp('STATUS: COMPLETED')
toc




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

Kappa_F=-(inv(J_t))*del_Jt_F*inv(J);
Kappa_Q=inv(J_t)*K_Q*(inv(J));
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


