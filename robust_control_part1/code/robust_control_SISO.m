clear; clc; close all;
%% SISO Analysis and Control Design
clear; clc; close all;
load('Assignment_Data_SC42145_2022');
s =tf ('s');
G=ss(A,B,C,D);  
G1=tf(G);  
L1=-G1(1,1);
%----------------------zeros and poles------------------------------%
num=[0.07988 0.003315 0.8677 -0.006493 0.03458];
den=[1  0.5979 10.98 4.709 0.5421 0.1827];
zeros=roots(num);
poles=roots(den);
%zeros location
real_z1 = real(zeros);
imag_z1 = imag(zeros);
%-----------------------poles location-------------------------------%
real_z2 = real(poles);
imag_z2 = imag(poles);
figure
plot(real_z1,imag_z1,'bo',real_z2,imag_z2,'r*','MarkerSize',10);
title('Pole-zero map');
xlabel('Real Axis') ;
ylabel('Imaginary Axis') ;
L1=minreal(L1);
figure
margin(L1)
Lcl1=feedback(L1,1);
dcgain(Lcl1);
S = stepinfo(L1);

%analysis about bandwidth and relation between time-domain and frequency-main properties%



%different parameters K_P
%frequency-main properties
K=[1;10;50;100;200;500;1000];
figure
for i=1:7
margin(K(i)*L1)
hold on
end
title('Different K_P parameters for PID controller');
legend('K_P=1','K_P=10','K_P=50','K_P=100','K_P=200','K_P=500','K_P=1000');
%time-domain properties
figure
t=1:0.005:10;
for i=1:7
Lcl1=feedback(K(i)*L1,1);
step(Lcl1,t);
hold on
end
title('Different K_P parameters for PID controller');
legend('K_P=1','K_P=10','K_P=50','K_P=100','K_P=200','K_P=500','K_P=1000');



%different parameters K_I for I controller
%frequency-main properties
Ki=[0.1;1;5;10;20;50;100];
figure
for i=1:7
margin(Ki(i)/s*L1)
hold on
end
title('Different P_I parameters for PID controller');
legend('K_I=0.1','K_I=1','K_I=5','K_I=10','K_I=20','K_I=50','K_I=100');
%time-domain properties
figure
t=1:0.005:10;
for i=1:7
Lcl1=feedback(Ki(i)/s*L1,1);
step(Lcl1,t);
hold on
end
title('Different P_I parameters for PID controller');
legend('K_I=0.1','K_I=1','K_I=5','K_I=10','K_I=20','K_I=50','K_I=100');



%different parameters K_D for D controller
%frequency-main properties
wco= 1000;
Kd=[0.001;0.01;0.1;1;2;5;10];
figure
for i=1:7
margin(Kd(i)*s/(s+wco)*L1)
hold on
end
title('Different P_D parameters for PID controller');
legend('K_D=0.001','K_D=0.01','K_D=O.1','K_D=1','K_D=2','K_D=5','K_D=10');
%time-domain properties
figure
t=1:0.005:10;
for i=1:7
Lcl1=feedback(Kd(i)*s/(s+wco)*L1,1);
step(Lcl1,t);
hold on
end
title('Different P_D parameters for PID controller');
legend('K_D=0.001','K_D=0.01','K_D=O.1','K_D=1','K_D=2','K_D=5','K_D=10');



%different parameters K_I for PID controller
%frequency-main properties
Kp = 100;
Ki = [0.1;1;5;10;20;50;100];
Kd = 5;
wco= 1000;
figure
for i=1:7
K_i=Ki(i);
Con1=K_i/s+Kp+Kd*s/(s+wco);
margin(Con1*L1)
hold on
end
title('Different P_I parameters for PID controller');
legend('P_I=0.1','P_I=1','P_I=5','P_I=10','P_I=20','P_I=50','P_I=100');
%time-domain properties
Kp = 10;
Ki = [0.1;1;5;10;20;50;100];
Kd = 5;
wco= 1000;
t=1:0.005:700;
figure
for i=1:7
K_i=Ki(i);
Con1=K_i/s+Kp+Kd*s/(s+wco);
Lcl1=feedback(Con1*L1,1);
step(Lcl1,t);
hold on
end
title('Different P_I parameters for PID controller');
legend('P_I=0.1','P_I=1','P_I=5','P_I=10','P_I=20','P_I=50','P_I=100');
% time-domain information
Kp = 10;
Ki = [0.1;1;5;10;20;50;100];
Kd = 5;
wco= 1000;
for i=1:7
K_i=Ki(i);
Con1=K_i/s+Kp+Kd*s/(s+wco);
Lcl1=feedback(Con1*L1,1);
stepinfo(Lcl1)
end
% frequency-domain information
Kp = 10;
Ki = [0.1;1;5;10;20;50;100];
Kd = 5;
wco= 1000;
for i=1:7
K_i=Ki(i);
Con1=K_i/s+Kp+Kd*s/(s+wco);
allmargin(Con1*L1)
end


%different parameters K_D for PID controller
Kp = 100;
Ki = 82;
Kd = [0.001;0.01;0.1;1;2;5;10];
wco= 1000;
figure
for i=1:7
K_d=Kd(i);
Con1=Ki/s+Kp+K_d*s/(s+wco);
margin(Con1*L1)
hold on
end
title('Different P_D parameters for PID controller');
legend('P_D=0.001','P_D=0.01','P_D=0.1','P_D=1','P_D=2','P_D=5','P_D=10');

%% Notch control and PID control
%first iteration
%---------------------Notch control------------------------%
w1=0.2;
w2=3.29;
beta1=10;
beta2=100;
beta3=10;
beta4=100;
G_N1=(s^2+2*w1*beta1+w1^2)/(s^2+2*w1*beta2+w1^2);
G_N2=(s^2+2*w2*beta3+w2^2)/(s^2+2*w2*beta4+w2^2);
figure
margin(L1*G_N1*G_N2)
%---------------------PID control------------------------%
Kp = 800;
Ki = 10;
Kd = 100;
wco= 10000;
Con1=Ki/s+Kp+Kd*s/(s+wco);
L2=-Con1*G_N1*G_N2*G1(1,1);
L2=minreal(L2);
figure
margin(L2)
Lcl2=feedback(L2,1);
figure
step(Lcl2)
dcgain(Lcl2)
S = stepinfo(L2);
t = 0:0.005:1000;
F = step(Lcl2,t);
S = stepinfo(F,t)

%second iteration
%---------------------Notch control------------------------%
w1=0.2;
w2=3.29;
beta1=1;
beta2=100
beta3=1;
beta4=100;
G_N1=(s^2+2*w1*beta1+w1^2)/(s^2+2*w1*beta2+w1^2);
G_N2=(s^2+2*w2*beta3+w2^2)/(s^2+2*w2*beta4+w2^2);
figure
margin(L1*G_N1*G_N2)
%---------------------PID control------------------------%
Kp = 330000;
Ki = 1;
Kd = 1;
wco= 10000;
Con1=Ki/s+Kp+Kd*s/(s+wco);
L2=-Con1*G_N1*G_N2*G1(1,1);
L2=minreal(L2);
figure
margin(L2)
Lcl2=feedback(L2,1);
figure
step(Lcl2)
dcgain(Lcl2)
S = stepinfo(L2);
t = 0:0.005:1000;
F = step(Lcl2,t);
S = stepinfo(F,t)

w1=0.2;
w2=3.29;
beta1=5;
beta2=100;
beta3=5;
beta4=100;
G_N1=(s^2+2*w1*beta1+w1^2)/(s^2+2*w1*beta2+w1^2);
G_N2=(s^2+2*w2*beta3+w2^2)/(s^2+2*w2*beta4+w2^2);
figure
margin(L1*G_N1*G_N2)


%---------------------PID control------------------------%
%final chosen PID parameters
Kp = 100;
Ki = 82;
Kd = 5;
wco= 1000;
Con1=Ki/s+Kp+Kd*s/(s+wco);
L2=-Con1*G_N1*G_N2*G1(1,1);
L2=minreal(L2);
figure
margin(L2)
Lcl2=feedback(L2,1);
figure
step(Lcl2)
dcgain(Lcl2)
S = stepinfo(L2);
t = 0:0.005:1000;
F = step(Lcl2,t);
S = stepinfo(F,t)

%% Disturbance rejection
%------------Disturbance rejection----------------%
% using former controller
L3=-Con1*G_N1*G_N2*G1(1,1);
y=G1(1,3)/(1+L3);
figure
t = 0:0.005:350;
step(y,t)

%enhanced controller
Kp = 600;
Ki = 200;
Kd = 5;
wco= 1000;
Con1=Ki/s+Kp+Kd*s/(s+wco);
L3=-Con1*G_N1*G_N2*G1(1,1);
y=G1(1,3)/(1+L3);
figure
t = 0:0.005:350;
step(y,t)
