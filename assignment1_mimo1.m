clear all; clc;close all;

load('Assignment_Data_SC42145_2022.mat');
FWT_tf = tf(FWT);

G = FWT_tf(1:2,1:2);
% G = ss(A,B(:,1:2),C,zeros(2,2));
Gd = FWT_tf(1:2,3);

RGA_w0 = freqresp(G,0);
RGA_w1 = freqresp(G,0.3*2*pi*1i);
RGA = RGA_w0.*pinv(RGA_w0).'
RGA = RGA_w1.*pinv(RGA_w1).'

g1 = C*inv(0.3*2*pi*eye(5)-A)*B(:,1:2);
RGA = g1.*inv(g1)'

[p,z] = pzmap(minreal(ss(G)))
%% mixed sensitivity function
s=tf('s');
wB1 = 0.3*2*pi;
A = 1/10000;
M = 3;
Wp=[(s/M+wB1)/(s+wB1*A) 0; 0 0.2];
Wu = [0.01 0;0 (5e-3*s^2+7e-4*s+5e-5)/(s^2+14e-4*s+1e-6)];
Wt = [];
[K,CL,GAM,INFO]=mixsyn(G,Wp,Wu,Wt);
K = minreal(K);
L = minreal(G)*K;   %pay attention to the order!
sys = feedback(L,eye(2));
% sys = (L+eye(2))\L;
t = 0:0.05:200;
u = ones(length(t),2);
figure();
lsim(sys,u,t);


%% generalized 
systemnames = 'G Wp Wu Gd'; % Define systems
inputvar = '[d(1); r(2); u(2)]'; % Input generalized plant
input_to_G = '[u]';
input_to_Wu = '[u]';
input_to_Gd= '[d]';
input_to_Wp = '[r-Gd-G]';
outputvar = '[Wp;Wu;r-G-Gd]'; % Output generalized plant
sysoutname= 'P';
sysic;
[K2,CL2,GAM2,INFO2] = hinfsyn(P,2,2); % Hinf design
K2=minreal(K2);
K_tf2 = tf(K2);
L2 = G*K2;%pay attention to the order!
sys2 = feedback(L2,eye(2));
t = 0:0.05:200;
u = ones(length(t),2);
figure();
lsim(sys2,u,t);

P_my = [zeros(2,2) Wu;Wp Wp*G;-eye(2) -G];
[K3,CL3,GAM3,INFO3] = hinfsyn(P_my,2,2); % Hinf design
K3 = minreal(K3);
L3 = G*K3;%pay attention to the order!
sys3 = feedback(L3,eye(2));
t = 0:0.05:200;
u = ones(length(t),2);
figure();
lsim(sys3,u,t);

%--------for simulink--------%
ctr_A  = K2.A;
ctr_B  = K2.B;
ctr_C  = K2.C;
ctr_D  = K2.D;
Gss = minreal(ss(G));
GA = Gss.A;
GB = Gss.B;
GC = Gss.C;
GD = Gss.D;
Gdss = minreal(ss(Gd));
GdA = Gdss.A;
GdB = Gdss.B;
GdC = Gdss.C;
GdD = Gdss.D;
%----------------------------%
%% generalized nyquist
L4 = minreal(G*K2);
pole(L4)
detL4 = (L4(1,1)+1)*(L4(2,2)+1)-L4(2,1)*L4(1,2);
nyquist(detL4);





