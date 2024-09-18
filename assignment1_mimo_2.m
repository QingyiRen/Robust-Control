clear all; clc;close all;

load('Assignment_Data_SC42145_2022.mat');
FWT_tf = tf(FWT);

G = FWT_tf(1,1:2);
Gd = FWT_tf(1,3);
Gd = [1 Gd];
% FFT to wind data
% L = 60001;
% Fs = 100;
% plot(WindData(:,1),WindData(:,2));
% y = fft(WindData(:,2));
% figure();
% plot(WindData(:,1),abs(y));
% p2 = abs(y/L);
% p1 = p2(1:L/2+1);
% % p1(2:end-1) = 2*p1(2:end-1);
% f = Fs*(0:(L/2))/L;
% figure();
% plot(f,p1);

%% new generalized 
s=tf('s');
wB1 = 0.5*2*pi;
wB2 = 10*pi;
A = 1/10000;
M = 3;
Wp=(s/M+wB1)/(s+wB1*A);
% Wu = [0.1*(s^2+14e-4*s+1e-6)/(5e-3*s^2+7e-4*s+5e-5) 0;0 0.001*(s+10)^3/(s+0.1)^3];
Wu = [100*(s+0.1)^2/((s+10)^2) 0;0 0.01*(s+1)^2/((s+0.01)^2)];
% Wu = [1*(s^2+14e-4*s+1e-6)/(5e-3*s^2+7e-4*s+5e-5) 0;0 0.01/(s+0.01)];
% Wu=[s/(s+10) 0;0 1/(s+1)];




Wt = [];
%原始
% systemnames = 'G Wp Wu'; % Define systems
% inputvar = '[w(1); u(2)]'; % Input generalized plant
% input_to_G = '[u]';
% input_to_Wu = '[u]';
% % input_to_Wt= '[G]';
% input_to_Wp = '[w+G]';
% outputvar = '[Wp;Wu;-G-w]'; % Output generalized plant
% sysoutname= 'P';

%一个Gd
% systemnames = 'G Wp Wu Gd'; % Define systems
% inputvar = '[w(1); u(2)]'; % Input generalized plant
% input_to_G = '[u]';
% input_to_Wu = '[u]';
% % input_to_Wt= '[G]';
% input_to_Wp = '[Gd+G]';
% input_to_Gd = '[w]';
% outputvar = '[Wp;Wu;-G-Gd]'; % Output generalized plant
% sysoutname= 'P';

%Gd and I
systemnames = 'G Wp Wu Gd'; % Define systems
inputvar = '[w(2); u(2)]'; % Input generalized plant
input_to_G = '[u]';
input_to_Wu = '[u]';
% input_to_Wt= '[G]';
input_to_Wp = '[Gd+G]';
input_to_Gd = '[w]';
outputvar = '[Wp;Wu;-G-Gd]'; % Output generalized plant
sysoutname= 'P';
sysic;
[K,CL,GAM,INFO] = hinfsyn(P,1,2); % Hinf design
K_tf = tf(K);
L = G*K;%pay attention to the order!
sys = feedback(L,eye(1));
t = 0:0.05:200;
u = ones(length(t),1);
figure();
subplot(2,2,1);
lsim(sys,u,t);

%%
% subplot(2,2,2);
% bode(K);

%KS and its weight
S = inv(1-G*K);
KS = K*S;
subplot(2,2,3);
bode(KS(1,1));
hold on;
bode(1/(Wu(1,1)));
legend('\beta','Wu1');
subplot(2,2,4);
bode(KS(2,1));
hold on;
bode(1/(Wu(2,2)));
legend('\tau_e','Wu2');

% sensitivity and its weight
subplot(2,2,2);
bode(S);
hold on;
bode(1/Wp);
legend('s','Wp');


%for simulink
sys_G = ss(minreal(G));
G_A = sys_G.A;
G_B = sys_G.B;
G_C = sys_G.C;
G_D = sys_G.D;

ctr_A  = K.A;
ctr_B  = K.B;
ctr_C  = K.C;
ctr_D  = K.D;

sys_Gd = minreal(ss(minreal(Gd(1,2))));
Gd_A = sys_Gd.A;
Gd_B = sys_Gd.B;
Gd_C = sys_Gd.C;
Gd_D = sys_Gd.D;
