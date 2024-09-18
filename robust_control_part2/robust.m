clear all; clc;close all;
% load('K_tf2.mat');
load('K_last.mat');
K_pre = K_tf2;
load('Assignment_Data_SC42145_2022.mat');
FWT_tf = tf(FWT);

G = FWT_tf(1:2,1:2);
Gd = FWT_tf(1:2,3);

s=tf('s');
wB1 = 0.3*2*pi;
A = 1/10000;
M = 3;
Wp=[(s/M+wB1)/(s+wB1*A) 0; 0 0.2];
Wu = [0.01 0;0 (5e-3*s^2+7e-4*s+5e-5)/(s^2+14e-4*s+1e-6)];

Wi=[(s/(16*pi)+0.2)/(s/(64*pi)+1) 0;0 (s/(16*pi)+0.2)/(s/(64*pi)+1)];
Wo=[(0.05*s+0.2)/(0.01*s+1) 0;0 (0.05*s+0.2)/(0.01*s+1)];

%% generalized plant
systemnames = 'G Wp Wu Wi Wo';
inputvar = '[udeli(2); udelo(2); w(2); u(2)]';
outputvar='[Wi; Wo; Wp; Wu; -G-udelo-w]';
input_to_G = '[u+udeli]';
input_to_Wp = '[G+udelo+w]';
input_to_Wu = '[u]';
input_to_Wi = '[u]';
input_to_Wo = '[G]';
sysoutname='P'; cleanupsysic= 'yes'; sysic;

%% check NS NP RS RP

N=lft(P,K_pre);
N=minreal(N);
%% generalized nyquist
L4 = minreal(G*K_pre);
pole(L4)
detL4 = (L4(1,1)+1)*(L4(2,2)+1)-L4(2,1)*L4(1,2);
nyquist(detL4);
% nyquist(N)
N_ss=tf(N);
bode(N)
max(real(eig(N))) %NS

omega=logspace(-3,3,61);
Nf=frd(N,omega);
blk=[2 4];
[mubnds,muinfo]=mussv(Nf(5:8,5:6),blk,'c');
muNP=mubnds(:,1);
[muNPinf, muNPw]=norm(muNP,inf); 
muNPinf  %NP

blk=[1 1;1 1;1 1;1 1];
[mubnds,muinfo]=mussv(Nf(1:4,1:4),blk,'c');
muRS=mubnds(:,1);
[muRSinf, muRSw]=norm(muRS,inf);
muRSinf  %RS

blk=[1 1;1 1;1 1;1 1;2 4];
[mubnds,muinfo]=mussv(Nf(1:8,1:6),blk,'c');
muRP=mubnds(:,1);
[muRPinf, muRPw]=norm(muRP,inf);
muRPinf  %RP

%% mu synthesis
Delta=[ultidyn('D1',[1,1]) 0 0 0; 0 ultidyn('D2',[1,1]) 0 0; ...
       0 0 ultidyn('D3',[1,1]) 0; 0 0 0 ultidyn('D4',[1,1])];
Punc=lft(Delta,P);
% opt=dkitopt('display','full','FrequencyVector', omega,'DisplayWhileAutoIter','on');
% opts = musynOptions('Display','full','MixedMU','on','FullDG',false,'MaxIter',20);
% opts.Display='short';
% [K,clp,bnd,dkinfo]=dksyn(Punc,2,2,opt);
[K,CLperf] = musyn(Punc,2,2);
%% check 
N=lft(P,K); max(real(eig(N))) %NS

omega=logspace(-3,3,61);
Nf=frd(N,omega);
blk=[2 4];
[mubnds,muinfo]=mussv(Nf(5:8,5:6),blk,'c');
muNP=mubnds(:,1);
[muNPinf, muNPw]=norm(muNP,inf); 
muNPinf   %NP

blk=[1 1;1 1;1 1;1 1];
[mubnds,muinfo]=mussv(Nf(1:4,1:4),blk,'c');
muRS=mubnds(:,1);
[muRSinf, muRSw]=norm(muRS,inf);
muRSinf  %RS

blk=[1 1;1 1;1 1;1 1;2 4];
[mubnds,muinfo]=mussv(Nf(1:8,1:6),blk,'c');
muRP=mubnds(:,1);
[muRPinf, muRPw]=norm(muRP,inf);
muRPinf  %RP



%Bode plot
L1 = G*K_pre;
L2 = G*K;
bode(L1,L2)
legend('previous MIMO controller','mixed-sensitivity controller')

%Step response
sys1 = feedback(G,K_pre);
sys2 = feedback(G,K);
figure();
step(sys1,sys2);
legend('previous MIMO controller','mixed-sensitivity controller')

%For simulink
sys_G = ss(minreal(G));
G_A = sys_G.A;
G_B = sys_G.B;
G_C = sys_G.C;
G_D = sys_G.D;

ctrK_A  = K.A;
ctrK_B  = K.B;
ctrK_C  = K.C;
ctrK_D  = K.D;
 
K_pre = ss(K_pre);
ctrKpre_A  = K_pre.A;
ctrKpre_B  = K_pre.B;
ctrKpre_C  = K_pre.C;
ctrKpre_D  = K_pre.D;

sys_Gd = minreal(ss(minreal(Gd)));
Gd_A = sys_Gd.A;
Gd_B = sys_Gd.B;
Gd_C = sys_Gd.C;
Gd_D = sys_Gd.D;
