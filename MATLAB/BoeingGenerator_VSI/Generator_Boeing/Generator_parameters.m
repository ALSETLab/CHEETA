%##########################################################################
% Data of generator 
%##########################################################################
clear all

%##########################################################################
% Main Generator parameter set #3
%##########################################################################
Rpfd=4*634.998e-6; 
Lpfd=4.9*6e-6;     
Lmd=4*164.571e-6;  
Lmq=4*125.772e-6;  
Llo=4*58.615e-9;   
Ra=4*2.794e-3;     
Lls=4*2.822e-6;    
Lpkd=16*9.093e-6;   
Rpkd=8*11.72e-3;   
Lpkq=16*3.175e-6;   
Rpkq=8*3.712e-3;   

%Machine operational parameters 
Nfds_gen=25; %(3/2)*(Mmaf/Lmd);
%Parameters for simulation
Lamd=1/(1/Lls+1/Lpfd+1/Lpkd+1/Lmd);
Lamq=1/(1/Lls+1/Lmq+1/Lpkq);
Ld=Lls+Lmd;
Lq=Lls+Lmq;
Lf=Lpfd+Lmd;
Ldfm=Ld*Lf-Lmd^2;

%Magnetizing characteristic
%Magnetic flux
a=2*[0 15.9 31.82 42.6 51.7 56.9 62.9 66.0 67.6 69.2 70.4 71.2 71.6 72.0]; 
%Ksat value
% 115V@0.6A
b=[1 1 1 1 0.904 0.746 0.550 0.433 0.355 0.303 0.264 0.233 0.209 0.189];

% Generator Controller parameters
Kv=64;
wz=100;
wp=10000;
vt=230;

