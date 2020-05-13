clc
clear all;
close all;


R=15;
L=1.2e-3; RL=50e-3; 
C=22e-6;

f=400; w=2*pi*f; 
fsw=20e+3; Tsw=1/fsw; wsw=2*pi*fsw;
%parameters for IGBT 6MBP30RH060-50
Vfs=1.2;
Vfd=0.7;
Rsd=0.05;
DT=1e-6;


Vdc=300; Vdcref=300;
% Idref=3*sqrt(3); Iqref=0;
Idref=3*sqrt(3); Iqref=0;
Ddref=0.355; Dqref=0.11;

Vse=60;
Vsm=Vse*sqrt(2);
Vsdq=[sqrt(3/2)*Vsm; 0];
Vdref=Vsdq(1); Vqref=Vsdq(2);
fv=100; fi=1000;

kpi=2*pi*fi*L/Vdcref; 
kii=2*pi*fi*(RL)/Vdcref;
kpv=2*pi*fv*C;
kiv=2*pi*fv/R;

fprintf('\ndone\n');




