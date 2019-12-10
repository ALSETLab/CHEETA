%% Load FMU, initialize, produce linear system
fmu = loadFMU('Linearization.fmu');
%fmu.fmiInstantiateModel(); % different calls for 1.0/2.0 and ME/CS, might
%not be needed and comment out if already instantiated
fmu.initialize(); % default initialization
[A,B,C,D,YLIN] = fmu.linearize()

%% Reduce model
sys = ss2tf(A,B,C,D);
[sys,g] = balreal(sys);
elim = (g<1e-8);
rsys = modred(sys, elim);
