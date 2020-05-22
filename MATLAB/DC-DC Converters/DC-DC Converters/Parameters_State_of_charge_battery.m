%DC bus voltage
Vsource=1050;

%Battery parameters
inSOC=85;   % Initial SoC
RatedCap=75;  % Rated capacity (Ah)
NomVoltage=44.4;

%Circuit parameters
L1=0.04;
C1=0.003;
C2=C1;
Rc1=0.02;
Rc2=Rc1;
Vc1=Vsource;
Vc2=NomVoltage*21;
Lbus=0.387e-06;

%Triwave signal
freq=5000;
t1=0.5/freq;
t2=1/freq;

gain=1/150;

%Initial duty cycle
stepTime=0;
initA=0.88;
initD=0.00001;

%Voltage control
highSOC=95;
lowSOC=65;
highVolt=1050;
lowVolt=880;
highCur=120;
lowCur=-120;
