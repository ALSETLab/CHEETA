clc;
clear all;
T = open('CHEETAFlightDataTable.mat');

time = T.CHEETAFlightProfilePreliminary{:,1};
altitude = T.CHEETAFlightProfilePreliminary{:,2};
mach = T.CHEETAFlightProfilePreliminary{:,3};
velocity = T.CHEETAFlightProfilePreliminary{:,4};
thrust = T.CHEETAFlightProfilePreliminary{:,5};
netPower = T.CHEETAFlightProfilePreliminary{:,6};
shaftPower = T.CHEETAFlightProfilePreliminary{:,7};
motorRPM = T.CHEETAFlightProfilePreliminary{:,8};
motorTorque = T.CHEETAFlightProfilePreliminary{:,9};
distance = T.CHEETAFlightProfilePreliminary{:,10};
weight = T.CHEETAFlightProfilePreliminary{:,11};
fuelBurned = T.CHEETAFlightProfilePreliminary{:,12};
rho_density = T.CHEETAFlightProfilePreliminary{:,13};

%% set the parameters up as tables to put into Dymola
altitude_t = [time, altitude];
mach_t = [time, mach];
velocity_t = [time,velocity];
thrust_t = [time, thrust];
netPower_t = [time, netPower];
shaftPower_t = [time, shaftPower];
motorRPM_t = [time, motorRPM];
motorTorque_t = [time, motorTorque];
distance_t = [time, distance];
weight_t = [time, weight];
fuelBurned_t = [time, fuelBurned];
rho_density_t = [time, rho_density];