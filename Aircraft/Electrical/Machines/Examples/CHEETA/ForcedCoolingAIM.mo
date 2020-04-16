within CHEETA.Aircraft.Electrical.Machines.Examples.CHEETA;
model ForcedCoolingAIM
  "AIM with forced cooling modeled in a simple thermal model"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=1000)
               annotation (Placement(transformation(extent={{28,44},{48,64}})));
  ElectrifiedPowertrains.ElectricDrives.Common.Blocks.EnergyAnalyser driveEfficiencyComputation(useBusConnector=true)
    annotation (Placement(transformation(extent={{92,-40},{112,-20}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.ElectricQuantities machineVariables(terminalConnection=electricDrive.machine.data.terminalConnection)
    annotation (Placement(transformation(extent={{122,-40},{142,-20}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{146,-6},
            {158,6}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Data.Speed.MSL_18p5kW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage machine(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW data),
    redeclare ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=true)
                    annotation (Placement(transformation(extent={{52,-10},{72,
            10}})));
  replaceable Atmospheres.CoolingMedium.LH2                   coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{182,82},{194,94}})));
  ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.ThreeMassTEFC machineThermal(redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.Records.Data.ForcedCoolingThreeMassEstimation2.MSL_18p5kW data(
        coolingMedium=coolingMedium))
                        annotation (Placement(transformation(extent={{92,40},{
            112,60}})));
  Modelica.Thermal.FluidHeatFlow.Sources.Ambient coolingMediaSource(
    medium=coolingMedium,
    constantAmbientPressure=100000,
    constantAmbientTemperature=313.15) annotation (Placement(transformation(extent={{68,64},
            {56,76}})));
  Modelica.Thermal.FluidHeatFlow.Sources.Ambient coolingMediaSink(
    medium=coolingMedium,
    constantAmbientPressure=100000,
    constantAmbientTemperature=313.15) annotation (Placement(transformation(extent={{166,64},
            {178,76}})));
  Modelica.Thermal.FluidHeatFlow.Sensors.TemperatureSensor outletTemperature(
    medium=coolingMedium)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={154,62})));
  Modelica.Thermal.FluidHeatFlow.Sensors.TemperatureSensor inletTemperature(
    medium=coolingMedium)
    annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=270,
        origin={82,84})));
  Modelica.Thermal.FluidHeatFlow.Sensors.VolumeFlowSensor volumeFlowSensor(medium=coolingMedium)
    annotation (Placement(transformation(extent={{146,66},{138,74}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.FourElements inverterThermal(redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Data.FourElements.FF150R17KE4 data)
    annotation (Placement(transformation(extent={{-2,40},{18,60}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature inverterAmbientTemperature
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={8,80})));
  Modelica.Thermal.FluidHeatFlow.Sources.IdealPump linearFan1(
    medium=coolingMedium,
    dp0(displayUnit="Pa") = 1000,
    wNominal=electricDrive.machine.data.w_nom,
    V_flow0=10/60)            annotation (Placement(transformation(extent={{116,64},
            {128,76}})));
  Mechanical.Loads.Fan fan
    annotation (Placement(transformation(extent={{194,-4},{202,4}})));
  FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0, L=0)
    annotation (Placement(transformation(extent={{-44,-4},{-32,8}})));
  CB.CircuitBreaker                     circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{24,4},{44,12}})));
  CB.CircuitBreaker                     circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{24,-8},{44,0}})));
  HTS.Stekly.Stekly                   stekly(l=1)
           annotation (Placement(transformation(extent={{-6,2},{10,10}})));
  HTS.Stekly.Stekly                   stekly1(l=1)
           annotation (Placement(transformation(extent={{-6,-10},{10,-2}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{-10,-50},{-30,-30}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-54,-50},{-34,-30}})));
equation
  connect(driveEfficiencyComputation.electricDriveBus, electricDrive.electricDriveBus)
    annotation (Line(
      points={{102,-40},{102,-50},{62,-50},{62,-10}},
      color={0,86,166},
      thickness=0.5));
  connect(machineVariables.electricDriveBus, driveEfficiencyComputation.electricDriveBus)
    annotation (Line(
      points={{132,-40},{132,-50},{102,-50},{102,-40}},
      color={0,86,166},
      thickness=0.5));
  connect(electricDrive.flange, multiSensor.flange_a) annotation (Line(points={{72,0},{
          146,0}},                                                                                    color={0,0,0}));
  connect(outletTemperature.flowPort,coolingMediaSink. flowPort)
    annotation (Line(points={{154,66},{154,70},{166,70}},
                                                       color={255,0,0}));
  connect(machineThermal.flange, electricDrive.flange)
    annotation (Line(points={{112,50},{122,50},{122,0},{72,0}},  color={0,0,0}));
  connect(inverterAmbientTemperature.port,inverterThermal. heatPort_heatSink)
    annotation (Line(points={{8,74},{8,60}},              color={191,0,0}));
  connect(inletTemperature.flowPort,coolingMediaSource. flowPort)
    annotation (Line(points={{82,80},{82,70},{68,70}}, color={255,0,0}));
  connect(coolingMediaSource.flowPort,machineThermal. flowPort_a)
    annotation (Line(points={{68,70},{94,70},{94,60}}, color={255,0,0}));
  connect(volumeFlowSensor.flowPort_a,coolingMediaSink. flowPort) annotation (Line(points={{146,70},
          {166,70}},                                                                                          color={255,0,0}));
  connect(inverterThermal.thermalPortInverter, electricDrive.thermalPortInverter)
    annotation (Line(points={{8,40},{8,30},{56,30},{56,10}},     color={199,0,0}));
  connect(electricDrive.thermalPortMachine, machineThermal.thermalPort)
    annotation (Line(points={{68,10},{68,30},{102,30},{102,40}},    color={191,0,0}));
  connect(inverterAmbientTemperature.T, inletTemperature.y)
    annotation (Line(points={{8,87.2},{8,94},{82,94},{82,88.4}},     color={0,0,127}));
  connect(linearFan1.flowPort_b, volumeFlowSensor.flowPort_b)
    annotation (Line(points={{128,70},{138,70}},       color={255,0,0}));
  connect(linearFan1.flange_a, machineThermal.flange)
    annotation (Line(points={{122,64},{122,50},{112,50}},              color={0,0,0}));
  connect(machineThermal.flowPort_b, linearFan1.flowPort_a) annotation (Line(points={{110,60},
          {110,70},{116,70}},                                                                                  color={255,0,0}));
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{158,0},{193,0}},color={0,0,0}));
  connect(electricDrive.desiredSpeed, tauRef.y)
    annotation (Line(points={{62,12},{62,54},{49,54}},color={0,0,127}));
  connect(circuitBreaker1.n1, electricDrive.pin_p)
    annotation (Line(points={{43.8,6},{52,6}}, color={0,0,255}));
  connect(electricDrive.pin_n, circuitBreaker2.n1)
    annotation (Line(points={{52,-6},{43.8,-6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p, stekly.pin_p)
    annotation (Line(points={{-31,6},{-7,6}}, color={0,0,255}));
  connect(circuitBreaker1.p1, stekly.pin_n)
    annotation (Line(points={{24,6},{11,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, stekly1.pin_p) annotation (Line(points={{
          -31,-2},{-20,-2},{-20,-6},{-7,-6}}, color={0,0,255}));
  connect(circuitBreaker2.p1, stekly1.pin_n)
    annotation (Line(points={{24,-6},{11,-6}}, color={0,0,255}));
  connect(stekly.port_a, thermalConductor.port_a)
    annotation (Line(points={{2,2},{2,-40},{-10,-40}}, color={191,0,0}));
  connect(stekly1.port_a, thermalConductor.port_a)
    annotation (Line(points={{2,-10},{2,-40},{-10,-40}}, color={191,0,0}));
  connect(thermalConductor.port_b, fixedTemperature.port)
    annotation (Line(points={{-30,-40},{-34,-40}}, color={191,0,0}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{200,
            100}})),
    Icon(coordinateSystem(extent={{-60,-60},{200,100}},   preserveAspectRatio=false), graphics),
    experiment(StopTime=100, Interval=0.1),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end ForcedCoolingAIM;
