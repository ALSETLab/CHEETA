within CHEETA.Examples.CHEETAElectricalSystem;
model Battery_system
  "AIM with forced cooling modeled in a simple thermal model"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{44,64},
            {56,76}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{54,32},{74,40}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{54,20},{74,28}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                        stekly_ExtraHeatGeneration(
                                               l=1, G_d=100)
    annotation (Placement(transformation(extent={{22,30},{38,38}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                        stekly_ExtraHeatGeneration1(
                                                l=1, G_d=100)
    annotation (Placement(transformation(extent={{22,18},{38,26}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{16,-46},{-4,-26}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 20)
    annotation (Placement(transformation(extent={{-50,-46},{-30,-26}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc(
    L=0.001,
    i=0.1,
    C=0.001,
    v(start=1000) = 1000)                                     annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={6,28})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="Hz") = 100)
    annotation (Placement(transformation(extent={{-4,-14},{16,6}})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Blocks.EnergyAnalysis energyAnalysis(
      useBusConnector=true)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-42,74})));
  Aircraft.Mechanical.Loads.Fan      fan1(J=1)
    annotation (Placement(transformation(extent={{120,24},{130,34}})));
  Aircraft.Electrical.Battery.DC_Battery dC_Battery annotation (Placement(
        transformation(
        extent={{-12,-11},{12,11}},
        rotation=270,
        origin={-25,28})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Data.Speed.MSL_18p5kW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      Aircraft.Electrical.PowerElectronics.Converters.DCAC.SwitchingInverter
      inverter,
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW
        data))      annotation (Placement(transformation(extent={{84,18},{104,
            38}})));
  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{72,50},{92,70}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc1(
    L=0.001,
    i=0.1,
    C=0.1,
    v=0)                                                      annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-6,-88})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm2(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-16,-130},{4,-110}})));
  Aircraft.Mechanical.Loads.Fan      fan2(J=10)
    annotation (Placement(transformation(extent={{124,-98},{144,-78}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                      stekly_ExtraHeatGeneration2(
                                              l=1, G_d=100)
    annotation (Placement(transformation(extent={{20,-78},{36,-86}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                      stekly_ExtraHeatGeneration3(
                                              l=1, G_d=100)
    annotation (Placement(transformation(extent={{20,-90},{36,-98}})));
  Aircraft.Electrical.Machines.ElectricDrives.SimpleSpeedDrive simpleSpeedDrive(
      wref=733.038285)
    annotation (Placement(transformation(extent={{70,-100},{90,-80}})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Blocks.EnergyAnalysis energyAnalysis1(
      useBusConnector=true)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-56,-64})));
  Aircraft.Electrical.Battery.DC_Battery dC_Battery1
                                                    annotation (Placement(
        transformation(
        extent={{-12,-11},{12,11}},
        rotation=270,
        origin={-35,-88})));
  ElectrifiedPowertrains.ElectricDrives.Common.Blocks.EnergyAnalyser driveEfficiencyComputation(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{110,-8},{130,12}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.ElectricQuantities machineVariables(
      terminalConnection=electricDrive.machine.data.terminalConnection)
    annotation (Placement(transformation(extent={{140,-8},{160,12}})));
equation
  connect(circuitBreaker1.p1, stekly_ExtraHeatGeneration.pin_n)
    annotation (Line(points={{54,34},{39,34}},color={0,0,255}));
  connect(circuitBreaker2.p1, stekly_ExtraHeatGeneration1.pin_n)
    annotation (Line(points={{54,22},{39,22}},color={0,0,255}));
  connect(stekly_ExtraHeatGeneration.port_a, thermalConductor.port_a)
    annotation (Line(points={{30,30},{30,-36},{16,-36}}, color={191,0,0}));
  connect(stekly_ExtraHeatGeneration1.port_a, thermalConductor.port_a)
    annotation (Line(points={{30,18},{30,-36},{16,-36}}, color={191,0,0}));
  connect(thermalConductor.port_b, fixedTemperature.port)
    annotation (Line(points={{-4,-36},{-30,-36}},  color={191,0,0}));
  connect(dcdc.fire_p,pwm. fire)
    annotation (Line(points={{0,16},{0,7}},        color={255,0,255}));
  connect(dcdc.dc_p2, stekly_ExtraHeatGeneration.pin_p)
    annotation (Line(points={{16,34},{21,34}},   color={0,0,255}));
  connect(stekly_ExtraHeatGeneration1.pin_p, dcdc.dc_n2)
    annotation (Line(points={{21,22},{16,22}},   color={0,0,255}));
  connect(dcdc.dc_p1, dC_Battery.p1) annotation (Line(points={{-4,34},{-12,34},
          {-12,34.24},{-14,34.24}},      color={0,0,255}));
  connect(dcdc.dc_n1, dC_Battery.n1) annotation (Line(points={{-4,22},{-12,22},
          {-12,22.24},{-14,22.24}},      color={0,0,255}));
  connect(energyAnalysis.batteryBus, dC_Battery.batteryBus1) annotation (Line(
      points={{-42,64},{-42,33.04},{-36,33.04}},
      color={0,255,0},
      thickness=0.5));
  connect(fan1.flange_a1, electricDrive.flange) annotation (Line(points={{118.75,
          29},{112,29},{112,28},{104,28}},        color={0,0,0}));
  connect(electricDrive.pin_p, circuitBreaker1.n1)
    annotation (Line(points={{84,34},{73.8,34}}, color={0,0,255}));
  connect(electricDrive.pin_n, circuitBreaker2.n1)
    annotation (Line(points={{84,22},{73.8,22}}, color={0,0,255}));
  connect(electricDrive.desiredSpeed, tauRef.y)
    annotation (Line(points={{94,40},{94,60},{93,60}},   color={0,0,127}));
  connect(pwm2.fire,dcdc1. fire_p) annotation (Line(points={{-12,-109},{-12,-100}},
                      color={255,0,255}));
  connect(dcdc1.dc_n2, stekly_ExtraHeatGeneration3.pin_p)
    annotation (Line(points={{4,-94},{19,-94}}, color={0,0,255}));
  connect(stekly_ExtraHeatGeneration2.pin_p, dcdc1.dc_p2)
    annotation (Line(points={{19,-82},{4,-82},{4,-82}}, color={0,0,255}));
  connect(stekly_ExtraHeatGeneration2.port_a, thermalConductor.port_a)
    annotation (Line(points={{28,-78},{28,-36},{16,-36}}, color={191,0,0}));
  connect(stekly_ExtraHeatGeneration3.port_a, thermalConductor.port_a)
    annotation (Line(points={{28,-90},{28,-36},{16,-36}}, color={191,0,0}));
  connect(fan2.flange_a1, simpleSpeedDrive.flange1) annotation (Line(points={{121.5,
          -88},{106,-88},{106,-89.6},{89,-89.6}}, color={0,0,0}));
  connect(simpleSpeedDrive.dc_p1, stekly_ExtraHeatGeneration2.pin_n)
    annotation (Line(points={{71.6,-85.8},{53.8,-85.8},{53.8,-82},{37,-82}},
        color={0,0,255}));
  connect(simpleSpeedDrive.dc_n1, stekly_ExtraHeatGeneration3.pin_n)
    annotation (Line(points={{71.6,-94},{37,-94}}, color={0,0,255}));
  connect(energyAnalysis1.batteryBus, dC_Battery1.batteryBus1) annotation (Line(
      points={{-56,-74},{-56,-82.96},{-46,-82.96}},
      color={0,255,0},
      thickness=0.5));
  connect(dC_Battery1.p1, dcdc1.dc_p1) annotation (Line(points={{-24,-81.76},{-21,
          -81.76},{-21,-82},{-16,-82}}, color={0,0,255}));
  connect(dcdc1.dc_n1, dC_Battery1.n1) annotation (Line(points={{-16,-94},{-20,-94},
          {-20,-93.76},{-24,-93.76}}, color={0,0,255}));
  connect(driveEfficiencyComputation.electricDriveBus, electricDrive.electricDriveBus)
    annotation (Line(
      points={{120,-8},{120,-18},{94,-18},{94,18}},
      color={0,86,166},
      thickness=0.5));
  connect(machineVariables.electricDriveBus,driveEfficiencyComputation. electricDriveBus)
    annotation (Line(
      points={{150,-8},{150,-18},{120,-18},{120,-8}},
      color={0,86,166},
      thickness=0.5));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-140},{180,
            100}})),
    Icon(coordinateSystem(extent={{-60,-140},{180,100}},  preserveAspectRatio=false), graphics),
    experiment(Interval=0.1, __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Battery_system;
