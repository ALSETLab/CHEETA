within CHEETA.Examples.CHEETAElectricalSystem;
model Battery_system1
  "AIM with forced cooling modeled in a simple thermal model"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{138,68},
            {150,80}})));
  Aircraft.Mechanical.Loads.Fan      fan1(J=1)
    annotation (Placement(transformation(extent={{120,24},{130,34}})));
  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{72,48},{92,68}})));
  ElectrifiedPowertrains.ElectricDrives.Common.Blocks.EnergyAnalyser driveEfficiencyComputation(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{110,-8},{130,12}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.ElectricQuantities machineVariables(
      terminalConnection=electricDrive.machine.data.terminalConnection)
    annotation (Placement(transformation(extent={{140,-8},{160,12}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Data.Speed.MSL_18p5kW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.LossyLinearized
      inverter,
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{86,18},{106,
            38}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0, L=0)
    annotation (Placement(transformation(extent={{-34,24},{-22,36}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc1(
    L=0.0001,
    i=0.1,
    C=0.1,
    v=1000)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={12,28})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm2(
      constantDutyCycle=0.5, f(displayUnit="Hz") = 100)
    annotation (Placement(transformation(extent={{2,-12},{22,8}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-26,72},{-6,92}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{40,72},{20,92}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly2(l=1)
    annotation (Placement(transformation(extent={{46,38},{62,30}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly3(l=1)
    annotation (Placement(transformation(extent={{46,26},{62,18}})));
equation
  connect(machineVariables.electricDriveBus,driveEfficiencyComputation. electricDriveBus)
    annotation (Line(
      points={{150,-8},{150,-18},{120,-18},{120,-8}},
      color={0,86,166},
      thickness=0.5));
  connect(fan1.flange_a1, electricDrive.flange) annotation (Line(points={{
          118.75,29},{112,29},{112,28},{106,28}}, color={0,0,0}));
  connect(electricDrive.electricDriveBus, driveEfficiencyComputation.electricDriveBus)
    annotation (Line(
      points={{96,18},{96,-18},{120,-18},{120,-8}},
      color={0,86,166},
      thickness=0.5));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{93,58},{96,58},{96,40}}, color={0,0,127}));
  connect(dcdc1.dc_p1, simplifiedFuelCell.pin_p)
    annotation (Line(points={{2,34},{-21,34}}, color={0,0,255}));
  connect(dcdc1.dc_n1, simplifiedFuelCell.pin_p1)
    annotation (Line(points={{2,22},{0,22},{0,26},{-21,26}}, color={0,0,255}));
  connect(dcdc1.fire_p, pwm2.fire)
    annotation (Line(points={{6,16},{6,9}}, color={255,0,255}));
  connect(fixedTemperature.port, thermalConductor.port_b)
    annotation (Line(points={{-6,82},{20,82}}, color={191,0,0}));
  connect(thermalConductor.port_a, stekly2.port_a)
    annotation (Line(points={{40,82},{54,82},{54,38}}, color={191,0,0}));
  connect(stekly2.pin_p, dcdc1.dc_p2)
    annotation (Line(points={{45,34},{22,34}}, color={0,0,255}));
  connect(dcdc1.dc_n2, stekly3.pin_p)
    annotation (Line(points={{22,22},{45,22}}, color={0,0,255}));
  connect(stekly2.pin_n, electricDrive.pin_p)
    annotation (Line(points={{63,34},{86,34}}, color={0,0,255}));
  connect(electricDrive.pin_n, stekly3.pin_n)
    annotation (Line(points={{86,22},{63,22}}, color={0,0,255}));
  connect(stekly2.port_a, stekly3.port_a)
    annotation (Line(points={{54,38},{54,26}}, color={191,0,0}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{180,
            100}})),
    Icon(coordinateSystem(extent={{-60,-60},{180,100}},   preserveAspectRatio=false), graphics),
    experiment(Interval=0.1, __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Battery_system1;
