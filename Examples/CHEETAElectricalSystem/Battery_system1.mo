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
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc1
                                                              annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={12,28})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm2(
      constantDutyCycle=0.5, f(displayUnit="Hz") = 100)
    annotation (Placement(transformation(extent={{2,-12},{22,8}})));
  Aircraft.Electrical.HTS.SimpleLine simpleLine(i=0.1)
    annotation (Placement(transformation(extent={{48,30},{68,38}})));
  Aircraft.Electrical.HTS.SimpleLine simpleLine1(i=0.1)
    annotation (Placement(transformation(extent={{48,18},{68,26}})));
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
  connect(dcdc1.dc_p2, simpleLine.p1)
    annotation (Line(points={{22,34},{49,34}}, color={0,0,255}));
  connect(electricDrive.pin_p, simpleLine.n1)
    annotation (Line(points={{86,34},{67,34}}, color={0,0,255}));
  connect(dcdc1.dc_n2, simpleLine1.p1)
    annotation (Line(points={{22,22},{49,22}}, color={0,0,255}));
  connect(electricDrive.pin_n, simpleLine1.n1)
    annotation (Line(points={{86,22},{67,22}}, color={0,0,255}));
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
