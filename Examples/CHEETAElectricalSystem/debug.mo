within CHEETA.Examples.CHEETAElectricalSystem;
model debug
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  replaceable Atmospheres.CoolingMedium.LH2                   coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{138,68},
            {150,80}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 20)
    annotation (Placement(transformation(extent={{-44,66},{-24,86}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=1)
                                                        annotation (Placement(
        transformation(extent={{8,66},{-12,86}})));
  Aircraft.Electrical.HTS.HTS_Piline3 hTS_Piline3_1(
                                              l=1, I_c0=10.68,
    I_crit=2000,
    R_L=10,
    G_d=0)
    annotation (Placement(transformation(extent={{14,36},{30,28}})));
  Aircraft.Electrical.HTS.HTS_Piline3 hTS_Piline3_2(
                                              l=1, I_c0=10.68,
    I_crit=2000,
    R_L=10,
    G_d=0)
    annotation (Placement(transformation(extent={{16,22},{32,14}})));
  ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.VoltageInputConstantEfficiency
                                                         converterVoltageInput(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.Records.Data.ConstantEfficiency.Constant100percent
      data)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-8,24})));
  Modelica.Blocks.Sources.Constant voltage_ce(k=500)
                                                    annotation (Placement(transformation(extent={{-34,-14},
            {-14,6}})));
  Aircraft.Mechanical.Loads.Fan fan2
    annotation (Placement(transformation(extent={{148,20},{156,28}})));
  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{36,52},{56,72}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{38,28},{58,36}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{38,16},{58,24}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare CHEETA.Aircraft.Electrical.Machines.Records.Boeing747
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      Aircraft.Electrical.PowerElectronics.Converters.DCAC.SwitchingInverter
      inverter,
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare CHEETA.Aircraft.Electrical.Machines.Records.Boeing747
        data))      annotation (Placement(transformation(extent={{74,14},{94,34}})));
  ElectrifiedPowertrains.ElectricDrives.Common.Blocks.EnergyAnalyser driveEfficiencyComputation(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{90,-14},{110,6}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.ElectricQuantities machineVariables(
      terminalConnection=electricDrive.machine.data.terminalConnection)
    annotation (Placement(transformation(extent={{120,-14},{140,6}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-6,-24},{14,-4}})));
  Modelica.Blocks.Sources.Constant voltage_ce1(k=500)
                                                    annotation (Placement(transformation(extent={{-34,-72},
            {-14,-52}})));
  ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.VoltageInputConstantEfficiency
                                                         converterVoltageInput1(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.Records.Data.ConstantEfficiency.Constant100percent
      data)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-8,-34})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-44,-48},{-24,-28}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=0)
    annotation (Placement(transformation(extent={{-80,8},{-68,20}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{126,18},
            {138,30}})));
  Modelica.Blocks.Sources.Constant voltage_ce3(k=500)
                                                    annotation (Placement(transformation(extent={{78,-226},
            {98,-206}})));
equation
  connect(fixedTemperature.port, thermalConductor.port_b)
    annotation (Line(points={{-24,76},{-12,76}}, color={191,0,0}));
  connect(thermalConductor.port_a, hTS_Piline3_2.port_a)
    annotation (Line(points={{8,76},{24,76},{24,22}}, color={191,0,0}));
  connect(hTS_Piline3_1.port_a, hTS_Piline3_2.port_a)
    annotation (Line(points={{22,36},{22,22},{24,22}}, color={191,0,0}));
  connect(converterVoltageInput.p2, hTS_Piline3_1.pin_p)
    annotation (Line(points={{2,30},{8,30},{8,32},{13,32}}, color={0,0,255}));
  connect(converterVoltageInput.v, voltage_ce.y)
    annotation (Line(points={{0,12},{0,-4},{-13,-4}},   color={0,0,127}));
  connect(electricDrive.pin_p, circuitBreaker1.n1)
    annotation (Line(points={{74,30},{57.8,30}}, color={0,0,255}));
  connect(circuitBreaker1.p1, hTS_Piline3_1.pin_n) annotation (Line(points={{38,
          30},{34,30},{34,32},{31,32}}, color={0,0,255}));
  connect(circuitBreaker2.n1, electricDrive.pin_n)
    annotation (Line(points={{57.8,18},{74,18}}, color={0,0,255}));
  connect(hTS_Piline3_2.pin_n, circuitBreaker2.p1)
    annotation (Line(points={{33,18},{38,18}}, color={0,0,255}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{57,62},{84,62},{84,36}}, color={0,0,127}));
  connect(driveEfficiencyComputation.electricDriveBus, electricDrive.electricDriveBus)
    annotation (Line(
      points={{100,-14},{100,-24},{84,-24},{84,14}},
      color={0,86,166},
      thickness=0.5));
  connect(machineVariables.electricDriveBus,driveEfficiencyComputation. electricDriveBus)
    annotation (Line(
      points={{130,-14},{130,-24},{100,-24},{100,-14}},
      color={0,86,166},
      thickness=0.5));
  connect(thermalConductor.port_a, hTS_Piline3_1.port_a)
    annotation (Line(points={{8,76},{22,76},{22,36}}, color={191,0,0}));
  connect(converterVoltageInput.n2, ground.p)
    annotation (Line(points={{2,18},{4,18},{4,-4}}, color={0,0,255}));
  connect(converterVoltageInput1.v, voltage_ce1.y)
    annotation (Line(points={{0,-46},{0,-62},{-13,-62}}, color={0,0,127}));
  connect(converterVoltageInput1.p1, converterVoltageInput.n1) annotation (Line(
        points={{-18,-28},{-34,-28},{-34,18},{-18,18}}, color={0,0,255}));
  connect(converterVoltageInput1.p2, ground.p) annotation (Line(points={{2,-28},
          {10,-28},{10,2},{4,2},{4,-4}}, color={0,0,255}));
  connect(hTS_Piline3_2.pin_p, converterVoltageInput1.n2) annotation (Line(
        points={{15,18},{14,18},{14,-40},{2,-40}}, color={0,0,255}));
  connect(converterVoltageInput1.p1, ground1.p)
    annotation (Line(points={{-18,-28},{-34,-28}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p, converterVoltageInput.p1) annotation (Line(
        points={{-67,18},{-44,18},{-44,30},{-18,30}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, converterVoltageInput1.n1) annotation (
      Line(points={{-67,10},{-42,10},{-42,-40},{-18,-40}}, color={0,0,255}));
  connect(electricDrive.flange, multiSensor.flange_a)
    annotation (Line(points={{94,24},{126,24}}, color={0,0,0}));
  connect(fan2.flange_a1, multiSensor.flange_b)
    annotation (Line(points={{147,24},{138,24}}, color={0,0,0}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},{
            260,100}})),
    Icon(coordinateSystem(extent={{-100,-180},{260,100}}, preserveAspectRatio=false), graphics),
    experiment(
      StopTime=10,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end debug;
