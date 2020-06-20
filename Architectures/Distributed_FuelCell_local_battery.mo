within CHEETA.Architectures;
model Distributed_FuelCell_local_battery
  "iii. Distributed architecture, with one set of fuel cell stacks routed to one individual propulsor motor (power augmented from battery DC bus)"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{28,30},{48,50}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{140,4},
            {152,16}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{186,44},
            {198,56}})));
  Aircraft.Mechanical.Loads.Fan fan(J=10)
    annotation (Placement(transformation(extent={{186,6},{194,14}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_1MW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{78,0},{98,20}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{62,-26},{82,-6}})));
  Modelica.Electrical.Analog.Sources.CosineVoltage   cosineVoltage(
    V=450,
    freqHz=0.1,
    offset=550)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-34,2})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-64.5,25.5})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-62,-4},{-42,16}})));
  Aircraft.Electrical.BusExt busExt(nn=1, np=2)
    annotation (Placement(transformation(extent={{2,-26},{0,50}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
equation
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{152,10},{185,10}},
                                              color={0,0,0}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{49,40},{88,40},{88,22}}, color={0,0,127}));
  connect(multiSensor.flange_a, electricDrive.flange) annotation (Line(points={{140,10},
          {98,10}},                             color={0,0,0}));
  connect(electricDrive.pin_n, ground1.p)
    annotation (Line(points={{78,4},{72,4},{72,-6}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, ground9.p)
    annotation (Line(points={{-55.0909,21.6176},{-52,21.6176},{-52,16}},
                                                             color={0,0,255}));
  connect(battery_FC_Charging.n1, cosineVoltage.n) annotation (Line(points={{
          -55.0909,21.6176},{-42,21.6176},{-42,2},{-40,2}},
                                                   color={0,0,255}));
  connect(cosineVoltage.p, busExt.p[1]) annotation (Line(points={{-28,2},{-24,2},
          {-24,0.6},{-2.22045e-16,0.6}},color={0,0,255}));
  connect(battery_FC_Charging.p1, busExt.p[2]) annotation (Line(points={{
          -55.0909,31.3235},{-38,31.3235},{-38,23.4},{-2.22045e-16,23.4}},
                                                color={0,0,255}));
  connect(busExt.n[1], electricDrive.pin_p) annotation (Line(points={{2,12},{40,
          12},{40,16},{78,16}},     color={0,0,255}));
  connect(booleanExpression.y, battery_FC_Charging.u1) annotation (Line(points={{-69,0},
          {-63.4545,0},{-63.4545,12.8824}},          color={255,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{
            200,60}})),
    Icon(coordinateSystem(extent={{-140,-100},{200,60}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=100,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Distributed_FuelCell_local_battery;
