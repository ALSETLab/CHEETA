within CHEETA.Architectures;
model Distributed_FuelCell_local_battery_ESM
  "iii. Distributed architecture, with one set of fuel cell stacks routed to one individual propulsor motor (power augmented from battery DC bus)"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Ramp tauRef(
    height=-733.038285,
    duration=200,
    offset=0,
    startTime=2800)
               annotation (Placement(transformation(extent={{-84,36},{-64,56}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{142,36},
            {154,48}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{18,-28},{38,-8}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-78,-6})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-108.5,17.5})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-106,-12},{-86,8}})));
  Aircraft.Electrical.BusExt busExt(      np=2, nn=1)
    annotation (Placement(transformation(extent={{-42,-34},{-44,42}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-134,-18},{-114,2}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{76,-90},{56,-70}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{30,-90},{10,-70}})));
  Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage2
    hTS_filmboiling3_1(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=10000,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0,
    a=0.1,
    b=0.5,
    P=10) annotation (Placement(transformation(extent={{-26,0},{-10,8}})));
  Modelica.Blocks.Sources.Ramp tauRef1(
    height=733.038285,
    duration=200,
    offset=1,
    startTime=0)
               annotation (Placement(transformation(extent={{-84,66},{-64,86}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-36,42},{-16,62}})));
  Aircraft.Electrical.BusExt busExt1(np=1, nn=1)
    annotation (Placement(transformation(extent={{12,-16},{10,24}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Records.Base.Speed
        data(redeclare
          CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
          machineData, tau_max=1000)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{44,-12},{64,8}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.EnergyAnalyser machineAnalyser(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{124,-96},{104,-76}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{112,-8},
            {124,4}})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
      tau_nominal=-6820.93, w_nominal(displayUnit="rpm") = 733.03828583762)
                                          annotation (Placement(transformation(extent={{170,-44},
            {150,-24}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{110,-44},{130,-24}})));
equation
  connect(battery_FC_Charging.n1, ground9.p)
    annotation (Line(points={{-99.0909,13.6176},{-96,13.6176},{-96,8}},
                                                             color={0,0,255}));
  connect(battery_FC_Charging.n1, constantVoltage.n) annotation (Line(points={{
          -99.0909,13.6176},{-86,13.6176},{-86,-6},{-84,-6}},
                                                            color={0,0,255}));
  connect(constantVoltage.p, busExt.p[1]) annotation (Line(points={{-72,-6},{
          -68,-6},{-68,-7.4},{-44,-7.4}},   color={0,0,255}));
  connect(battery_FC_Charging.p1, busExt.p[2]) annotation (Line(points={{
          -99.0909,23.3235},{-82,23.3235},{-82,15.4},{-44,15.4}},
                                                color={0,0,255}));
  connect(booleanExpression.y, battery_FC_Charging.u1) annotation (Line(points={{-113,-8},
          {-107.455,-8},{-107.455,4.88235}},         color={255,0,255}));
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{32,-80},{55,-80}}, color={0,0,127}));
  connect(hTS_filmboiling3_1.port_a,prescribedTemperature. port) annotation (
      Line(points={{-17.8,0},{-18,0},{-18,-80},{10,-80}},    color={191,0,0}));
  connect(hTS_filmboiling3_1.pin_p, busExt.n[1])
    annotation (Line(points={{-27,4},{-42,4}},color={0,0,255}));
  connect(tauRef1.y, add.u1) annotation (Line(points={{-63,76},{-50,76},{-50,58},
          {-38,58}},
                   color={0,0,127}));
  connect(add.u2, tauRef.y)
    annotation (Line(points={{-38,46},{-63,46}},
                                               color={0,0,127}));
  connect(busExt1.p[1], hTS_filmboiling3_1.pin_n)
    annotation (Line(points={{10,4},{-9,4}}, color={0,0,255}));
  connect(electricDrive.pin_p, busExt1.n[1])
    annotation (Line(points={{44,4},{12,4}}, color={0,0,255}));
  connect(electricDrive.pin_n, ground1.p)
    annotation (Line(points={{44,-8},{28,-8}},          color={0,0,255}));
  connect(add.y, electricDrive.desiredSpeed)
    annotation (Line(points={{-15,52},{54,52},{54,10}}, color={0,0,127}));
  connect(machineAnalyser.electricDriveBus, electricDrive.electricDriveBus)
    annotation (Line(
      points={{114,-96},{54,-96},{54,-12}},
      color={0,86,166},
      thickness=0.5));
  connect(inertia.flange_b,load. flange)
    annotation (Line(
      points={{130,-34},{150,-34}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(inertia.flange_a,multiSensor. flange_b) annotation (Line(points={{110,-34},
          {102,-34},{102,-14},{138,-14},{138,-2},{124,-2}},    color={0,0,0}));
  connect(electricDrive.flange,multiSensor. flange_a)
    annotation (Line(points={{64,-2},{112,-2}},color={0,0,0}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{
            200,60}})),
    Icon(coordinateSystem(extent={{-140,-100},{200,60}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=3000,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Distributed_FuelCell_local_battery_ESM;
