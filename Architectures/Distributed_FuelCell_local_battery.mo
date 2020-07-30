within CHEETA.Architectures;
model Distributed_FuelCell_local_battery
  "iii. Distributed architecture, with one set of fuel cell stacks routed to one individual propulsor motor (power augmented from battery DC bus)"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Ramp tauRef(
    height=-733.038285,
    duration=200,
    offset=0,
    startTime=3400)
               annotation (Placement(transformation(extent={{-52,4},{-32,24}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{140,-22},
            {152,-10}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{186,44},
            {198,56}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{72,-66},{92,-46}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-100,-20})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-126,-48},{-106,-28}})));
  Aircraft.Electrical.BusExt busExt(      np=1, nn=1)
    annotation (Placement(transformation(extent={{-64,-48},{-66,28}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{26,-70},{46,-50}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{32,-86},{12,-66}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-14,-86},{-34,-66}})));
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
    P=10) annotation (Placement(transformation(extent={{-48,-14},{-32,-6}})));
  Modelica.Blocks.Sources.Ramp tauRef1(
    height=733.038285,
    duration=200,
    offset=1,
    startTime=0)
               annotation (Placement(transformation(extent={{-52,34},{-32,54}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-2,16},{18,36}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque             load(
      tau_constant=-500)                  annotation (Placement(transformation(extent={{198,-58},
            {178,-38}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{138,-58},{158,-38}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW_controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data))
    annotation (Placement(transformation(extent={{88,-26},{108,-6}})));
  Aircraft.Electrical.BusExt busExt1(nn=1, np=1)
    annotation (Placement(transformation(extent={{50,8},{48,-28}})));
equation
  connect(constantVoltage.p, busExt.p[1]) annotation (Line(points={{-94,-20},{
          -90,-20},{-90,-10},{-66,-10}},    color={0,0,255}));
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-12,-76},{11,-76}},color={0,0,127}));
  connect(hTS_filmboiling3_1.port_a,prescribedTemperature. port) annotation (
      Line(points={{-39.8,-14},{-40,-14},{-40,-76},{-34,-76}},
                                                             color={191,0,0}));
  connect(hTS_filmboiling3_1.pin_p, busExt.n[1])
    annotation (Line(points={{-49,-10},{-64,-10}},
                                              color={0,0,255}));
  connect(tauRef1.y, add.u1) annotation (Line(points={{-31,44},{-18,44},{-18,32},
          {-4,32}},color={0,0,127}));
  connect(add.u2, tauRef.y)
    annotation (Line(points={{-4,20},{-16,20},{-16,14},{-31,14}},
                                               color={0,0,127}));
  connect(inertia.flange_b,load. flange)
    annotation (Line(
      points={{158,-48},{178,-48}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(inertia.flange_a, multiSensor.flange_b) annotation (Line(points={{138,-48},
          {130,-48},{130,-28},{166,-28},{166,-16},{152,-16}},  color={0,0,0}));
  connect(electricDrive.flange, multiSensor.flange_a)
    annotation (Line(points={{108,-16},{140,-16}},
                                               color={0,0,0}));
  connect(electricDrive.desiredSpeed, add.y)
    annotation (Line(points={{98,-4},{98,26},{19,26}}, color={0,0,127}));
  connect(constantVoltage.n, ground9.p) annotation (Line(points={{-106,-20},{
          -116,-20},{-116,-28}}, color={0,0,255}));
  connect(electricDrive.pin_n, ground1.p)
    annotation (Line(points={{88,-22},{82,-22},{82,-46}}, color={0,0,255}));
  connect(busExt1.n[1], electricDrive.pin_p) annotation (Line(points={{50,-10},
          {68,-10},{68,-10},{88,-10}}, color={0,0,255}));
  connect(busExt1.p[1], hTS_filmboiling3_1.pin_n)
    annotation (Line(points={{48,-10},{-31,-10}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{
            200,60}})),
    Icon(coordinateSystem(extent={{-140,-100},{200,60}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=3600,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Distributed_FuelCell_local_battery;
