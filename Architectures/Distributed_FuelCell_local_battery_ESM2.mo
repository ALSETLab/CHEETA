within CHEETA.Architectures;
model Distributed_FuelCell_local_battery_ESM2
  "iii. Distributed architecture, with one set of fuel cell stacks routed to one individual propulsor motor (power augmented from battery DC bus)"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Ramp tauRef(
    height=-733.038285,
    duration=200,
    offset=0,
    startTime=2800)
               annotation (Placement(transformation(extent={{-80,34},{-60,54}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                annotation (Placement(transformation(extent={{146,34},
            {158,46}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{30,-70},{50,-50}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-74,-8})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-104.5,15.5})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-102,-14},{-82,6}})));
  Aircraft.Electrical.BusExt busExt(np=2, nn=1)
    annotation (Placement(transformation(extent={{-38,-36},{-40,40}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-130,-20},{-110,0}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{80,-92},{60,-72}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{34,-92},{14,-72}})));
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
    P=10) annotation (Placement(transformation(extent={{-22,-2},{-6,6}})));
  Modelica.Blocks.Sources.Ramp tauRef1(
    height=733.038285,
    duration=200,
    offset=1,
    startTime=0)
               annotation (Placement(transformation(extent={{-80,64},{-60,84}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-32,40},{-12,60}})));
  Aircraft.Electrical.BusExt busExt1(np=1, nn=1)
    annotation (Placement(transformation(extent={{16,-18},{14,22}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_5MW_ESM_Controller
        data),
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
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_5MW_ESM data))
               annotation (Placement(transformation(extent={{48,-14},{68,6}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.EnergyAnalyser machineAnalyser(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{92,-48},{72,-28}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{116,-10},
            {128,2}})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
      tau_nominal=-6820.93, w_nominal(displayUnit="rpm") = 733.03828583762)
                                          annotation (Placement(transformation(extent={{174,-46},
            {154,-26}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{114,-46},{134,-26}})));
  Modelica.Electrical.Analog.Basic.VariableResistor resistor annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={26,-30})));
  Modelica.Blocks.Sources.Step step(
    height=-900,
    offset=1000,
    startTime=100)
    annotation (Placement(transformation(extent={{-42,-68},{-22,-48}})));
equation
  connect(battery_FC_Charging.n1,ground9. p)
    annotation (Line(points={{-95.0909,11.6176},{-92,11.6176},{-92,6}},
                                                             color={0,0,255}));
  connect(battery_FC_Charging.n1,constantVoltage. n) annotation (Line(points={{
          -95.0909,11.6176},{-82,11.6176},{-82,-8},{-80,-8}},
                                                            color={0,0,255}));
  connect(constantVoltage.p,busExt. p[1]) annotation (Line(points={{-68,-8},{
          -64,-8},{-64,-9.4},{-40,-9.4}},   color={0,0,255}));
  connect(battery_FC_Charging.p1,busExt. p[2]) annotation (Line(points={{
          -95.0909,21.3235},{-78,21.3235},{-78,13.4},{-40,13.4}},
                                                color={0,0,255}));
  connect(booleanExpression.y,battery_FC_Charging. u1) annotation (Line(points={{-109,
          -10},{-103.455,-10},{-103.455,2.88235}},   color={255,0,255}));
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{36,-82},{59,-82}}, color={0,0,127}));
  connect(hTS_filmboiling3_1.port_a,prescribedTemperature. port) annotation (
      Line(points={{-13.8,-2},{-14,-2},{-14,-82},{14,-82}},  color={191,0,0}));
  connect(hTS_filmboiling3_1.pin_p,busExt. n[1])
    annotation (Line(points={{-23,2},{-38,2}},color={0,0,255}));
  connect(tauRef1.y,add. u1) annotation (Line(points={{-59,74},{-46,74},{-46,56},
          {-34,56}},
                   color={0,0,127}));
  connect(add.u2,tauRef. y)
    annotation (Line(points={{-34,44},{-59,44}},
                                               color={0,0,127}));
  connect(busExt1.p[1],hTS_filmboiling3_1. pin_n)
    annotation (Line(points={{14,2},{-5,2}}, color={0,0,255}));
  connect(electricDrive.pin_p,busExt1. n[1])
    annotation (Line(points={{48,2},{16,2}}, color={0,0,255}));
  connect(electricDrive.pin_n,ground1. p)
    annotation (Line(points={{48,-10},{40,-10},{40,-50}},
                                                        color={0,0,255}));
  connect(add.y,electricDrive. desiredSpeed)
    annotation (Line(points={{-11,50},{58,50},{58,8}},  color={0,0,127}));
  connect(machineAnalyser.electricDriveBus,electricDrive. electricDriveBus)
    annotation (Line(
      points={{82,-48},{82,-56},{58,-56},{58,-14}},
      color={0,86,166},
      thickness=0.5));
  connect(inertia.flange_b,load. flange)
    annotation (Line(
      points={{134,-36},{154,-36}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(inertia.flange_a,multiSensor. flange_b) annotation (Line(points={{114,-36},
          {106,-36},{106,-16},{142,-16},{142,-4},{128,-4}},    color={0,0,0}));
  connect(electricDrive.flange,multiSensor. flange_a)
    annotation (Line(points={{68,-4},{116,-4}},color={0,0,0}));
  connect(resistor.p, electricDrive.pin_p)
    annotation (Line(points={{26,-20},{26,2},{48,2}}, color={0,0,255}));
  connect(resistor.n, ground1.p)
    annotation (Line(points={{26,-40},{26,-50},{40,-50}}, color={0,0,255}));
  connect(resistor.R, step.y) annotation (Line(points={{14,-30},{-4,-30},{-4,
          -58},{-21,-58}}, color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{
            200,100}})),
    Icon(coordinateSystem(extent={{-140,-100},{200,100}}, preserveAspectRatio=false), graphics),
    experiment(
      StopTime=3000,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Distributed_FuelCell_local_battery_ESM2;
