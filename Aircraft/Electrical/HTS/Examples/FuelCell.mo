within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell "Double line to ground fault on the load side of the inverter."

  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-110,-60},{-90,-40}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.0146)
    annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
  Modelica.Electrical.Analog.Sources.RampVoltage     rampVoltage(
    V=100,
    duration=2,
    offset=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-100,-20})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=0.2)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,0})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(v(fixed=true, start=10),
      C=1.6635)                                                  annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-62,22})));
  LiquidCooled.HTS_filmboiling_Voltage2 hTS_filmboiling3_1(
    l=1,
    n=20,
    I_c0=7800,
    A=0.1,
    A_cu=0.1,
    I_crit=100000,
    T_c(displayUnit="K"),
    R_L=1e-4,
    G_d=0,
    a=0.1,
    b=0.5,
    P=1)  annotation (Placement(transformation(extent={{-6,4},{10,-4}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{28,24},{8,44}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{74,24},{54,44}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor3
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-14,-22})));
  Modelica.Blocks.Sources.Step     step(
    height=-999.95,
    offset=1000,
    startTime=5)
    annotation (Placement(transformation(extent={{-62,-32},{-42,-12}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
                                                             averagedInverter(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant100percent
      data)
    annotation (Placement(transformation(extent={{40,-16},{60,4}})));
  Modelica.Blocks.Sources.Sine sine1[
                                   3](
    offset={0,0,0},
    freqHz=60*{1,1,1},
    amplitude=1000*{1,1,1},
    phase={0,-2.0943951023932,-4.1887902047864})
    annotation (Placement(transformation(extent={{-74,-74},{-54,-54}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation noModulation1(
      enableLimitation=false)
    annotation (Placement(transformation(extent={{-22,-74},{-2,-54}})));
  Modelica.Electrical.MultiPhase.Basic.VariableResistor
                                            resistor2
    annotation (Placement(transformation(extent={{76,-16},{96,4}})));
  Modelica.Electrical.MultiPhase.Basic.Star star
    annotation (Placement(transformation(extent={{114,-16},{134,4}})));
  Modelica.Blocks.Sources.Constant const2[3](k={20,20,20})
    annotation (Placement(transformation(extent={{126,6},{106,26}})));
equation
  connect(resistor.n, resistor1.p)
    annotation (Line(points={{-54,0},{-40,0}}, color={0,0,255}));
  connect(ground2.p, rampVoltage.n)
    annotation (Line(points={{-100,-40},{-100,-30}}, color={0,0,255}));
  connect(rampVoltage.p, resistor.p)
    annotation (Line(points={{-100,-10},{-100,0},{-74,0}}, color={0,0,255}));
  connect(capacitor.p, resistor.p) annotation (Line(points={{-72,22},{-84,22},{
          -84,0},{-74,0}}, color={0,0,255}));
  connect(capacitor.n, resistor1.p) annotation (Line(points={{-52,22},{-46,22},
          {-46,0},{-40,0}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.port_a, prescribedTemperature1.port)
    annotation (Line(points={{2.2,4},{2,4},{2,34},{8,34}}, color={191,0,0}));
  connect(hTS_filmboiling3_1.pin_p, resistor1.n)
    annotation (Line(points={{-7,0},{-20,0}}, color={0,0,255}));
  connect(const1.y, prescribedTemperature1.T)
    annotation (Line(points={{53,34},{30,34}}, color={0,0,127}));
  connect(resistor3.n, ground2.p) annotation (Line(points={{-14,-32},{-14,-40},
          {-100,-40}}, color={0,0,255}));
  connect(resistor3.R, step.y)
    annotation (Line(points={{-26,-22},{-41,-22}}, color={0,0,127}));
  connect(averagedInverter.normalizedPhaseVoltages,noModulation1. normalizedPhaseVoltages)
    annotation (Line(points={{38,-6},{18,-6},{18,-64},{-1,-64}},   color={0,0,
          127}));
  connect(resistor2.plug_p, averagedInverter.plug)
    annotation (Line(points={{76,-6},{60,-6}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.pin_n, averagedInverter.pin_p)
    annotation (Line(points={{11,0},{40,0}}, color={0,0,255}));
  connect(resistor3.p, averagedInverter.pin_p) annotation (Line(points={{-14,
          -12},{16,-12},{16,0},{40,0}}, color={0,0,255}));
  connect(averagedInverter.pin_n, ground2.p) annotation (Line(points={{40,-12},
          {22,-12},{22,-40},{-100,-40}}, color={0,0,255}));
  connect(sine1.y, noModulation1.phaseVoltages)
    annotation (Line(points={{-53,-64},{-24,-64}}, color={0,0,127}));
  connect(resistor2.plug_n, star.plug_p)
    annotation (Line(points={{96,-6},{114,-6}}, color={0,0,255}));
  connect(star.pin_n, ground2.p) annotation (Line(points={{134,-6},{148,-6},{
          148,-40},{-100,-40}}, color={0,0,255}));
  connect(const2.y, resistor2.R)
    annotation (Line(points={{105,16},{86,16},{86,6}}, color={0,0,127}));
  connect(averagedInverter.electricDriveBus, noModulation1.electricDriveBus)
    annotation (Line(
      points={{50,-16},{52,-16},{52,-80},{-12,-80},{-12,-74}},
      color={0,86,166},
      thickness=0.5));
  annotation (Diagram(coordinateSystem(extent={{-140,-100},{100,60}})), Icon(
        coordinateSystem(extent={{-140,-100},{100,60}})),
    experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
end FuelCell;
