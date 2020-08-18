within CHEETA.Aircraft.Electrical.HTS.Examples;
model DLG_Inverter_Load_Fault
  "Double line to ground fault on the load side of the inverter."

  Modelica.Blocks.Sources.Ramp ramp[3](
    height={-2000.1,-2000.1,-00},
    duration=100,
    offset={2000.25,2000.25,2000.25},
    startTime=2)
    annotation (Placement(transformation(extent={{-44,30},{-24,50}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-142,-46},{-122,-26}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{-8,-84},{-28,-64}})));
  Modelica.Blocks.Sources.Constant const1(k=25)
    annotation (Placement(transformation(extent={{26,-84},{6,-64}})));
  LiquidCooled.HTS_filmboiling_Current  hTS_filmboiling3_1(
    l=0.1,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=100000,
    T_c(displayUnit="K"),
    R_L=1e-4,
    G_d=0,
    a=0.1,
    b=0.5,
    P=10) annotation (Placement(transformation(extent={{-50,2},{-34,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{-30,-34},{-10,-14}})));
  Modelica.Electrical.MultiPhase.Basic.VariableResistor
                                            resistor1
    annotation (Placement(transformation(extent={{24,-10},{44,10}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
                                                             averagedInverter(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant100percent
      data)
    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
  Modelica.Electrical.MultiPhase.Basic.Star star
    annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation noModulation1(
      enableLimitation=false)
    annotation (Placement(transformation(extent={{-74,-68},{-54,-48}})));
  Modelica.Blocks.Sources.Sine sine1[
                                   3](
    offset={0,0,0},
    freqHz=60*{1,1,1},
    amplitude=1100*{1,1,1},
    phase={0,-2.0943951023932,-4.1887902047864})
    annotation (Placement(transformation(extent={{-126,-68},{-106,-48}})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=1.6635) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-106,-12})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.0146)
    annotation (Placement(transformation(extent={{-94,-4},{-74,16}})));
  Modelica.Electrical.Analog.Sources.ConstantCurrent constantCurrent(I=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-132,-8})));
equation
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{-6,-74},{5,-74}},    color={0,0,127}));
  connect(hTS_filmboiling3_1.port_a, prescribedTemperature1.port) annotation (
      Line(points={{-41.8,2},{-42,2},{-42,-74},{-28,-74}}, color={191,0,0}));
  connect(averagedInverter.pin_p, hTS_filmboiling3_1.pin_n)
    annotation (Line(points={{-12,6},{-33,6}}, color={0,0,255}));
  connect(averagedInverter.pin_n, ground3.p)
    annotation (Line(points={{-12,-6},{-20,-6},{-20,-14}}, color={0,0,255}));
  connect(resistor1.plug_p, averagedInverter.plug)
    annotation (Line(points={{24,0},{8,0}},    color={0,0,255}));
  connect(resistor1.plug_n, star.plug_p)
    annotation (Line(points={{44,0},{62,0}},   color={0,0,255}));
  connect(star.pin_n, ground3.p) annotation (Line(points={{82,0},{86,0},{86,-18},
          {-20,-18},{-20,-14}},      color={0,0,255}));
  connect(averagedInverter.normalizedPhaseVoltages, noModulation1.normalizedPhaseVoltages)
    annotation (Line(points={{-14,0},{-34,0},{-34,-58},{-53,-58}}, color={0,0,
          127}));
  connect(noModulation1.phaseVoltages, sine1.y)
    annotation (Line(points={{-76,-58},{-105,-58}},
                                                  color={0,0,127}));
  connect(noModulation1.electricDriveBus, averagedInverter.electricDriveBus)
    annotation (Line(
      points={{-64,-68},{-64,-76},{-2,-76},{-2,-10}},
      color={0,86,166},
      thickness=0.5));
  connect(ramp.y, resistor1.R) annotation (Line(points={{-23,40},{0,40},{0,28},
          {34,28},{34,12}},       color={0,0,127}));
  connect(capacitor.n, ground2.p) annotation (Line(points={{-106,-22},{-106,-26},
          {-132,-26}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.pin_p, resistor.n)
    annotation (Line(points={{-51,6},{-74,6}}, color={0,0,255}));
  connect(capacitor.p, resistor.p)
    annotation (Line(points={{-106,-2},{-106,6},{-94,6}}, color={0,0,255}));
  connect(ground2.p, constantCurrent.p)
    annotation (Line(points={{-132,-26},{-132,-18}}, color={0,0,255}));
  connect(constantCurrent.n, resistor.p)
    annotation (Line(points={{-132,2},{-132,6},{-94,6}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-140,-100},{100,60}})), Icon(
        coordinateSystem(extent={{-140,-100},{100,60}})),
    experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
end DLG_Inverter_Load_Fault;
