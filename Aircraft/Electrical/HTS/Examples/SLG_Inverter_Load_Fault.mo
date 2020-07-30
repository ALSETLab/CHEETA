within CHEETA.Aircraft.Electrical.HTS.Examples;
model SLG_Inverter_Load_Fault
  "Single line to ground fault on the load side of the inverter."

  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage(
    V=-999,
    duration=0.05,
    offset=1000,
    startTime=2)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-114,-4})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-124,-40},{-104,-20}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-48,-80},{-68,-60}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{-14,-80},{-34,-60}})));
  LiquidCooled.HTS_filmboiling_Voltage2 hTS_filmboiling3_2(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=100000,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0,
    a=0.1,
    b=0.5,
    P=10) annotation (Placement(transformation(extent={{-90,6},{-74,14}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-74,-50},{-54,-30}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.EnergyAnalysis.ConstantEfficiency
                                                             constantEfficiencyInverter
    annotation (Placement(transformation(extent={{-54,-6},{-34,14}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1)
    annotation (Placement(transformation(extent={{-16,0},{4,20}})));
  Modelica.Blocks.Sources.Ramp ramp[3](
    height={-2000.05,-00,-00},
    duration=0,
    offset={2000.25,2000.25,2000.25},
    startTime=2)
    annotation (Placement(transformation(extent={{140,30},{160,50}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={110,-8})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{100,-44},{120,-24}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{176,-84},{156,-64}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{210,-84},{190,-64}})));
  LiquidCooled.HTS_filmboiling_Voltage2 hTS_filmboiling3_1(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=100000,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0,
    a=0.1,
    b=0.5,
    P=10) annotation (Placement(transformation(extent={{134,2},{150,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{154,-34},{174,-14}})));
  Modelica.Electrical.MultiPhase.Basic.VariableResistor
                                            resistor1
    annotation (Placement(transformation(extent={{208,-10},{228,10}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
                                                             averagedInverter(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant100percent
      data)
    annotation (Placement(transformation(extent={{172,-10},{192,10}})));
  Modelica.Electrical.MultiPhase.Basic.Star star
    annotation (Placement(transformation(extent={{246,-10},{266,10}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation noModulation1(
      enableLimitation=false)
    annotation (Placement(transformation(extent={{110,-68},{130,-48}})));
  Modelica.Blocks.Sources.Sine sine1[
                                   3](
    offset={0,0,0},
    freqHz=60*{1,1,1},
    amplitude=1*1000*{1,1,1},
    phase={0,-2.0943951023932,-4.1887902047864})
    annotation (Placement(transformation(extent={{58,-68},{78,-48}})));
equation
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-46,-70},{-35,-70}},
                                                 color={0,0,127}));
  connect(rampVoltage.p, hTS_filmboiling3_2.pin_p)
    annotation (Line(points={{-114,6},{-114,10},{-91,10}}, color={0,0,255}));
  connect(hTS_filmboiling3_2.port_a,prescribedTemperature. port) annotation (
      Line(points={{-81.8,6},{-82,6},{-82,-70},{-68,-70}},   color={191,0,0}));
  connect(rampVoltage.n, ground.p)
    annotation (Line(points={{-114,-14},{-114,-20}}, color={0,0,255}));
  connect(constantEfficiencyInverter.p1, hTS_filmboiling3_2.pin_n)
    annotation (Line(points={{-54,10},{-73,10}}, color={0,0,255}));
  connect(constantEfficiencyInverter.n1, ground1.p)
    annotation (Line(points={{-54,-2},{-64,-2},{-64,-30}}, color={0,0,255}));
  connect(constantEfficiencyInverter.p2, resistor.p)
    annotation (Line(points={{-34,10},{-16,10}}, color={0,0,255}));
  connect(constantEfficiencyInverter.n2, resistor.n) annotation (Line(points={{
          -34,-2},{12,-2},{12,10},{4,10}}, color={0,0,255}));
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{178,-74},{189,-74}}, color={0,0,127}));
  connect(constantVoltage.p, hTS_filmboiling3_1.pin_p)
    annotation (Line(points={{110,2},{110,6},{133,6}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.port_a, prescribedTemperature1.port) annotation (
      Line(points={{142.2,2},{142,2},{142,-74},{156,-74}}, color={191,0,0}));
  connect(constantVoltage.n, ground2.p)
    annotation (Line(points={{110,-18},{110,-24}}, color={0,0,255}));
  connect(averagedInverter.pin_p, hTS_filmboiling3_1.pin_n)
    annotation (Line(points={{172,6},{151,6}}, color={0,0,255}));
  connect(averagedInverter.pin_n, ground3.p)
    annotation (Line(points={{172,-6},{164,-6},{164,-14}}, color={0,0,255}));
  connect(resistor1.plug_p, averagedInverter.plug)
    annotation (Line(points={{208,0},{192,0}}, color={0,0,255}));
  connect(resistor1.plug_n, star.plug_p)
    annotation (Line(points={{228,0},{246,0}}, color={0,0,255}));
  connect(star.pin_n, ground3.p) annotation (Line(points={{266,0},{270,0},{270,
          -18},{164,-18},{164,-14}}, color={0,0,255}));
  connect(averagedInverter.normalizedPhaseVoltages, noModulation1.normalizedPhaseVoltages)
    annotation (Line(points={{170,0},{150,0},{150,-58},{131,-58}}, color={0,0,
          127}));
  connect(noModulation1.phaseVoltages, sine1.y)
    annotation (Line(points={{108,-58},{79,-58}}, color={0,0,127}));
  connect(noModulation1.electricDriveBus, averagedInverter.electricDriveBus)
    annotation (Line(
      points={{120,-68},{120,-76},{182,-76},{182,-10}},
      color={0,86,166},
      thickness=0.5));
  connect(ramp.y, resistor1.R) annotation (Line(points={{161,40},{184,40},{184,
          28},{218,28},{218,12}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-140,-100},{280,60}})), Icon(
        coordinateSystem(extent={{-140,-100},{280,60}})),
    experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
end SLG_Inverter_Load_Fault;
