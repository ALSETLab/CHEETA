within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit

  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{36,-60},{56,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{28,24},{8,44}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{74,24},{54,44}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor3
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={18,-22})));
  Modelica.Blocks.Sources.Ramp     ramp(
    height=-9999.99,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-34,-32},{-14,-12}})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{42,-32},{62,-12}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=100)
    annotation (Placement(transformation(extent={{70,-32},{90,-12}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.12086)
    annotation (Placement(transformation(extent={{70,-6},{90,14}})));
  LiquidCooled.HTS_filmboiling_Voltage2 HTS(
    l=10,
    n=5,
    I_c0=5000,
    A=1e-3,
    A_cu=5.04e-6,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P(displayUnit="mm") = 0.1035,
    C=1.59e-9) annotation (Placement(transformation(extent={{-26,2},{-10,-6}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-54,-20})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{36,-162},{56,-142}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{28,-78},{8,-58}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{74,-78},{54,-58}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor1
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={18,-124})));
  Modelica.Blocks.Sources.Ramp     ramp1(
    height=-9999.99,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-34,-134},{-14,-114}})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter1(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{42,-134},{62,-114}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor4(R=100)
    annotation (Placement(transformation(extent={{70,-134},{90,-114}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(n=2, R_ohm_current=0.01)
    annotation (Placement(transformation(extent={{-64,-112},{-50,-98}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=0.12086)
    annotation (Placement(transformation(extent={{70,-108},{90,-88}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-96,-146},{-76,-126}})));
  Modelica.Blocks.Sources.Constant const4(k=353.15)
    annotation (Placement(transformation(extent={{-130,-146},{-110,-126}})));
  LiquidCooled.HTS_filmboiling_Voltage2 HTS1(
    l=10,
    n=5,
    I_c0=8000,
    A=1e-3,
    A_cu=5.04e-6,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 53060,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P(displayUnit="mm") = 0.1035,
    C=1.59e-9)
    annotation (Placement(transformation(extent={{-26,-100},{-10,-108}})));
equation
  connect(const1.y, prescribedTemperature1.T)
    annotation (Line(points={{53,34},{30,34}}, color={0,0,127}));
  connect(resistor3.n, ground2.p)
    annotation (Line(points={{18,-32},{18,-40},{46,-40}},   color={0,0,255}));
  connect(resistor3.R,ramp. y)
    annotation (Line(points={{6,-22},{-13,-22}}, color={0,0,127}));
  connect(inverter.dc_n, ground2.p) annotation (Line(points={{42,-28},{30,-28},
          {30,-40},{46,-40}},   color={0,0,255}));
  connect(inverter.ac, resistor2.p)
    annotation (Line(points={{62,-22},{70,-22}}, color={0,0,255}));
  connect(resistor2.n, ground2.p) annotation (Line(points={{90,-22},{94,-22},{
          94,-40},{46,-40}},   color={0,0,255}));
  connect(inductor.p, resistor2.p) annotation (Line(points={{70,4},{66,4},{66,
          -22},{70,-22}}, color={0,0,255}));
  connect(inductor.n, ground2.p) annotation (Line(points={{90,4},{94,4},{94,-40},
          {46,-40}}, color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature1.port) annotation (Line(points={{
          -17.8,2},{-18,2},{-18,34},{8,34}}, color={191,0,0}));
  connect(HTS.pin_n, resistor3.p) annotation (Line(points={{-9,-2},{22,-2},{22,
          -12},{18,-12}}, color={0,0,255}));
  connect(constantVoltage.p, HTS.pin_p)
    annotation (Line(points={{-54,-10},{-54,-2},{-27,-2}}, color={0,0,255}));
  connect(constantVoltage.n, ground2.p) annotation (Line(points={{-54,-30},{-54,
          -44},{28,-44},{28,-40},{46,-40}}, color={0,0,255}));
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{53,-68},{30,-68}},
                                               color={0,0,127}));
  connect(resistor1.n,ground1. p)
    annotation (Line(points={{18,-134},{18,-142},{46,-142}},color={0,0,255}));
  connect(resistor1.R, ramp1.y)
    annotation (Line(points={{6,-124},{-13,-124}}, color={0,0,127}));
  connect(inverter1.dc_n, ground1.p) annotation (Line(points={{42,-130},{30,
          -130},{30,-142},{46,-142}}, color={0,0,255}));
  connect(inverter1.ac, resistor4.p)
    annotation (Line(points={{62,-124},{70,-124}}, color={0,0,255}));
  connect(resistor4.n,ground1. p) annotation (Line(points={{90,-124},{94,-124},
          {94,-142},{46,-142}},color={0,0,255}));
  connect(inductor1.p, resistor4.p) annotation (Line(points={{70,-98},{66,-98},
          {66,-124},{70,-124}}, color={0,0,255}));
  connect(inductor1.n, ground1.p) annotation (Line(points={{90,-98},{94,-98},{
          94,-142},{46,-142}}, color={0,0,255}));
  connect(const4.y,prescribedTemperature4. T)
    annotation (Line(points={{-109,-136},{-98,-136}},
                                               color={0,0,127}));
  connect(prescribedTemperature4.port, fuelCell_EquationBased_DetailedRohm1.port_a)
    annotation (Line(points={{-76,-136},{-57,-136},{-57,-112}}, color={191,0,0}));
  connect(HTS1.port_a, prescribedTemperature3.port) annotation (Line(points={{
          -17.8,-100},{-18,-100},{-18,-68},{8,-68}}, color={191,0,0}));
  connect(HTS1.pin_n, resistor1.p)
    annotation (Line(points={{-9,-104},{18,-104},{18,-114}}, color={0,0,255}));
  connect(HTS1.pin_p, fuelCell_EquationBased_DetailedRohm1.p1) annotation (Line(
        points={{-27,-104},{-38,-104},{-38,-104.3},{-50,-104.3}}, color={0,0,
          255}));
  connect(inverter.dc_p, resistor3.p) annotation (Line(points={{42,-16},{32,-16},
          {32,-12},{18,-12}}, color={0,0,255}));
  connect(inverter1.dc_p, resistor1.p) annotation (Line(points={{42,-118},{32,
          -118},{32,-110},{18,-110},{18,-114}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-140,-160},{100,60}})), Icon(
        coordinateSystem(extent={{-140,-160},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit;
