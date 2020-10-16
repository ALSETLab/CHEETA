within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit_ShortFault

  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{32,-66},{52,-46}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{24,18},{4,38}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{70,18},{50,38}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor3
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={14,-28})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{38,-38},{58,-18}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=100)
    annotation (Placement(transformation(extent={{66,-38},{86,-18}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.12086)
    annotation (Placement(transformation(extent={{66,-12},{86,8}})));
  LiquidCooled.HTS_filmboiling_Voltage2 HTS(
    l=10,
    n=5,
    I_c0=5000,
    A=1e-3,
    A_cu=5.04e-6,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 53060,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P(displayUnit="mm") = 0.1035,
    C=1.59e-9)
    annotation (Placement(transformation(extent={{-30,-4},{-14,-12}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-58,-26})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{32,-168},{52,-148}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature2
    annotation (Placement(transformation(extent={{24,-84},{4,-64}})));
  Modelica.Blocks.Sources.Constant const2(k=20)
    annotation (Placement(transformation(extent={{70,-84},{50,-64}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor5
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={14,-130})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter2(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{38,-140},{58,-120}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor6(R=100)
    annotation (Placement(transformation(extent={{66,-140},{86,-120}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm2(n=3)
    annotation (Placement(transformation(extent={{-68,-118},{-54,-104}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor2(L=0.12086)
    annotation (Placement(transformation(extent={{66,-114},{86,-94}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature5
    annotation (Placement(transformation(extent={{-100,-152},{-80,-132}})));
  Modelica.Blocks.Sources.Constant const5(k=353.15)
    annotation (Placement(transformation(extent={{-134,-152},{-114,-132}})));
  LiquidCooled.HTS_filmboiling_Voltage2 HTS2(
    l=10,
    n=5,
    I_c0=5000,
    A=1e-3,
    A_cu=5.04e-6,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 53060,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P(displayUnit="mm") = 0.1035,
    C=1.59e-9)
    annotation (Placement(transformation(extent={{-30,-106},{-14,-114}})));
  Modelica.Blocks.Sources.Ramp     ramp4(
    height=-10000,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-134,-58},{-114,-38}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{-98,-68},{-78,-48}})));
  Modelica.Blocks.Sources.Ramp     ramp5(
    height=1000,
    duration=0.5,
    startTime=10.75)
    annotation (Placement(transformation(extent={{-134,-90},{-114,-70}})));
  Modelica.Blocks.Sources.Ramp     ramp1(
    height=-10000,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-112,-190},{-92,-170}})));
  Modelica.Blocks.Math.Add add2
    annotation (Placement(transformation(extent={{-76,-200},{-56,-180}})));
  Modelica.Blocks.Sources.Ramp     ramp2(
    height=1000,
    duration=0.5,
    startTime=10.75)
    annotation (Placement(transformation(extent={{-112,-222},{-92,-202}})));
equation
  connect(const1.y,prescribedTemperature1. T)
    annotation (Line(points={{49,28},{26,28}}, color={0,0,127}));
  connect(resistor3.n,ground2. p)
    annotation (Line(points={{14,-38},{14,-46},{42,-46}},   color={0,0,255}));
  connect(inverter.dc_n,ground2. p) annotation (Line(points={{38,-34},{26,-34},
          {26,-46},{42,-46}},   color={0,0,255}));
  connect(inverter.ac,resistor2. p)
    annotation (Line(points={{58,-28},{66,-28}}, color={0,0,255}));
  connect(resistor2.n,ground2. p) annotation (Line(points={{86,-28},{90,-28},{
          90,-46},{42,-46}},   color={0,0,255}));
  connect(inductor.p,resistor2. p) annotation (Line(points={{66,-2},{62,-2},{62,
          -28},{66,-28}}, color={0,0,255}));
  connect(inductor.n,ground2. p) annotation (Line(points={{86,-2},{90,-2},{90,
          -46},{42,-46}},
                     color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature1.port) annotation (Line(points={{
          -21.8,-4},{-22,-4},{-22,28},{4,28}}, color={191,0,0}));
  connect(HTS.pin_n, resistor3.p) annotation (Line(points={{-13,-8},{18,-8},{18,
          -18},{14,-18}}, color={0,0,255}));
  connect(constantVoltage.p, HTS.pin_p) annotation (Line(points={{-58,-16},{-56,
          -16},{-56,-8},{-31,-8}}, color={0,0,255}));
  connect(constantVoltage.n, ground2.p) annotation (Line(points={{-58,-36},{-58,
          -50},{24,-50},{24,-46},{42,-46}}, color={0,0,255}));
  connect(const2.y,prescribedTemperature2. T)
    annotation (Line(points={{49,-74},{26,-74}},
                                               color={0,0,127}));
  connect(resistor5.n,ground3. p)
    annotation (Line(points={{14,-140},{14,-148},{42,-148}},color={0,0,255}));
  connect(inverter2.dc_n, ground3.p) annotation (Line(points={{38,-136},{26,
          -136},{26,-148},{42,-148}}, color={0,0,255}));
  connect(inverter2.ac, resistor6.p)
    annotation (Line(points={{58,-130},{66,-130}}, color={0,0,255}));
  connect(resistor6.n,ground3. p) annotation (Line(points={{86,-130},{90,-130},
          {90,-148},{42,-148}},color={0,0,255}));
  connect(inductor2.p, resistor6.p) annotation (Line(points={{66,-104},{62,-104},
          {62,-130},{66,-130}}, color={0,0,255}));
  connect(inductor2.n, ground3.p) annotation (Line(points={{86,-104},{90,-104},
          {90,-148},{42,-148}}, color={0,0,255}));
  connect(const5.y,prescribedTemperature5. T)
    annotation (Line(points={{-113,-142},{-102,-142}},
                                               color={0,0,127}));
  connect(prescribedTemperature5.port, fuelCell_EquationBased_DetailedRohm2.port_a)
    annotation (Line(points={{-80,-142},{-61,-142},{-61,-118}}, color={191,0,0}));
  connect(HTS2.port_a, prescribedTemperature2.port) annotation (Line(points={{
          -21.8,-106},{-22,-106},{-22,-74},{4,-74}}, color={191,0,0}));
  connect(HTS2.pin_n, resistor5.p) annotation (Line(points={{-13,-110},{18,-110},
          {18,-120},{14,-120}}, color={0,0,255}));
  connect(inverter2.dc_p, ground3.p) annotation (Line(points={{38,-124},{34,
          -124},{34,-122},{26,-122},{26,-148},{42,-148}}, color={0,0,255}));
  connect(HTS2.pin_p, fuelCell_EquationBased_DetailedRohm2.p1) annotation (Line(
        points={{-31,-110},{-42,-110},{-42,-110.3},{-54,-110.3}}, color={0,0,
          255}));
  connect(inverter.dc_p, resistor3.p) annotation (Line(points={{38,-22},{28,-22},
          {28,-18},{14,-18}}, color={0,0,255}));
  connect(ramp4.y, add1.u1) annotation (Line(points={{-113,-48},{-104,-48},{
          -104,-52},{-100,-52}}, color={0,0,127}));
  connect(add1.u2, ramp5.y) annotation (Line(points={{-100,-64},{-106,-64},{
          -106,-80},{-113,-80}}, color={0,0,127}));
  connect(ramp1.y, add2.u1) annotation (Line(points={{-91,-180},{-82,-180},{-82,
          -184},{-78,-184}}, color={0,0,127}));
  connect(add2.u2, ramp2.y) annotation (Line(points={{-78,-196},{-84,-196},{-84,
          -212},{-91,-212}}, color={0,0,127}));
  connect(resistor5.R, add2.y) annotation (Line(points={{2,-130},{-28,-130},{
          -28,-190},{-55,-190}}, color={0,0,127}));
  connect(resistor3.R, add1.u1) annotation (Line(points={{2,-28},{-52,-28},{-52,
          -52},{-100,-52}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-140,-180},{100,60}})), Icon(
        coordinateSystem(extent={{-140,-180},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"));
end FuelCell_ShortCircuit_ShortFault;
