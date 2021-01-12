within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit

  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{34,-56},{54,-36}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{26,28},{6,48}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{72,28},{52,48}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor1
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={16,-18})));
  Modelica.Blocks.Sources.Ramp     ramp1(
    height=-10000,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-36,-28},{-16,-8}})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter1(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{40,-28},{60,-8}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor4(R=100)
    annotation (Placement(transformation(extent={{68,-28},{88,-8}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(n=2, R_ohm_current=0.09)
    annotation (Placement(transformation(extent={{-72,-8},{-58,6}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=0.12086)
    annotation (Placement(transformation(extent={{68,-2},{88,18}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-98,-40},{-78,-20}})));
  Modelica.Blocks.Sources.Constant const4(k=353.15)
    annotation (Placement(transformation(extent={{-132,-40},{-112,-20}})));
  LiquidCooled.HTS_filmboiling_Voltage_Hydrogen HTS(
    l=10,
    n=20,
    I_c0=3700,
    A=0.1,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm"))
    annotation (Placement(transformation(extent={{-28,24},{-12,16}})));
equation
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{51,38},{28,38}}, color={0,0,127}));
  connect(resistor1.n,ground1. p)
    annotation (Line(points={{16,-28},{16,-36},{44,-36}},   color={0,0,255}));
  connect(resistor1.R, ramp1.y)
    annotation (Line(points={{4,-18},{-15,-18}},   color={0,0,127}));
  connect(inverter1.dc_n, ground1.p) annotation (Line(points={{40,-24},{28,-24},
          {28,-36},{44,-36}},         color={0,0,255}));
  connect(inverter1.ac, resistor4.p)
    annotation (Line(points={{60,-18},{68,-18}},   color={0,0,255}));
  connect(resistor4.n,ground1. p) annotation (Line(points={{88,-18},{92,-18},{
          92,-36},{44,-36}},   color={0,0,255}));
  connect(inductor1.p, resistor4.p) annotation (Line(points={{68,8},{64,8},{64,
          -18},{68,-18}},       color={0,0,255}));
  connect(inductor1.n, ground1.p) annotation (Line(points={{88,8},{92,8},{92,
          -36},{44,-36}},      color={0,0,255}));
  connect(const4.y,prescribedTemperature4. T)
    annotation (Line(points={{-111,-30},{-100,-30}},
                                               color={0,0,127}));
  connect(prescribedTemperature4.port, fuelCell_EquationBased_DetailedRohm1.port_a)
    annotation (Line(points={{-78,-30},{-65,-30},{-65,-8}},     color={191,0,0}));
  connect(inverter1.dc_p, resistor1.p) annotation (Line(points={{40,-12},{30,
          -12},{30,-4},{16,-4},{16,-8}},        color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1, HTS.pin_p) annotation (Line(
        points={{-58,-0.3},{-58,0},{-40,0},{-40,20},{-29,20}}, color={0,0,255}));
  connect(HTS.pin_n, resistor1.p)
    annotation (Line(points={{-11,20},{16,20},{16,-8}}, color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature3.port)
    annotation (Line(points={{-19.8,24},{-19.8,38},{6,38}}, color={191,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-140,-60},{100,60}})),  Icon(
        coordinateSystem(extent={{-140,-60},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit;
