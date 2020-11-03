within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit_detailedFC

  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{36,-56},{56,-36}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{28,28},{8,48}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{74,28},{54,48}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor1
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={18,-18})));
  Modelica.Blocks.Sources.Ramp     ramp1(
    height=-10000,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-34,-28},{-14,-8}})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter1(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{42,-28},{62,-8}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor4(R=100)
    annotation (Placement(transformation(extent={{70,-28},{90,-8}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=0.12086)
    annotation (Placement(transformation(extent={{70,-2},{90,18}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-96,-40},{-76,-20}})));
  Modelica.Blocks.Sources.Constant const4(k=353.15)
    annotation (Placement(transformation(extent={{-130,-40},{-110,-20}})));
  LiquidCooled.HTS_filmboiling_Voltage_Hydrogen HTS1(
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
    C=1.59e-9) annotation (Placement(transformation(extent={{-26,6},{-10,-2}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased_DetailedRohm
    fuelCell_EquationBased_DetailedRohm(R_ohm0=0.02)
    annotation (Placement(transformation(extent={{-76,10},{-56,30}})));
equation
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{53,38},{30,38}}, color={0,0,127}));
  connect(resistor1.n,ground1. p)
    annotation (Line(points={{18,-28},{18,-36},{46,-36}},   color={0,0,255}));
  connect(resistor1.R, ramp1.y)
    annotation (Line(points={{6,-18},{-13,-18}}, color={0,0,127}));
  connect(inverter1.dc_n, ground1.p) annotation (Line(points={{42,-24},{30,-24},
          {30,-36},{46,-36}}, color={0,0,255}));
  connect(inverter1.ac, resistor4.p)
    annotation (Line(points={{62,-18},{70,-18}}, color={0,0,255}));
  connect(resistor4.n,ground1. p) annotation (Line(points={{90,-18},{94,-18},{
          94,-36},{46,-36}},   color={0,0,255}));
  connect(inductor1.p, resistor4.p) annotation (Line(points={{70,8},{66,8},{66,
          -18},{70,-18}}, color={0,0,255}));
  connect(inductor1.n, ground1.p) annotation (Line(points={{90,8},{94,8},{94,
          -36},{46,-36}}, color={0,0,255}));
  connect(const4.y,prescribedTemperature4. T)
    annotation (Line(points={{-109,-30},{-98,-30}},
                                               color={0,0,127}));
  connect(HTS1.port_a, prescribedTemperature3.port) annotation (Line(points={{
          -17.8,6},{-18,6},{-18,38},{8,38}}, color={191,0,0}));
  connect(HTS1.pin_n, resistor1.p)
    annotation (Line(points={{-9,2},{18,2},{18,-8}}, color={0,0,255}));
  connect(inverter1.dc_p, ground1.p) annotation (Line(points={{42,-12},{38,-12},
          {38,-10},{30,-10},{30,-36},{46,-36}}, color={0,0,255}));
  connect(HTS1.pin_p, fuelCell_EquationBased_DetailedRohm.p1) annotation (Line(
        points={{-27,2},{-42,2},{-42,21},{-56,21}}, color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm.port_a, prescribedTemperature4.port)
    annotation (Line(points={{-66,10},{-66,-30},{-76,-30}}, color={191,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-140,-80},{100,60}})),  Icon(
        coordinateSystem(extent={{-140,-80},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit_detailedFC;
