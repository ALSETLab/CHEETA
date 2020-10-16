within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit_GasCooling

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
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased(R_ohm_current=0.03)
    annotation (Placement(transformation(extent={{-66,-6},{-52,8}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.12086)
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature2
    annotation (Placement(transformation(extent={{-96,-44},{-76,-24}})));
  Modelica.Blocks.Sources.Constant const2(k=353.15)
    annotation (Placement(transformation(extent={{-130,-44},{-110,-24}})));
  GasCooled.HTS_GasCooling_Voltage
                            HTS(
    l=10,
    n=20,
    I_c0=5000,
    A=0.1,
    A_cu=0.1,
    I_crit=3000,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d=10,
    a(displayUnit="cm") = 0.256,
    b(displayUnit="cm") = 0.363,
    P=0.1) annotation (Placement(transformation(extent={{-26,6},{-10,-2}})));

  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{36,-176},{56,-156}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{28,-92},{8,-72}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{74,-92},{54,-72}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor1
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={18,-138})));
  Modelica.Blocks.Sources.Ramp     ramp1(
    height=-9999.99,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-34,-148},{-14,-128}})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter1(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{42,-148},{62,-128}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor4(R=100)
    annotation (Placement(transformation(extent={{70,-148},{90,-128}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased1(R_ohm_current=0.1)
    annotation (Placement(transformation(extent={{-66,-122},{-52,-108}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=0.12086)
    annotation (Placement(transformation(extent={{70,-126},{90,-106}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-96,-160},{-76,-140}})));
  Modelica.Blocks.Sources.Constant const4(k=353.15)
    annotation (Placement(transformation(extent={{-130,-160},{-110,-140}})));
  GasCooled.HTS_GasCooling_Voltage
                            HTS1(
    l=10,
    n=20,
    I_c0=5000,
    A=0.1,
    A_cu=0.1,
    I_crit=3000,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d=10,
    a(displayUnit="cm") = 0.256,
    b(displayUnit="cm") = 0.363,
    P=0.1) annotation (Placement(transformation(extent={{-26,-110},{-10,-118}})));
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
  connect(inductor.p, resistor2.p) annotation (Line(points={{70,0},{66,0},{66,
          -22},{70,-22}}, color={0,0,255}));
  connect(inductor.n, ground2.p) annotation (Line(points={{90,0},{94,0},{94,-40},
          {46,-40}}, color={0,0,255}));
  connect(const2.y,prescribedTemperature2. T)
    annotation (Line(points={{-109,-34},{-98,-34}},
                                               color={0,0,127}));
  connect(prescribedTemperature2.port, fuelCell_EquationBased.port_a)
    annotation (Line(points={{-76,-34},{-59,-34},{-59,-6}}, color={191,0,0}));
  connect(HTS.port_a, prescribedTemperature1.port) annotation (Line(points={{
          -17.8,6},{-18,6},{-18,34},{8,34}}, color={191,0,0}));
  connect(fuelCell_EquationBased.p1, HTS.pin_p) annotation (Line(points={{-52,
          1.7},{-40,1.7},{-40,2},{-27,2}}, color={0,0,255}));
  connect(HTS.pin_n, resistor3.p)
    annotation (Line(points={{-9,2},{18,2},{18,-12}}, color={0,0,255}));
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{53,-82},{30,-82}},
                                               color={0,0,127}));
  connect(resistor1.n,ground1. p)
    annotation (Line(points={{18,-148},{18,-156},{46,-156}},color={0,0,255}));
  connect(resistor1.R, ramp1.y)
    annotation (Line(points={{6,-138},{-13,-138}}, color={0,0,127}));
  connect(inverter1.dc_n, ground1.p) annotation (Line(points={{42,-144},{30,-144},
          {30,-156},{46,-156}}, color={0,0,255}));
  connect(inverter1.ac, resistor4.p)
    annotation (Line(points={{62,-138},{70,-138}}, color={0,0,255}));
  connect(resistor4.n,ground1. p) annotation (Line(points={{90,-138},{94,-138},{
          94,-156},{46,-156}}, color={0,0,255}));
  connect(inductor1.p, resistor4.p) annotation (Line(points={{70,-116},{66,-116},
          {66,-138},{70,-138}}, color={0,0,255}));
  connect(inductor1.n, ground1.p) annotation (Line(points={{90,-116},{94,-116},{
          94,-156},{46,-156}}, color={0,0,255}));
  connect(const4.y,prescribedTemperature4. T)
    annotation (Line(points={{-109,-150},{-98,-150}},
                                               color={0,0,127}));
  connect(prescribedTemperature4.port, fuelCell_EquationBased1.port_a)
    annotation (Line(points={{-76,-150},{-59,-150},{-59,-122}}, color={191,0,0}));
  connect(HTS1.port_a, prescribedTemperature3.port) annotation (Line(points={{-17.8,
          -110},{-18,-110},{-18,-82},{8,-82}}, color={191,0,0}));
  connect(fuelCell_EquationBased1.p1, HTS1.pin_p) annotation (Line(points={{-52,
          -114.3},{-40,-114.3},{-40,-114},{-27,-114}}, color={0,0,255}));
  connect(HTS1.pin_n, resistor1.p)
    annotation (Line(points={{-9,-114},{18,-114},{18,-128}}, color={0,0,255}));
  connect(inverter1.dc_p, resistor1.p) annotation (Line(points={{42,-132},{28,
          -132},{28,-114},{18,-114},{18,-128}}, color={0,0,255}));
  connect(inverter.dc_p, resistor3.p) annotation (Line(points={{42,-16},{32,-16},
          {32,0},{18,0},{18,-12}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-140,-200},{100,60}})), Icon(
        coordinateSystem(extent={{-140,-200},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit_GasCooling;
