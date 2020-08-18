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
    fuelCell_EquationBased(R_ohm(R=0.05))
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
  annotation (Diagram(coordinateSystem(extent={{-140,-100},{100,60}})), Icon(
        coordinateSystem(extent={{-140,-100},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit_GasCooling;
