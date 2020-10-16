within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit_RoebelCable

  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{36,-60},{56,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{28,24},{8,44}})));
  Modelica.Blocks.Sources.Constant const1(k=77)
    annotation (Placement(transformation(extent={{74,24},{54,44}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor3
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={18,-22})));
  Modelica.Blocks.Sources.Ramp     ramp(
    height=-10000,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-34,-32},{-14,-12}})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{42,-32},{62,-12}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=100)
    annotation (Placement(transformation(extent={{70,-32},{90,-12}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased_DetailedRohm
    fuelCell_EquationBased_DetailedRohm(E_0=200)
    annotation (Placement(transformation(extent={{-66,-10},{-52,4}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.12086)
    annotation (Placement(transformation(extent={{70,-6},{90,14}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature2
    annotation (Placement(transformation(extent={{-96,-42},{-76,-22}})));
  Modelica.Blocks.Sources.Constant const2(k=353.15)
    annotation (Placement(transformation(extent={{-130,-44},{-110,-24}})));
  LiquidCooled.HTS_filmboiling_Voltage HTS(
    l=10,
    n=5.29,
    I_c0=6000,
    A=2.835e-4,
    A_cu=5.04e-6,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 35270,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P(displayUnit="mm") = 0.1035)
    annotation (Placement(transformation(extent={{-26,2},{-10,-6}})));
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
  connect(const2.y,prescribedTemperature2. T)
    annotation (Line(points={{-109,-34},{-104,-34},{-104,-32},{-98,-32}},
                                               color={0,0,127}));
  connect(prescribedTemperature2.port, fuelCell_EquationBased_DetailedRohm.port_a)
    annotation (Line(points={{-76,-32},{-59,-32},{-59,-10}}, color={191,0,0}));
  connect(inverter.dc_p, resistor3.p) annotation (Line(points={{42,-16},{32,-16},
          {32,-2},{22,-2},{22,-12},{18,-12}}, color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature1.port) annotation (Line(points={{
          -17.8,2},{-4,2},{-4,34},{8,34}}, color={191,0,0}));
  connect(HTS.pin_n, resistor3.p) annotation (Line(points={{-9,-2},{22,-2},{22,
          -12},{18,-12}}, color={0,0,255}));
  connect(HTS.pin_p, fuelCell_EquationBased_DetailedRohm.p1) annotation (Line(
        points={{-27,-2},{-40,-2},{-40,-2.3},{-52,-2.3}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-140,-60},{100,60}})),  Icon(
        coordinateSystem(extent={{-140,-60},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit_RoebelCable;
