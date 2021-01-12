within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit_GasCooling

  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{32,-54},{52,-34}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{24,30},{4,50}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{70,30},{50,50}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor1
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={14,-16})));
  Modelica.Blocks.Sources.Ramp     ramp1(
    height=-10000,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-38,-26},{-18,-6}})));
  PowerElectronics.Converters.DCAC.TheveninEquivalent inverter1(R=0.1, R_cm=0.1)
    annotation (Placement(transformation(extent={{38,-26},{58,-6}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor4(R=100)
    annotation (Placement(transformation(extent={{66,-26},{86,-6}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased1(R_ohm_current=0.17)
    annotation (Placement(transformation(extent={{-68,2},{-54,16}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=0.12086)
    annotation (Placement(transformation(extent={{66,-4},{86,16}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-100,-38},{-80,-18}})));
  Modelica.Blocks.Sources.Constant const4(k=353.15)
    annotation (Placement(transformation(extent={{-134,-38},{-114,-18}})));
  GasCooled.HTS_GasCooling_Voltage HTS1(
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
    annotation (Placement(transformation(extent={{-32,26},{-16,18}})));
equation
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{49,40},{26,40}}, color={0,0,127}));
  connect(resistor1.n,ground1. p)
    annotation (Line(points={{14,-26},{14,-34},{42,-34}},   color={0,0,255}));
  connect(resistor1.R, ramp1.y)
    annotation (Line(points={{2,-16},{-17,-16}},   color={0,0,127}));
  connect(inverter1.dc_n, ground1.p) annotation (Line(points={{38,-22},{26,-22},
          {26,-34},{42,-34}},   color={0,0,255}));
  connect(inverter1.ac, resistor4.p)
    annotation (Line(points={{58,-16},{66,-16}},   color={0,0,255}));
  connect(resistor4.n,ground1. p) annotation (Line(points={{86,-16},{90,-16},{
          90,-34},{42,-34}},   color={0,0,255}));
  connect(inductor1.p, resistor4.p) annotation (Line(points={{66,6},{62,6},{62,
          -16},{66,-16}},       color={0,0,255}));
  connect(inductor1.n, ground1.p) annotation (Line(points={{86,6},{90,6},{90,
          -34},{42,-34}},      color={0,0,255}));
  connect(const4.y,prescribedTemperature4. T)
    annotation (Line(points={{-113,-28},{-102,-28}},
                                               color={0,0,127}));
  connect(prescribedTemperature4.port, fuelCell_EquationBased1.port_a)
    annotation (Line(points={{-80,-28},{-61,-28},{-61,2}},      color={191,0,0}));
  connect(inverter1.dc_p, resistor1.p) annotation (Line(points={{38,-10},{24,
          -10},{24,8},{14,8},{14,-6}},          color={0,0,255}));
  connect(HTS1.port_a, prescribedTemperature3.port) annotation (Line(points={{
          -23.8,26},{-24,26},{-24,40},{4,40}}, color={191,0,0}));
  connect(fuelCell_EquationBased1.p1, HTS1.pin_p) annotation (Line(points={{-54,9.7},
          {-44,9.7},{-44,22},{-33,22}},      color={0,0,255}));
  connect(HTS1.pin_n, resistor1.p)
    annotation (Line(points={{-15,22},{14,22},{14,-6}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-140,-60},{100,60}})),  Icon(
        coordinateSystem(extent={{-140,-60},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit_GasCooling;
