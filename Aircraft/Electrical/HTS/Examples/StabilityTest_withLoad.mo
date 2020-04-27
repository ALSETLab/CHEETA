within CHEETA.Aircraft.Electrical.HTS.Examples;
model StabilityTest_withLoad
  Stekly.Stekly_TransmissionLosses_2port stekly_TransmissionLosses_2port1(
    l=1,
    G_d=100,
    R_0=0.1,
    R=100,
    I_crit=1000) annotation (Placement(transformation(extent={{-8,6},{8,14}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-10,-66},{10,-46}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage1(
    V=0.5,
    duration=5,
    offset=0.01)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,-4})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G=0.1)
           annotation (Placement(transformation(extent={{52,-38},{32,-18}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature1(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{82,-38},{62,-18}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
equation
  connect(thermalConductor1.port_a, fixedTemperature1.port)
    annotation (Line(points={{52,-28},{62,-28}}, color={191,0,0}));
  connect(thermalConductor1.port_b, stekly_TransmissionLosses_2port1.port_a)
    annotation (Line(points={{32,-28},{0,-28},{0,6}}, color={191,0,0}));
  connect(stekly_TransmissionLosses_2port1.p_in, rampVoltage1.p)
    annotation (Line(points={{-9,12},{-40,12},{-40,6}}, color={0,0,255}));
  connect(stekly_TransmissionLosses_2port1.n_in, rampVoltage1.n) annotation (
      Line(points={{-9,8},{-16,8},{-16,-20},{-40,-20},{-40,-14}}, color={0,0,
          255}));
  connect(stekly_TransmissionLosses_2port1.p_out, ground1.p) annotation (Line(
        points={{9,12},{22,12},{22,-46},{0,-46}}, color={0,0,255}));
  connect(stekly_TransmissionLosses_2port1.n_in, ground2.p) annotation (Line(
        points={{-9,8},{-16,8},{-16,-20},{-40,-20}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
end StabilityTest_withLoad;
