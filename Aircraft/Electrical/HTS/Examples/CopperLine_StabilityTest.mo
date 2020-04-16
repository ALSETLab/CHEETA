within CHEETA.Aircraft.Electrical.HTS.Examples;
model CopperLine_StabilityTest
  Stekly.Stekly_CopperLosses stekly_CopperLosses(
    l=1,
    n=15.2,
    I_c0=10,
    I_crit=1038) annotation (Placement(transformation(extent={{-8,4},{8,-4}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage(V=0.5, duration=5,
    offset=0)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,-22})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor(G=
        1) annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
equation
  connect(stekly_CopperLosses.pin_p, rampVoltage.p)
    annotation (Line(points={{-9,0},{-20,0},{-20,-12}}, color={0,0,255}));
  connect(rampVoltage.n, ground.p)
    annotation (Line(points={{-20,-32},{-20,-40},{0,-40}}, color={0,0,255}));
  connect(fixedTemperature.port, thermalConductor.port_a)
    annotation (Line(points={{-40,20},{-30,20}}, color={191,0,0}));
  connect(thermalConductor.port_b, stekly_CopperLosses.port_a)
    annotation (Line(points={{-10,20},{0,20},{0,4}}, color={191,0,0}));
  connect(stekly_CopperLosses.pin_n, ground.p)
    annotation (Line(points={{9,0},{22,0},{22,-40},{0,-40}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=5, __Dymola_Algorithm="Euler"));
end CopperLine_StabilityTest;
