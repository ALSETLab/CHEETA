within CHEETA.Aircraft.Electrical.HTS.Examples;
model HTSTest
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=10)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-30,30})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-10,-4},{10,16}})));
  Stekly.Stekly_TransmissionLosses
         stekly_TransmissionLosses(
                l=1, I_c0=1e-3,
    G_d=100,
    R_0=1e-6)
    annotation (Placement(transformation(extent={{-8,52},{8,44}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-64,70},{-44,90}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={26,34})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{-6,70},{-26,90}})));
equation
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-30,20},{-30,16},{0,16}}, color={0,0,255}));
  connect(constantVoltage.p, stekly_TransmissionLosses.pin_p)
    annotation (Line(points={{-30,40},{-30,48},{-9,48}}, color={0,0,255}));
  connect(stekly_TransmissionLosses.pin_n, resistor.p)
    annotation (Line(points={{9,48},{26,48},{26,44}}, color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{26,24},{26,16},{0,16}}, color={0,0,255}));
  connect(fixedTemperature.port, thermalConductor.port_b)
    annotation (Line(points={{-44,80},{-26,80}}, color={191,0,0}));
  connect(stekly_TransmissionLosses.port_a, thermalConductor.port_a)
    annotation (Line(points={{0,52},{0,80},{-6,80}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=10, __Dymola_Algorithm="Rkfix2"));
end HTSTest;
