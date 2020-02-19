within CHEETA.Aircraft.Electrical.HTS.Examples;
model HTSTest
  CHEETA.Aircraft.Electrical.HTS.HTS hTS(l=1)
    annotation (Placement(transformation(extent={{-4,64},{12,56}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=10)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-30,48})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-10,14},{10,34}})));
  Modelica.Blocks.Sources.Ramp     ramp(
    height=1000,
    duration=10,
    offset=60)
    annotation (Placement(transformation(extent={{-26,72},{-6,92}})));
equation
  connect(hTS.pin_p, constantVoltage.p)
    annotation (Line(points={{-5,60},{-30,60},{-30,58}}, color={0,0,255}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-30,38},{-30,34},{0,34}}, color={0,0,255}));
  connect(hTS.temperature, ramp.y)
    annotation (Line(points={{4,66},{4,82},{-5,82}}, color={0,0,127}));
  connect(ground.p, hTS.pin_n) annotation (Line(points={{0,34},{22,34},{22,60},
          {13,60}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=10));
end HTSTest;
