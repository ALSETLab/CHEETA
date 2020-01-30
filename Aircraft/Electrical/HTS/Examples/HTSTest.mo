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
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=10) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={36,46})));
  Modelica.Blocks.Sources.Constant const1(k=10)
    annotation (Placement(transformation(extent={{-26,72},{-6,92}})));
equation
  connect(hTS.pin_p, constantVoltage.p)
    annotation (Line(points={{-5,60},{-30,60},{-30,58}}, color={0,0,255}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-30,38},{-30,34},{0,34}}, color={0,0,255}));
  connect(hTS.pin_n, resistor.p)
    annotation (Line(points={{13,60},{36,60},{36,56}}, color={0,0,255}));
  connect(ground.p, resistor.n)
    annotation (Line(points={{0,34},{36,34},{36,36}}, color={0,0,255}));
  connect(hTS.temperature, const1.y)
    annotation (Line(points={{4,66},{4,82},{-5,82}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HTSTest;
