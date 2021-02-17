within CHEETA.Aircraft.Electrical.Battery.Examples;
model BatteryCharging
  Modelica.Electrical.Analog.Sources.CosineVoltage cosineVoltage(
    V=100,
    f=0.01,
    offset=900) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-4,22})));
  Battery_FC_Charging                             battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-34.5,45.5})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-32,16},{-12,36}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1000)
    annotation (Placement(transformation(extent={{34,12},{54,32}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{52,-14},{72,6}})));
equation
  connect(battery_FC_Charging.n1,ground1. p)
    annotation (Line(points={{-25.0909,41.6176},{-22,41.6176},{-22,36}},
                                                             color={0,0,255}));
  connect(battery_FC_Charging.n1,cosineVoltage. n) annotation (Line(points={{
          -25.0909,41.6176},{-12,41.6176},{-12,22},{-10,22}},
                                                   color={0,0,255}));
  connect(booleanExpression.y,battery_FC_Charging. u1) annotation (Line(points={{-39,20},
          {-33.4545,20},{-33.4545,32.8824}},         color={255,0,255}));
  connect(cosineVoltage.p, resistor.p)
    annotation (Line(points={{2,22},{34,22}}, color={0,0,255}));
  connect(battery_FC_Charging.p1, resistor.p) annotation (Line(points={{
          -25.0909,51.3235},{16,51.3235},{16,22},{34,22}}, color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{54,22},{62,22},{62,6}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BatteryCharging;
