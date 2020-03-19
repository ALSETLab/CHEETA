within CHEETA.Aircraft.Electrical.CB;
model CB_Trigger
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-34,0})));
  Modelica.Blocks.Logical.Greater greater
    annotation (Placement(transformation(extent={{12,-10},{32,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                           "positive pin"
    annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1
                           "negative pin"
    annotation (Placement(transformation(extent={{-70,-30},{-50,-10}})));
  Modelica.Blocks.Sources.Constant const(k=k)
    annotation (Placement(transformation(extent={{-24,-34},{-10,-20}})));
  parameter Real k "Reference overcurrent value";
  Modelica.Blocks.Interfaces.BooleanOutput y1
                                    "Connector of Boolean output signal"
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation
  connect(greater.u1, currentSensor.i) annotation (Line(points={{10,0},{-8,0},{
          -8,-6.66134e-16},{-23,-6.66134e-16}}, color={0,0,127}));
  connect(currentSensor.p, p1)
    annotation (Line(points={{-34,10},{-34,20},{-60,20}}, color={0,0,255}));
  connect(currentSensor.n, n1)
    annotation (Line(points={{-34,-10},{-34,-20},{-60,-20}}, color={0,0,255}));
  connect(greater.u2, const.y) annotation (Line(points={{10,-8},{0,-8},{0,-27},
          {-9.3,-27}}, color={0,0,127}));
  connect(greater.y, y1)
    annotation (Line(points={{33,0},{70,0}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-40},
            {60,40}}), graphics={Rectangle(extent={{-60,40},{60,-40}},
            lineColor={28,108,200})}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-60,-40},{60,40}})));
end CB_Trigger;
