within CHEETA.Aircraft.Electrical.Interfaces;
model multiphtoabc
  Modelica.Electrical.Polyphase.Interfaces.PositivePlug m annotation (Placement(
        transformation(extent={{-106,-4},{-86,16}}), iconTransformation(extent=
            {{-106,-4},{-86,16}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin a
    annotation (Placement(transformation(extent={{94,86},{114,106}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin b
    annotation (Placement(transformation(extent={{94,-6},{114,14}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin c
    annotation (Placement(transformation(extent={{94,-106},{114,-86}})));
equation
  a.i = m.pin[1].i;
  a.v = m.pin[1].v;
  b.i = m.pin[2].i;
  b.v = m.pin[2].v;
  c.i = m.pin[3].i;
  c.v = m.pin[3].v;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-88,122},{44,42}},
          lineColor={28,108,200},
          textString="3ph-abc"),
        Line(points={{2,46}}, color={238,46,47}),
        Line(
          points={{-94,6},{94,90}},
          color={255,0,0},
          thickness=1),
        Line(
          points={{-92,2},{94,-94}},
          color={255,0,0},
          thickness=1),
        Line(
          points={{-94,4},{104,8}},
          color={255,0,0},
          thickness=1)}),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end multiphtoabc;
