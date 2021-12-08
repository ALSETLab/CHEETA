within CHEETA.Aircraft.Electrical.FuelCell.PEM;
block AbsInv "1/x-1"


  Inv inv annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  Modelica.Blocks.Interfaces.RealInput u "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Math.Abs abs
    annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation

  connect(inv.u, u)
    annotation (Line(points={{-72,0},{-120,0}}, color={0,0,127}));
  connect(inv.y, abs.u)
    annotation (Line(points={{-49,0},{36,0}}, color={0,0,127}));
  connect(abs.y, y) annotation (Line(points={{59,0},{110,0}}, color={0,0,127}));
  annotation (
    defaultComponentName="abs1",
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}), graphics={
        Polygon(
          points={{92,0},{70,8},{70,-8},{92,0}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,80},{0,0},{80,80}}),
        Line(points={{0,-14},{0,68}}, color={192,192,192}),
        Polygon(
          points={{0,90},{-8,68},{8,68},{0,90}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-34,-28},{38,-76}},
          textColor={192,192,192},
          textString="abs"),
        Line(points={{-88,0},{76,0}}, color={192,192,192})}),
    Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <em>absolute value</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>abs</strong>( u );
</pre></blockquote>
<p>
The Boolean parameter generateEvent decides whether Events are generated at zero crossing (Modelica specification before 3) or not.
</p>
</html>"));
end AbsInv;
