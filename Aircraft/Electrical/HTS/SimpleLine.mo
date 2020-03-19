within CHEETA.Aircraft.Electrical.HTS;
model SimpleLine
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}}),
        iconTransformation(extent={{-100,-10},{-80,10}})));
  parameter Modelica.SIunits.Inductance L=1e-6 "Line inductance";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1
                "Negative electrical pin"
    annotation (Placement(transformation(extent={{80,-10},{100,10}}),
        iconTransformation(extent={{80,-10},{100,10}})));
  parameter Modelica.SIunits.Resistance R=0
    "Line resistance at temperature T_ref";
equation
  connect(inductor.p, p1)
    annotation (Line(points={{-40,0},{-66,0},{-66,0},{-90,0}},
                                                color={0,0,255}));
  connect(inductor.n, resistor.p)
    annotation (Line(points={{-20,0},{20,0}}, color={0,0,255}));
  connect(resistor.n, n1)
    annotation (Line(points={{40,0},{90,0}},  color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -40},{100,40}}), graphics={
                  Rectangle(
          extent={{-80,40},{80,-40}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),Rectangle(
          extent={{-60,20},{60,-20}},
          lineColor={0,0,255},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),Text(
          extent={{-60,20},{60,-20}},
          lineColor={255,255,0},
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,40}})));
end SimpleLine;
