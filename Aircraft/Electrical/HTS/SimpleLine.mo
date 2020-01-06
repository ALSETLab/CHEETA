within CHEETA.Aircraft.Electrical.HTS;
model SimpleLine
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
  parameter Modelica.SIunits.Inductance L=1e-6 "Line inductance";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1
                "Negative electrical pin"
    annotation (Placement(transformation(extent={{94,-10},{114,10}})));
  parameter Modelica.SIunits.Resistance R=0
    "Line resistance at temperature T_ref";
equation
  connect(inductor.p, p1)
    annotation (Line(points={{-40,0},{-104,0}}, color={0,0,255}));
  connect(inductor.n, resistor.p)
    annotation (Line(points={{-20,0},{20,0}}, color={0,0,255}));
  connect(resistor.n, n1)
    annotation (Line(points={{40,0},{104,0}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SimpleLine;
