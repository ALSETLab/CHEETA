within CHEETA.Aircraft.Electrical.HTS;
model PiLine "Pi line model"
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
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-30})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C=C) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,-30})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-70,-68},{-50,-48}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{50,-66},{70,-46}})));
  parameter Modelica.SIunits.Capacitance C=0 "Line capacitance";
equation
  connect(inductor.p, p1)
    annotation (Line(points={{-40,0},{-104,0}}, color={0,0,255}));
  connect(inductor.n, resistor.p)
    annotation (Line(points={{-20,0},{20,0}}, color={0,0,255}));
  connect(resistor.n, n1)
    annotation (Line(points={{40,0},{104,0}}, color={0,0,255}));
  connect(p1, capacitor.p)
    annotation (Line(points={{-104,0},{-60,0},{-60,-20}}, color={0,0,255}));
  connect(capacitor1.p, n1)
    annotation (Line(points={{60,-20},{60,0},{104,0}}, color={0,0,255}));
  connect(capacitor.n, ground.p)
    annotation (Line(points={{-60,-40},{-60,-48}}, color={0,0,255}));
  connect(capacitor1.n, ground1.p)
    annotation (Line(points={{60,-40},{60,-46}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PiLine;
