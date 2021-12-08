within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model FuelCell
  TerminalV terminalV(Ncell=Ncell)
    annotation (Placement(transformation(extent={{-64,44},{-20,64}})));
  Modelica.Blocks.Interfaces.RealInput I "Input signal connector"
    annotation (Placement(transformation(extent={{-132,64},{-100,96}}),
        iconTransformation(extent={{-132,64},{-100,96}})));
  Modelica.Blocks.Interfaces.RealInput Troom "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-132,-96},{-100,-64}}),
        iconTransformation(extent={{-132,-96},{-100,-64}})));
  Modelica.Blocks.Interfaces.RealInput Panode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-132,14},{-100,46}}),
        iconTransformation(extent={{-132,14},{-100,46}})));
  Modelica.Blocks.Interfaces.RealInput Pcathode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-132,-44},{-100,-12}}),
        iconTransformation(extent={{-132,-44},{-100,-12}})));
  parameter Real Ncell = 48;
  parameter Real Acell = 3.2e-2;
  parameter Real h = 37.5;
  parameter Real Tinit = 307.7;
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-68,10},{-48,30}})));
  Modelica.Blocks.Math.Gain gain(k=h*Acell*Ncell/2)
    annotation (Placement(transformation(extent={{-26,10},{-6,30}})));
  Modelica.Blocks.Interfaces.RealOutput V "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,30},{120,50}}),
        iconTransformation(extent={{100,30},{120,50}})));
  Heat heat(Ncell=Ncell)
            annotation (Placement(transformation(extent={{-14,-32},{6,-12}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=Ncell*terminalV.dG)
    annotation (Placement(transformation(extent={{-66,-64},{-46,-44}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=I/(2.0*96487.0))
    annotation (Placement(transformation(extent={{-66,-76},{-46,-56}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=I)
    annotation (Placement(transformation(extent={{-46,-18},{-26,2}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=Troom)
    annotation (Placement(transformation(extent={{-46,-46},{-26,-26}})));
  Modelica.Blocks.Math.MultiSum multiSum(k={-1,-1,-1,1}, nu=4)
    annotation (Placement(transformation(extent={{26,28},{38,40}})));
  Modelica.Blocks.Sources.RealExpression Power(y=I*V)
    annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  Modelica.Blocks.Math.Gain gain1(k=1/(2.2e4))
    annotation (Placement(transformation(extent={{46,28},{58,40}})));
  Modelica.Blocks.Continuous.TransferFunction transferFunction(
    a={1,0},
    initType=Modelica.Blocks.Types.Init.InitialState,
    x_start={Tinit})
    annotation (Placement(transformation(extent={{68,28},{80,40}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=373, uMin=273)
    annotation (Placement(transformation(extent={{78,2},{86,10}})));
  Modelica.Blocks.Interfaces.RealOutput Tout "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-50},{120,-30}}),
        iconTransformation(extent={{100,-50},{120,-30}})));
  Modelica.Blocks.Sources.RealExpression fdbk(y=Tout)
    annotation (Placement(transformation(extent={{-94,10},{-74,30}})));
equation
  connect(terminalV.I, I)
    annotation (Line(points={{-66,62},{-66,80},{-116,80}}, color={0,0,127}));
  connect(terminalV.T, Troom) annotation (Line(points={{-66,56},{-78,56},{-78,-80},
          {-116,-80}}, color={0,0,127}));
  connect(terminalV.Panode, Panode) annotation (Line(points={{-66,50.8},{-96,50.8},
          {-96,30},{-116,30}}, color={0,0,127}));
  connect(terminalV.Pcathode, Pcathode) annotation (Line(points={{-66,46},{-94,46},
          {-94,-28},{-116,-28}},
                             color={0,0,127}));
  connect(feedback.u2, Troom)
    annotation (Line(points={{-58,12},{-58,-80},{-116,-80}}, color={0,0,127}));
  connect(feedback.y, gain.u)
    annotation (Line(points={{-49,20},{-28,20}}, color={0,0,127}));
  connect(terminalV.V, V) annotation (Line(points={{-19,61.4},{90,61.4},{90,40},
          {110,40}}, color={0,0,127}));
  connect(product1.u1, realExpression.y)
    annotation (Line(points={{-28,-54},{-45,-54}}, color={0,0,127}));
  connect(product1.u2, realExpression1.y)
    annotation (Line(points={{-28,-66},{-45,-66}}, color={0,0,127}));
  connect(heat.I, realExpression2.y)
    annotation (Line(points={{-16,-16},{-20,-16},{-20,-8},{-25,-8}},
                                                   color={0,0,127}));
  connect(heat.T_room, realExpression3.y)
    annotation (Line(points={{-16,-28},{-20,-28},{-20,-36},{-25,-36}},
                                                   color={0,0,127}));
  connect(Power.y, multiSum.u[1]) annotation (Line(points={{11,50},{14,50},{14,37.15},
          {26,37.15}}, color={0,0,127}));
  connect(gain.y, multiSum.u[2]) annotation (Line(points={{-5,20},{12,20},{12,32},
          {14,32},{14,35.05},{26,35.05}}, color={0,0,127}));
  connect(heat.qsl, multiSum.u[3]) annotation (Line(points={{7,-22},{16,-22},{16,
          32.95},{26,32.95}}, color={0,0,127}));
  connect(product1.y, multiSum.u[4]) annotation (Line(points={{-5,-60},{20,-60},
          {20,30.85},{26,30.85}},                 color={0,0,127}));
  connect(multiSum.y, gain1.u)
    annotation (Line(points={{39.02,34},{44.8,34}}, color={0,0,127}));
  connect(gain1.y, transferFunction.u)
    annotation (Line(points={{58.6,34},{66.8,34}}, color={0,0,127}));
  connect(limiter.y, Tout) annotation (Line(points={{86.4,6},{98,6},{98,-40},{110,
          -40}},color={0,0,127}));
  connect(limiter.u, transferFunction.y) annotation (Line(points={{77.2,6},{66,6},
          {66,18},{86,18},{86,34},{80.6,34}}, color={0,0,127}));
  connect(feedback.u1, fdbk.y)
    annotation (Line(points={{-66,20},{-73,20}}, color={0,0,127}));
  connect(heat.T, feedback.u1)
    annotation (Line(points={{-16,-22},{-66,-22},{-66,20}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
          Text(
          extent={{-90,32},{88,-26}},
          textColor={28,108,200},
          textString="Fuel cell")}),                             Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelCell;
