within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model TerminalV
  parameter Real C_dl = 4;
  parameter Real Ncell = 48;
  PotentialE potentialE
    annotation (Placement(transformation(extent={{-116,60},{-90,80}})));
  Modelica.Blocks.Interfaces.RealInput I "Input signal connector"
    annotation (Placement(transformation(extent={{-220,60},{-180,100}})));
  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-220,0},{-180,40}})));
  Modelica.Blocks.Interfaces.RealInput Panode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-220,-52},{-180,-12}})));
  Modelica.Blocks.Interfaces.RealInput Pcathode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-220,-100},{-180,-60}})));
  DropDelay dropDelay
    annotation (Placement(transformation(extent={{-114,20},{-94,40}})));
  Modelica.Blocks.Math.MultiSum multiSum(k={1,-1,-1,-1}, nu=4)
    annotation (Placement(transformation(extent={{120,68},{132,80}})));
  act1_function act1_function1
    annotation (Placement(transformation(extent={{-114,-4},{-94,16}})));
  act2_function act2_function1
    annotation (Placement(transformation(extent={{-114,-34},{-94,-14}})));
  concentration_function concentration_function1
    annotation (Placement(transformation(extent={{-112,-74},{-92,-54}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1, uMin=0)
    annotation (Placement(transformation(extent={{-76,-68},{-68,-60}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=I)
    annotation (Placement(transformation(extent={{-160,70},{-140,90}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=T)
    annotation (Placement(transformation(extent={{-160,56},{-140,76}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=Panode)
    annotation (Placement(transformation(extent={{-160,40},{-140,60}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=Pcathode)
    annotation (Placement(transformation(extent={{-160,26},{-140,46}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=T)
    annotation (Placement(transformation(extent={{-160,-38},{-140,-18}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=I)
    annotation (Placement(transformation(extent={{-160,-4},{-140,16}})));
  ohmic_loss_function ohmic_loss_function1
    annotation (Placement(transformation(extent={{-112,-96},{-92,-76}})));
  Modelica.Blocks.Math.Division division
    annotation (Placement(transformation(extent={{-48,-30},{-28,-10}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=0.001)
    annotation (Placement(transformation(extent={{-110,-50},{-100,-38}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{-82,-46},{-72,-36}})));
  Modelica.Blocks.Math.Division division1
    annotation (Placement(transformation(extent={{-48,-80},{-28,-60}})));
  Modelica.Blocks.Math.Add add2
    annotation (Placement(transformation(extent={{-10,-50},{0,-40}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=10, uMin=0.00001)
    annotation (Placement(transformation(extent={{12,-48},{20,-40}})));
  Modelica.Blocks.Math.Division division2
    annotation (Placement(transformation(extent={{20,-16},{0,4}})));
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-44,20},{-24,40}})));
  Modelica.Blocks.Math.Gain gain(k=1/C_dl)
    annotation (Placement(transformation(extent={{4,20},{24,40}})));
  Modelica.Blocks.Continuous.TransferFunction transferFunction(a={1,0},
      initType=Modelica.Blocks.Types.Init.InitialState)
    annotation (Placement(transformation(extent={{38,20},{58,40}})));
  Modelica.Blocks.Math.Gain gain1(k=Ncell)
    annotation (Placement(transformation(extent={{148,64},{168,84}})));
  Modelica.Blocks.Math.Feedback feedback1
    annotation (Placement(transformation(extent={{182,64},{202,84}})));
  Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=58,  uMin=0)
    annotation (Placement(transformation(extent={{214,70},{222,78}})));
  Modelica.Blocks.Interfaces.RealOutput V "Connector of Real output signal"
    annotation (Placement(transformation(extent={{260,64},{280,84}})));
  Modelica.Blocks.Interfaces.RealOutput dG "Connector of Real output signal"
    annotation (Placement(transformation(extent={{260,-70},{280,-50}})));
equation
  connect(concentration_function1.y, limiter.u)
    annotation (Line(points={{-91,-64},{-76.8,-64}}, color={0,0,127}));
  connect(potentialE.I, realExpression.y) annotation (Line(points={{-118,78.2},{
          -132,78.2},{-132,80},{-139,80}}, color={0,0,127}));
  connect(realExpression1.y, potentialE.T) annotation (Line(points={{-139,66},{-126,
          66},{-126,73.4},{-118,73.4}}, color={0,0,127}));
  connect(realExpression2.y, potentialE.Panode) annotation (Line(points={{-139,50},
          {-130,50},{-130,58},{-124,58},{-124,68},{-118,68}}, color={0,0,127}));
  connect(realExpression3.y, potentialE.Pcathode) annotation (Line(points={{-139,
          36},{-122,36},{-122,62},{-118,62}}, color={0,0,127}));
  connect(realExpression4.y, act2_function1.T)
    annotation (Line(points={{-139,-28},{-116,-28}}, color={0,0,127}));
  connect(concentration_function1.T, act2_function1.T) annotation (Line(points={
          {-114,-68},{-124,-68},{-124,-28},{-116,-28}}, color={0,0,127}));
  connect(ohmic_loss_function1.T, act2_function1.T) annotation (Line(points={{-114,
          -90},{-124,-90},{-124,-28},{-116,-28}},                       color={0,
          0,127}));
  connect(division.u1, act2_function1.y) annotation (Line(points={{-50,-14},{-84,
          -14},{-84,-24},{-93,-24}}, color={0,0,127}));
  connect(add1.u2, realExpression6.y)
    annotation (Line(points={{-83,-44},{-99.5,-44}}, color={0,0,127}));
  connect(add1.y, division.u2) annotation (Line(points={{-71.5,-41},{-60,-41},{-60,
          -26},{-50,-26}}, color={0,0,127}));
  connect(division1.u1, limiter.y)
    annotation (Line(points={{-50,-64},{-67.6,-64}}, color={0,0,127}));
  connect(division1.u2, add1.y) annotation (Line(points={{-50,-76},{-60,-76},{-60,
          -41},{-71.5,-41}}, color={0,0,127}));
  connect(add2.u1, division.y) annotation (Line(points={{-11,-42},{-16,-42},{-16,
          -20},{-27,-20}}, color={0,0,127}));
  connect(add2.u2, division1.y) annotation (Line(points={{-11,-48},{-16,-48},{-16,
          -70},{-27,-70}}, color={0,0,127}));
  connect(limiter1.u, add2.y) annotation (Line(points={{11.2,-44},{0.5,-44},{0.5,
          -45}}, color={0,0,127}));
  connect(limiter1.y, division2.u2) annotation (Line(points={{20.4,-44},{30,-44},
          {30,-12},{22,-12}}, color={0,0,127}));
  connect(division2.y, feedback.u2)
    annotation (Line(points={{-1,-6},{-34,-6},{-34,22}}, color={0,0,127}));
  connect(gain.u, feedback.y)
    annotation (Line(points={{2,30},{-25,30}}, color={0,0,127}));
  connect(feedback.u1, division.u2) annotation (Line(points={{-42,30},{-60,30},{
          -60,-26},{-50,-26}}, color={0,0,127}));
  connect(gain.y, transferFunction.u)
    annotation (Line(points={{25,30},{36,30}}, color={0,0,127}));
  connect(transferFunction.y, division2.u1)
    annotation (Line(points={{59,30},{68,30},{68,0},{22,0}}, color={0,0,127}));
  connect(multiSum.u[1], potentialE.E)
    annotation (Line(points={{120,77.15},{-89,74.6}}, color={0,0,127}));
  connect(act1_function1.y, multiSum.u[2]) annotation (Line(points={{-93,6},{-84,
          6},{-84,75.05},{120,75.05}}, color={0,0,127}));
  connect(multiSum.u[3], division2.u1) annotation (Line(points={{120,72.95},{94,
          72.95},{94,30},{68,30},{68,0},{22,0}}, color={0,0,127}));
  connect(ohmic_loss_function1.y, multiSum.u[4]) annotation (Line(points={{-91,-86},
          {114,-86},{114,70.85},{120,70.85}}, color={0,0,127}));
  connect(multiSum.y, gain1.u)
    annotation (Line(points={{133.02,74},{146,74}}, color={0,0,127}));
  connect(gain1.y, feedback1.u1)
    annotation (Line(points={{169,74},{184,74}}, color={0,0,127}));
  connect(feedback1.u2, dropDelay.Vdyn_d) annotation (Line(points={{192,66},{192,
          50},{-68,50},{-68,30},{-93,30}}, color={0,0,127}));
  connect(limiter2.u, feedback1.y)
    annotation (Line(points={{213.2,74},{201,74}}, color={0,0,127}));
  connect(limiter2.y, V)
    annotation (Line(points={{222.4,74},{270,74}}, color={0,0,127}));
  connect(potentialE.dG, dG) annotation (Line(points={{-89,65},{116,65},{116,-60},
          {270,-60}}, color={0,0,127}));
  connect(act1_function1.u, act2_function1.T) annotation (Line(points={{-116,6},
          {-124,6},{-124,-28},{-116,-28}}, color={0,0,127}));
  connect(dropDelay.I, realExpression5.y) annotation (Line(points={{-116,30},{
          -132,30},{-132,6},{-139,6}}, color={0,0,127}));
  connect(concentration_function1.I, realExpression5.y) annotation (Line(points=
         {{-114,-60},{-132,-60},{-132,6},{-139,6}}, color={0,0,127}));
  connect(act2_function1.I, realExpression5.y) annotation (Line(points={{-116,
          -20},{-132,-20},{-132,6},{-139,6}}, color={0,0,127}));
  connect(ohmic_loss_function1.I, realExpression5.y) annotation (Line(points={{
          -114,-82},{-132,-82},{-132,6},{-139,6}}, color={0,0,127}));
  connect(add1.u1, realExpression5.y) annotation (Line(points={{-83,-38},{-132,
          -38},{-132,6},{-139,6}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},
            {260,100}}), graphics={Rectangle(extent={{-180,100},{260,-100}},
            lineColor={28,108,200}), Text(
          extent={{-132,42},{216,-28}},
          textColor={28,108,200},
          textString="Terminal Voltage")}),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-180,-100},{260,100}})));
end TerminalV;
