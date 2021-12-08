within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model Heat
  parameter Real Ncell;
  Modelica.Blocks.Interfaces.RealInput I "Input signal connector"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}}),
        iconTransformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}}),
        iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealInput T_room "Input signal connector"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}}),
        iconTransformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{0,64},{20,84}})));
  Modelica.Blocks.Math.Product product2
    annotation (Placement(transformation(extent={{0,24},{20,44}})));
  Modelica.Blocks.Math.Product product3
    annotation (Placement(transformation(extent={{0,-36},{20,-16}})));
  Modelica.Blocks.Math.Gain CpH2(k=(1/(2.0*96487.0))*28.68)
    annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
  Modelica.Blocks.Math.Gain CpO2(k=29.39*(1/(4.0*96487.0)))
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
  Modelica.Blocks.Math.Gain CpH2O(k=75.4*(1/(2.0*96487.0)))
    annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  Modelica.Blocks.Math.Gain Hvaporization(k=40644*(1.0/(2.0*96487.0)))
    annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));
  Modelica.Blocks.Math.Add add(k1=-1, k2=2)
    annotation (Placement(transformation(extent={{-80,-8},{-60,12}})));
  Modelica.Blocks.Math.Add add1(k1=1, k2=-1)
    annotation (Placement(transformation(extent={{-80,-56},{-60,-36}})));
  Modelica.Blocks.Math.MultiSum multiSum(k={1,1,1,1}, nu=4)
    annotation (Placement(transformation(extent={{62,-6},{74,6}})));
  Modelica.Blocks.Math.Gain gain(k=Ncell)
    annotation (Placement(transformation(extent={{84,-6},{96,6}})));
  Modelica.Blocks.Interfaces.RealOutput qsl "Output signal connector"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation
  connect(I, CpH2.u)
    annotation (Line(points={{-120,60},{-82,60},{-82,80},{-42,80}},
                                                  color={0,0,127}));
  connect(product1.u1, CpH2.y)
    annotation (Line(points={{-2,80},{-19,80}}, color={0,0,127}));
  connect(product2.u1, CpO2.y)
    annotation (Line(points={{-2,40},{-19,40}}, color={0,0,127}));
  connect(CpO2.u, CpH2.u) annotation (Line(points={{-42,40},{-48,40},{-48,80},{-42,
          80}}, color={0,0,127}));
  connect(CpH2O.u, CpH2.u) annotation (Line(points={{-42,-20},{-48,-20},{-48,80},
          {-42,80}}, color={0,0,127}));
  connect(CpH2O.y, product3.u1)
    annotation (Line(points={{-19,-20},{-2,-20}}, color={0,0,127}));
  connect(Hvaporization.u, CpH2.u) annotation (Line(points={{-42,-60},{-48,-60},
          {-48,80},{-42,80}}, color={0,0,127}));
  connect(add.u1, T) annotation (Line(points={{-82,8},{-94,8},{-94,0},{-120,0}},
        color={0,0,127}));
  connect(add.u2, T_room) annotation (Line(points={{-82,-4},{-92,-4},{-92,-60},
          {-120,-60}},color={0,0,127}));
  connect(product2.u2, product1.u2) annotation (Line(points={{-2,28},{-12,28},{-12,
          68},{-2,68}}, color={0,0,127}));
  connect(add.y, product1.u2) annotation (Line(points={{-59,2},{-12,2},{-12,68},
          {-2,68}}, color={0,0,127}));
  connect(add1.u1, T) annotation (Line(points={{-82,-40},{-88,-40},{-88,8},{-94,
          8},{-94,0},{-120,0}},   color={0,0,127}));
  connect(add1.u2, T_room) annotation (Line(points={{-82,-52},{-92,-52},{-92,
          -60},{-120,-60}},
                       color={0,0,127}));
  connect(add1.y, product3.u2) annotation (Line(points={{-59,-46},{-50,-46},{-50,
          -74},{-8,-74},{-8,-32},{-2,-32}}, color={0,0,127}));
  connect(multiSum.u[1], product1.y) annotation (Line(points={{62,3.15},{54,3.15},
          {54,74},{21,74}}, color={0,0,127}));
  connect(product2.y, multiSum.u[2]) annotation (Line(points={{21,34},{30,34},{30,
          -2},{62,-2},{62,1.05}}, color={0,0,127}));
  connect(product3.y, multiSum.u[3]) annotation (Line(points={{21,-26},{46,-26},
          {46,-1.05},{62,-1.05}}, color={0,0,127}));
  connect(Hvaporization.y, multiSum.u[4]) annotation (Line(points={{-19,-60},{12,
          -60},{12,-58},{60,-58},{60,-3.15},{62,-3.15}}, color={0,0,127}));
  connect(multiSum.y, gain.u)
    annotation (Line(points={{75.02,0},{82.8,0}}, color={0,0,127}));
  connect(gain.y, qsl)
    annotation (Line(points={{96.6,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
          Text(
          extent={{-118,44},{110,-36}},
          textColor={28,108,200},
          textString="Heat")}),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Heat;
