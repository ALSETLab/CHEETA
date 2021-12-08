within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model PotentialE
  parameter Real E_0= 1.229;
  ActivePressure activePressure
    annotation (Placement(transformation(extent={{-60,62},{-32,82}})));
  Modelica.Blocks.Interfaces.RealInput I "Input signal connector"
    annotation (Placement(transformation(extent={{-140,62},{-100,102}}),
        iconTransformation(extent={{-140,62},{-100,102}})));
  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,14},{-100,54}}),
        iconTransformation(extent={{-140,14},{-100,54}})));
  Modelica.Blocks.Interfaces.RealInput Pcathode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
  Modelica.Blocks.Interfaces.RealInput Panode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-140,-40},{-100,0}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{12,62},{32,82}})));
  Modelica.Blocks.Math.Sqrt sqrt1
    annotation (Placement(transformation(extent={{-16,60},{-4,72}})));
  Modelica.Blocks.Math.Log log1
    annotation (Placement(transformation(extent={{46,62},{66,82}})));
  Modelica.Blocks.Math.Add3 add3_1(k3=-1)
    annotation (Placement(transformation(extent={{126,36},{146,56}})));

  Modelica.Blocks.Interfaces.RealOutput E "Connector of Real output signal"
    annotation (Placement(transformation(extent={{160,36},{180,56}})));
  Modelica.Blocks.Math.Product product2
    annotation (Placement(transformation(extent={{82,56},{102,76}})));
  Modelica.Blocks.Math.Product product3
    annotation (Placement(transformation(extent={{12,34},{32,54}})));
  Modelica.Blocks.Sources.Constant R2F(k=4.3085e-5)
    annotation (Placement(transformation(extent={{-32,32},{-20,44}})));
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
  Modelica.Blocks.Math.Gain gain(k=0.85e-3)
    annotation (Placement(transformation(extent={{12,-10},{32,10}})));
  Modelica.Blocks.Sources.Constant T0(k=298)
    annotation (Placement(transformation(extent={{-48,-24},{-36,-12}})));
  Modelica.Blocks.Math.Gain gain1(k=0.163)
    annotation (Placement(transformation(extent={{54,-36},{74,-16}})));
  Modelica.Blocks.Math.Add add(k1=-1)
    annotation (Placement(transformation(extent={{86,-42},{106,-22}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{120,-60},{140,-40}})));
  Modelica.Blocks.Sources.Constant dG0(k=237.2e3)
    annotation (Placement(transformation(extent={{54,-62},{66,-50}})));
  Modelica.Blocks.Math.Product product4
    annotation (Placement(transformation(extent={{18,-80},{38,-60}})));
  Modelica.Blocks.Math.Product product5
    annotation (Placement(transformation(extent={{-18,-86},{2,-66}})));
  Modelica.Blocks.Interfaces.RealOutput dG "Connector of Real output signal"
    annotation (Placement(transformation(extent={{160,-60},{180,-40}})));
  Modelica.Blocks.Sources.Constant R(k=8.31)
    annotation (Placement(transformation(extent={{-50,-88},{-38,-76}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=log1.y)
    annotation (Placement(transformation(extent={{-18,-60},{2,-40}})));
  Modelica.Blocks.Sources.Constant const(k=E_0)
    annotation (Placement(transformation(extent={{84,40},{96,52}})));
equation
  connect(activePressure.I, I)
    annotation (Line(points={{-62,78},{-92,78},{-92,82},{-120,82}},
                                                  color={0,0,127}));
  connect(activePressure.T, T) annotation (Line(points={{-62,74},{-96,74},{-96,
          34},{-120,34}},
                      color={0,0,127}));
  connect(activePressure.PO2, sqrt1.u)
    annotation (Line(points={{-31,66.2},{-17.2,66}}, color={0,0,127}));
  connect(sqrt1.y, product1.u2)
    annotation (Line(points={{-3.4,66},{10,66}}, color={0,0,127}));
  connect(activePressure.PH2, product1.u1)
    annotation (Line(points={{-31,79.4},{-31,78},{10,78}}, color={0,0,127}));
  connect(product1.y, log1.u)
    annotation (Line(points={{33,72},{44,72}}, color={0,0,127}));
  connect(add3_1.y, E)
    annotation (Line(points={{147,46},{170,46}}, color={0,0,127}));
  connect(log1.y, product2.u1)
    annotation (Line(points={{67,72},{80,72}}, color={0,0,127}));
  connect(product2.y, add3_1.u1) annotation (Line(points={{103,66},{108,66},{108,
          54},{124,54}}, color={0,0,127}));
  connect(product2.u2, product3.y) annotation (Line(points={{80,60},{70,60},{70,
          44},{33,44}}, color={0,0,127}));
  connect(product3.u1, T) annotation (Line(points={{10,50},{-78,50},{-78,74},{
          -96,74},{-96,34},{-120,34}},
                                   color={0,0,127}));
  connect(R2F.y, product3.u2)
    annotation (Line(points={{-19.4,38},{10,38}}, color={0,0,127}));
  connect(feedback.y, gain.u)
    annotation (Line(points={{-11,0},{10,0}}, color={0,0,127}));
  connect(gain.y, add3_1.u3) annotation (Line(points={{33,0},{118,0},{118,38},{124,
          38}}, color={0,0,127}));
  connect(T0.y, feedback.u2) annotation (Line(points={{-35.4,-18},{-20,-18},{-20,
          -8}}, color={0,0,127}));
  connect(feedback.u1, T) annotation (Line(points={{-28,0},{-36,0},{-36,50},{
          -78,50},{-78,74},{-96,74},{-96,34},{-120,34}},
                                                     color={0,0,127}));
  connect(gain1.u, feedback.y) annotation (Line(points={{52,-26},{0,-26},{0,0},{
          -11,0}}, color={0,0,127}));
  connect(add.y, add1.u1) annotation (Line(points={{107,-32},{112,-32},{112,-44},
          {118,-44}}, color={0,0,127}));
  connect(add.u1, gain1.y)
    annotation (Line(points={{84,-26},{75,-26}}, color={0,0,127}));
  connect(dG0.y, add.u2) annotation (Line(points={{66.6,-56},{78,-56},{78,-38},{
          84,-38}}, color={0,0,127}));
  connect(add1.u2, product4.y) annotation (Line(points={{118,-56},{80,-56},{80,-70},
          {39,-70}}, color={0,0,127}));
  connect(product4.u2, product5.y)
    annotation (Line(points={{16,-76},{3,-76}}, color={0,0,127}));
  connect(add1.y, dG)
    annotation (Line(points={{141,-50},{170,-50}}, color={0,0,127}));
  connect(product5.u2, R.y)
    annotation (Line(points={{-20,-82},{-37.4,-82}}, color={0,0,127}));
  connect(realExpression.y, product4.u1) annotation (Line(points={{3,-50},{10,-50},
          {10,-64},{16,-64}}, color={0,0,127}));
  connect(product5.u1, T) annotation (Line(points={{-20,-70},{-36,-70},{-36,50},
          {-78,50},{-78,74},{-96,74},{-96,34},{-120,34}}, color={0,0,127}));
  connect(add3_1.u2, const.y)
    annotation (Line(points={{124,46},{96.6,46}}, color={0,0,127}));
  connect(activePressure.Panode, Panode) annotation (Line(points={{-62,66},{-92,
          66},{-92,-20},{-120,-20}}, color={0,0,127}));
  connect(activePressure.Pcathode, Pcathode) annotation (Line(points={{-62,63.6},
          {-62,-80},{-120,-80}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {160,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{160,100}})));
end PotentialE;
