within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model ActivePressure
  Modelica.Blocks.Math.Gain I_Idensity(k=5)
    annotation (Placement(transformation(extent={{-78,50},{-58,70}})));
  Modelica.Blocks.Math.Gain factor2(k=1.653)
    annotation (Placement(transformation(extent={{-46,50},{-26,70}})));
  Modelica.Blocks.Interfaces.RealInput I "Input signal connector"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Power power(base=1.334)
    annotation (Placement(transformation(extent={{-80,10},{-60,30}})));
  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,0},{-100,40}})));
  H20vT h20vT annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
  Modelica.Blocks.Math.Division division
    annotation (Placement(transformation(extent={{-4,72},{16,92}})));
  Modelica.Blocks.Math.Exp exp1
    annotation (Placement(transformation(extent={{36,72},{56,92}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{70,70},{90,90}})));
  AbsInv abs1 annotation (Placement(transformation(extent={{106,70},{126,90}})));
  Modelica.Blocks.Math.Product product2
    annotation (Placement(transformation(extent={{142,64},{162,84}})));
  Modelica.Blocks.Math.Gain factor(k=0.5)
    annotation (Placement(transformation(extent={{30,22},{50,42}})));
  Modelica.Blocks.Interfaces.RealOutput PH2 "Connector of Real output signal"
    annotation (Placement(transformation(extent={{180,64},{200,84}})));
  Modelica.Blocks.Math.Gain factor1(k=4.192)
    annotation (Placement(transformation(extent={{-34,-50},{-14,-30}})));
  Modelica.Blocks.Math.Division division1
    annotation (Placement(transformation(extent={{6,-56},{26,-36}})));
  Modelica.Blocks.Interfaces.RealInput Panode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Math.Division division2
    annotation (Placement(transformation(extent={{6,-88},{26,-68}})));
  Modelica.Blocks.Interfaces.RealInput Pcathode
    "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-140,-104},{-100,-64}})));
  Modelica.Blocks.Math.Exp exp2
    annotation (Placement(transformation(extent={{50,-56},{70,-36}})));
  Modelica.Blocks.Math.Product product3
    annotation (Placement(transformation(extent={{88,-62},{108,-42}})));
  AbsInv abs3
    annotation (Placement(transformation(extent={{120,-62},{140,-42}})));
  Modelica.Blocks.Math.Product product4
    annotation (Placement(transformation(extent={{152,-68},{172,-48}})));
  Modelica.Blocks.Interfaces.RealOutput PO2 "Connector of Real output signal"
    annotation (Placement(transformation(extent={{180,-68},{200,-48}})));
  Modelica.Blocks.Math.Division division3
    annotation (Placement(transformation(extent={{6,-22},{26,-2}})));
equation
  connect(I_Idensity.y, factor2.u)
    annotation (Line(points={{-57,60},{-48,60}}, color={0,0,127}));
  connect(I_Idensity.u, I)
    annotation (Line(points={{-80,60},{-120,60}}, color={0,0,127}));
  connect(power.u, T)
    annotation (Line(points={{-82,20},{-120,20}}, color={0,0,127}));
  connect(factor2.y, division.u1) annotation (Line(points={{-25,60},{-18,60},{
          -18,88},{-6,88}}, color={0,0,127}));
  connect(exp1.u, division.y)
    annotation (Line(points={{34,82},{17,82}}, color={0,0,127}));
  connect(exp1.y, product1.u1) annotation (Line(points={{57,82},{62,82},{62,86},
          {68,86}}, color={0,0,127}));
  connect(power.y, division.u2) annotation (Line(points={{-59,20},{-12,20},{-12,
          76},{-6,76}}, color={0,0,127}));
  connect(h20vT.u, T) annotation (Line(points={{-82,-10},{-94,-10},{-94,20},{
          -120,20}}, color={0,0,127}));
  connect(product1.y, abs1.u)
    annotation (Line(points={{91,80},{104,80}}, color={0,0,127}));
  connect(abs1.y, product2.u1)
    annotation (Line(points={{127,80},{140,80}}, color={0,0,127}));
  connect(h20vT.y, factor.u) annotation (Line(points={{-59,-10},{-6,-10},{-6,32},
          {28,32}}, color={0,0,127}));
  connect(factor.y, product2.u2) annotation (Line(points={{51,32},{104,32},{104,
          68},{140,68}}, color={0,0,127}));
  connect(product2.y, PH2)
    annotation (Line(points={{163,74},{190,74}}, color={0,0,127}));
  connect(factor1.u, I_Idensity.y) annotation (Line(points={{-36,-40},{-52,-40},
          {-52,60},{-57,60}}, color={0,0,127}));
  connect(factor1.y, division1.u1)
    annotation (Line(points={{-13,-40},{4,-40}}, color={0,0,127}));
  connect(division2.u1, h20vT.y) annotation (Line(points={{4,-72},{-44,-72},{
          -44,-10},{-59,-10}}, color={0,0,127}));
  connect(division2.u2, Pcathode)
    annotation (Line(points={{4,-84},{-120,-84}}, color={0,0,127}));
  connect(exp2.u, division1.y)
    annotation (Line(points={{48,-46},{27,-46}}, color={0,0,127}));
  connect(exp2.y, product3.u1)
    annotation (Line(points={{71,-46},{86,-46}}, color={0,0,127}));
  connect(product3.y, abs3.u)
    annotation (Line(points={{109,-52},{118,-52}}, color={0,0,127}));
  connect(abs3.y, product4.u1)
    annotation (Line(points={{141,-52},{150,-52}}, color={0,0,127}));
  connect(division2.y, product3.u2) annotation (Line(points={{27,-78},{72,-78},
          {72,-58},{86,-58}}, color={0,0,127}));
  connect(product4.u2, h20vT.y) annotation (Line(points={{150,-64},{-44,-64},{
          -44,-10},{-59,-10}}, color={0,0,127}));
  connect(product4.y, PO2)
    annotation (Line(points={{173,-58},{190,-58}}, color={0,0,127}));
  connect(division3.y, product1.u2) annotation (Line(points={{27,-12},{62,-12},
          {62,74},{68,74}}, color={0,0,127}));
  connect(division3.u1, factor.u) annotation (Line(points={{4,-6},{-2,-6},{-2,
          -10},{-6,-10},{-6,32},{28,32}}, color={0,0,127}));
  connect(division3.u2, Panode) annotation (Line(points={{4,-18},{-54,-18},{-54,
          -60},{-120,-60}}, color={0,0,127}));
  connect(division1.u2, division.u2) annotation (Line(points={{4,-52},{-12,-52},
          {-12,76},{-6,76}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{180,100}})), Diagram(coordinateSystem(preserveAspectRatio=
            false, extent={{-100,-100},{180,100}})));
end ActivePressure;
