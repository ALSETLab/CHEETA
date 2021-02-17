within CHEETA.Aircraft.Electrical.Transformers;
model Ydd
  parameter Integer m=3 "Number of Phases";
  parameter Real R2 = 6.9143e-5 "Secondary Winding Resistance";
  parameter Real L2 = Modelica.Constants.eps "Secondary Winding Inductance";
  parameter Real R1 = 0.0019048 "Primary Winding Resistance";
  parameter Real L1 = Modelica.Constants.eps "Primary Winding Inductance";
  parameter Real Rm = 4761.9 "Magnetization Resistance";
  parameter Real Lm = 12.631 "Magnetization Inductance";
  parameter Real N = 200/(22*sqrt(3)) "Transformer Turn Ratio";

  Modelica.Electrical.Polyphase.Basic.Star starT(m=m) annotation (Placement(
        transformation(
        origin={-46,-44},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Electrical.Polyphase.Basic.Delta deltaT2(m=m) annotation (Placement(
        transformation(
        origin={28,44},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground groundT annotation (Placement(
        transformation(extent={{-56,-80},{-36,-60}})));
  Modelica.Electrical.Polyphase.Basic.Resistor R_sec1(m=m, R={R2,R2,R2})
    annotation (Placement(transformation(extent={{-14,44},{6,64}})));
  Modelica.Electrical.Polyphase.Basic.Delta deltaT1(m=m) annotation (Placement(
        transformation(
        origin={28,14},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Polyphase.Basic.Resistor R_sec2(m=m, R={R2,R2,R2})
    annotation (Placement(transformation(extent={{-14,14},{6,34}})));
  Modelica.Electrical.Polyphase.Basic.Resistor R_pr(m=m, R={R1,R1,R1})
    annotation (Placement(transformation(extent={{-114,30},{-94,50}})));
  Modelica.Electrical.Polyphase.Basic.Resistor R_m(m=m, R={Rm,Rm,Rm})
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-62,20})));
  Modelica.Electrical.Machines.BasicMachines.Components.IdealCore
                                              core(
    final m=m,
    final n12=N,
    final n13=N)  annotation (Placement(transformation(extent={{-46,20},{
            -26,40}})));
  Modelica.Electrical.Polyphase.Interfaces.PositivePlug Primary
    annotation (Placement(transformation(extent={{-172,-10},{-152,10}})));
  Modelica.Electrical.Polyphase.Interfaces.NegativePlug Secondary1
    annotation (Placement(transformation(extent={{94,50},{114,70}})));
  Modelica.Electrical.Polyphase.Interfaces.NegativePlug Secondary2
    annotation (Placement(transformation(extent={{94,-70},{114,-50}})));
equation
  connect(groundT.p,starT. pin_n)
    annotation (Line(points={{-46,-60},{-46,-54}}, color={0,0,255}));
  connect(core.plug_p2, R_sec1.plug_p) annotation (Line(points={{-26,40},
          {-26,54},{-14,54}}, color={0,0,255}));
  connect(deltaT2.plug_n, core.plug_n2)
    annotation (Line(points={{28,34},{-26,34}}, color={0,0,255}));
  connect(R_sec2.plug_p, core.plug_p3) annotation (Line(points={{-14,24},
          {-20,24},{-20,26},{-26,26}}, color={0,0,255}));
  connect(R_sec1.plug_n, deltaT2.plug_p)
    annotation (Line(points={{6,54},{28,54}}, color={0,0,255}));
  connect(deltaT1.plug_p, R_sec2.plug_n)
    annotation (Line(points={{28,24},{6,24}}, color={0,0,255}));
  connect(deltaT1.plug_n, core.plug_n3)
    annotation (Line(points={{28,4},{-26,4},{-26,20}}, color={0,0,255}));
  connect(R_pr.plug_n, core.plug_p1)
    annotation (Line(points={{-94,40},{-46,40}}, color={0,0,255}));
  connect(R_m.plug_p, core.plug_p1) annotation (Line(points={{-62,30},{
          -62,40},{-46,40}}, color={0,0,255}));
  connect(core.plug_n1, starT.plug_p)
    annotation (Line(points={{-46,20},{-46,-34}}, color={0,0,255}));
  connect(R_m.plug_n, starT.plug_p) annotation (Line(points={{-62,10},{
          -46,10},{-46,-34}},
                         color={0,0,255}));
  connect(R_pr.plug_p, Primary) annotation (Line(points={{-114,40},{-136,
          40},{-136,0},{-162,0}}, color={0,0,255}));
  connect(deltaT2.plug_p, Secondary1) annotation (Line(points={{28,54},{
          66,54},{66,60},{104,60}}, color={0,0,255}));
  connect(deltaT1.plug_p, Secondary2) annotation (Line(points={{28,24},{
          62,24},{62,-60},{104,-60}}, color={0,0,255}));
  connect(Primary, Primary)
    annotation (Line(points={{-162,0},{-162,0}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(extent={{-160,-100},{100,100}})),
    Icon(coordinateSystem(extent={{-160,-100},{100,100}}), graphics={
                             Polygon(
              points={{-92,62},{-72,42},{-72,-38},{-92,-58},{-92,62}},
              fillColor={135,135,135},
              fillPattern=FillPattern.VerticalCylinder),Polygon(
              points={{48,62},{28,42},{28,-38},{48,-58},{48,62}},
              fillColor={135,135,135},
              fillPattern=FillPattern.VerticalCylinder),Polygon(
              points={{-22,52},{-32,42},{-32,-38},{-22,-48},{-12,-38},{
              -12,42},{-22,52}},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={135,135,135}),Polygon(
              points={{-92,62},{48,62},{28,42},{-12,42},{-22,52},{-32,42},
              {-72,42},{-92,62}},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={135,135,135}),Polygon(
              points={{-92,-58},{48,-58},{28,-38},{-12,-38},{-22,-48},{
              -32,-38},{-72,-38},{-92,-58}},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={135,135,135}),Rectangle(
              extent={{-100,38},{-64,-34}},
              lineColor={128,0,255},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={128,0,255}),Rectangle(
              extent={{-106,30},{-58,-26}},
              lineColor={0,128,255},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={0,128,255}),Rectangle(
              extent={{-40,38},{-4,-34}},
              lineColor={128,0,255},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={128,0,255}),Rectangle(
              extent={{-46,30},{2,-26}},
              lineColor={0,128,255},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={0,128,255}),Rectangle(
              extent={{20,38},{56,-34}},
              lineColor={128,0,255},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={128,0,255}),Rectangle(
              extent={{14,30},{62,-26}},
              lineColor={0,128,255},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={0,128,255}),
                                  Text(
              extent={{128,-58},{-172,-98}},
          textString="Ydd
",        lineColor={0,0,0}),            Text(
              extent={{128,102},{-172,62}},
              lineColor={0,0,255},
              textString="%name"),       Rectangle(
          extent={{-124,104},{80,-100}},
          lineColor={255,0,0},
          pattern=LinePattern.Dash,
          lineThickness=0.5)}),
    experiment(
      __Dymola_NumberOfIntervals=50000,
      Tolerance=1e-05,
      __Dymola_fixedstepsize=1e-05,
      __Dymola_Algorithm="Dassl"));
end Ydd;
