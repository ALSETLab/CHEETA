within CHEETA.Aircraft.Mechanical.Loads;
model Fan "Simple fan model"
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)
    annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  Modelica.Mechanics.Rotational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
                    "Flange of left shaft" annotation (Placement(transformation(
          extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{
            -40,10}})));
  parameter Modelica.SIunits.Inertia J=1 "Moment of inertia of the fan blades";
equation
  connect(inertia.flange_b, fixed.flange)
    annotation (Line(points={{12,0},{30,0}}, color={0,0,0}));
  connect(inertia.flange_a, flange_a1)
    annotation (Line(points={{-8,0},{-50,0}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
            {40,40}}), graphics={
        Rectangle(extent={{-40,40},{40,-40}}, lineColor={28,108,200}),
        Polygon(
          points={{0,-4},{4,16},{6,26},{0,32},{-6,26},{-4,16},{0,-4}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,-18},{4,2},{6,12},{0,18},{-6,12},{-4,2},{0,-18}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          origin={-14,0},
          rotation=90,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,-18},{4,2},{6,12},{0,18},{-6,12},{-4,2},{0,-18}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          origin={14,0},
          rotation=270,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,-18},{4,2},{6,12},{0,18},{-6,12},{-4,2},{0,-18}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          origin={0,-14},
          rotation=180,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-40,-40},{40,40}})));
end Fan;
