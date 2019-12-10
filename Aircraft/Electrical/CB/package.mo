within CHEETA.Aircraft.Electrical;
package CB "Circuit breaker models"

annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
      Ellipse(
        extent={{-66,8},{-48,-10}},
        lineColor={0,0,0},
        lineThickness=1,
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{52,8},{70,-10}},
        lineColor={0,0,0},
        lineThickness=1,
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Line(
        points={{-66,0},{-98,0},{-100,0}},
        color={0,0,0},
        thickness=1),
      Line(
        points={{70,0},{98,0},{100,0}},
        color={0,0,0},
        thickness=1),
      Line(
        points={{-56,8},{-36,50},{4,72},{48,50},{62,8}},
        color={0,0,0},
        smooth=Smooth.Bezier,
        thickness=1)}));
end CB;
