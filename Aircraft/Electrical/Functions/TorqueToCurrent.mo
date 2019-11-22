within CHEETA.Aircraft.Electrical.Functions;
model TorqueToCurrent "Transform torque to current"
  Modelica.Blocks.Interfaces.RealInput T
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput I
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  parameter Real k "Torque constant";
equation
  I = T/k;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
        Line(points={{-100,-100},{100,100}}, color={28,108,200}),
        Text(
          extent={{-78,76},{-16,4}},
          lineColor={28,108,200},
          textString="T"),
        Text(
          extent={{34,-6},{88,-86}},
          lineColor={28,108,200},
          textString="I")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end TorqueToCurrent;
