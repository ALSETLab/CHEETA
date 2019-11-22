within CHEETA.Aircraft.Electrical.Functions;
model RotorToElectricalAngle
  "Change theta from the rotor angle to the electrical angle"
  parameter Real poles "Number of poles in the machine";
  Modelica.Blocks.Interfaces.RealInput theta_r
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput theta_e
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation
  theta_e = theta_r*(poles)/2;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
        Line(points={{-100,-100},{100,100}}, color={28,108,200}),
        Text(
          extent={{-78,76},{-16,4}},
          lineColor={28,108,200},
          textString="θr"),
        Text(
          extent={{34,-6},{88,-86}},
          lineColor={28,108,200},
          textString="θe")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end RotorToElectricalAngle;
