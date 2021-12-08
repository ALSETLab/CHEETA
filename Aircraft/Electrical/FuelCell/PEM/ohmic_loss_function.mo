within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model ohmic_loss_function

  Modelica.Blocks.Interfaces.RealInput I "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
equation
  y= -1.3*((-0.0158 + 3.8e-5*T) - (3e-5)*I)*I;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ohmic_loss_function;
