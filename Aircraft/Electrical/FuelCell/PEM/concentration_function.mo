within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model concentration_function

  Modelica.Blocks.Interfaces.RealInput I "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
equation
  y=-log((abs(1-I/25))+1.0e-100)*(4.3085e-5*T)*2.6;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end concentration_function;
