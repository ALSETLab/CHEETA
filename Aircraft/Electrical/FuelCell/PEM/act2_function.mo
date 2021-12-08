within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model act2_function

  Modelica.Blocks.Interfaces.RealInput I "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
equation
  y=((1.87e-4*log(I+1))*0.40)*T*1.3;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end act2_function;
