within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model H20vT


  Modelica.Blocks.Interfaces.RealInput u "Input signal connector"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput y "Output signal connector"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation
  y = (-40529.4522405347 + 401.94033912153*u-1.33430273535993*(u^2) + 0.00148378443819808*(u^3))/760
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end H20vT;
