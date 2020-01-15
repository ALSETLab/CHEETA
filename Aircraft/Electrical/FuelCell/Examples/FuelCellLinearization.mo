within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model FuelCellLinearization
  "Linearization of the fuel cell for model reduction"
  import FuelCell;
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{24,-32},{44,-12}})));
equation
  connect(ground.p, sOFCFullStack.pin_n) annotation (Line(points={{34,-12},{34,
          2},{18,2},{18,8},{16,8},{16,8.4},{5,8.4}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelCellLinearization;
