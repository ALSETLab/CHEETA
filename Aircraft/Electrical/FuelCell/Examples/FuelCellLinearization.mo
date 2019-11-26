within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model FuelCellLinearization
  "Linearization of the fuel cell for model reduction"
  import FuelCell;
  SOFCFullStack sOFCFullStack
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Electrical.Analog.Sources.ConstantCurrent constantCurrent
    annotation (Placement(transformation(extent={{-10,16},{10,36}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{24,-32},{44,-12}})));
equation
  connect(constantCurrent.p, sOFCFullStack.pin_p) annotation (Line(points={{-10,
          26},{-12,26},{-12,8.4},{-5,8.4}}, color={0,0,255}));
  connect(constantCurrent.n, sOFCFullStack.pin_n) annotation (Line(points={{10,
          26},{16,26},{16,8.4},{5,8.4}}, color={0,0,255}));
  connect(ground.p, sOFCFullStack.pin_n) annotation (Line(points={{34,-12},{34,
          2},{18,2},{18,8},{16,8},{16,8.4},{5,8.4}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelCellLinearization;
