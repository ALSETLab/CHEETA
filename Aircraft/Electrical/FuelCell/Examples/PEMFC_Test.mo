within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model PEMFC_Test
  PEMFC pEMFC(
    Tin_fuel(displayUnit="K") = 115,
    Tin_air(displayUnit="K") = 115,
    Tin_water(displayUnit="K") = 110) annotation (Placement(transformation(
        extent={{12,-8},{-12,8}},
        rotation=270,
        origin={-50,12})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=15)
    annotation (Placement(transformation(extent={{18,8},{38,28}})));
equation
  connect(resistor.p, pEMFC.pin_p1)
    annotation (Line(points={{18,18},{-43,18}}, color={0,0,255}));
  connect(resistor.n, pEMFC.pin_n1) annotation (Line(points={{38,18},{44,18},{
          44,6},{-43,6}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PEMFC_Test;
