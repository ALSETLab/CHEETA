within CHEETA.Aircraft.Electrical.FuelCell;
model LumpedFuelCells
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a1
    annotation (Placement(transformation(extent={{-112,-10},{-92,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{92,-10},{112,10}})));
  FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm3(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-8,32},{6,46}})));
  FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-8,-34},{6,-20}})));
  FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm2(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-8,2},{6,16}})));
  FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm4(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-8,-60},{6,-46}})));
equation
  connect(fuelCell_EquationBased_DetailedRohm3.p1, p1) annotation (Line(points=
          {{6,39.7},{42,39.7},{42,0},{102,0}}, color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1, p1) annotation (Line(points=
          {{6,-26.3},{82,-26.3},{82,0},{102,0}}, color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm2.p1, p1) annotation (Line(points=
          {{6,9.7},{42,9.7},{42,0},{102,0}}, color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm4.p1, p1) annotation (Line(points=
          {{6,-52.3},{38,-52.3},{38,-52},{82,-52},{82,0},{102,0}}, color={0,0,
          255}));
  connect(fuelCell_EquationBased_DetailedRohm3.port_a, port_a1) annotation (
      Line(points={{-1,32},{-2,32},{-2,22},{-44,22},{-44,0},{-102,0}}, color={
          191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm2.port_a, port_a1)
    annotation (Line(points={{-1,2},{-2,2},{-2,0},{-102,0}}, color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a, port_a1) annotation (
      Line(points={{-1,-34},{0,-34},{0,-40},{-44,-40},{-44,0},{-102,0}}, color=
          {191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm4.port_a, port_a1) annotation (
      Line(points={{-1,-60},{0,-60},{0,-68},{-44,-68},{-44,0},{-102,0}}, color=
          {191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end LumpedFuelCells;
