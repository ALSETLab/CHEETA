within CHEETA.Aircraft.Electrical.FuelCell;
model LumpedFuelCells_BB
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a FuelCell
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
  BusBar.Al_Bar_Generation al_Bar_Generation
    annotation (Placement(transformation(extent={{62,-10},{70,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a BB
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
equation
  connect(fuelCell_EquationBased_DetailedRohm3.port_a, FuelCell) annotation (
      Line(points={{-1,32},{-2,32},{-2,22},{-44,22},{-44,0},{-102,0}}, color={
          191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm2.port_a, FuelCell)
    annotation (Line(points={{-1,2},{-2,2},{-2,0},{-102,0}}, color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a, FuelCell) annotation (
      Line(points={{-1,-34},{0,-34},{0,-40},{-44,-40},{-44,0},{-102,0}}, color=
          {191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm4.port_a, FuelCell) annotation (
      Line(points={{-1,-60},{0,-60},{0,-68},{-44,-68},{-44,0},{-102,0}}, color=
          {191,0,0}));
  connect(al_Bar_Generation.FuelCell1, fuelCell_EquationBased_DetailedRohm3.p1)
    annotation (Line(points={{61,8.2},{36,8.2},{36,39.7},{6,39.7}}, color={0,0,
          255}));
  connect(al_Bar_Generation.FuelCell2, fuelCell_EquationBased_DetailedRohm2.p1)
    annotation (Line(points={{61,3.4},{34.5,3.4},{34.5,9.7},{6,9.7}}, color={0,
          0,255}));
  connect(al_Bar_Generation.FuelCell3, fuelCell_EquationBased_DetailedRohm1.p1)
    annotation (Line(points={{61,-4},{12,-4},{12,-26.3},{6,-26.3}}, color={0,0,
          255}));
  connect(al_Bar_Generation.FuelCell4, fuelCell_EquationBased_DetailedRohm4.p1)
    annotation (Line(points={{61,-8.2},{14,-8.2},{14,-52.3},{6,-52.3}}, color={
          0,0,255}));
  connect(al_Bar_Generation.port_a1, BB) annotation (Line(points={{62,0},{32,0},
          {32,86},{0,86},{0,100}}, color={191,0,0}));
  connect(al_Bar_Generation.HTS, p1)
    annotation (Line(points={{71,0},{102,0}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end LumpedFuelCells_BB;
