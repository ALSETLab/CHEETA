within CHEETA.Aircraft.Electrical.Battery.Examples;
model BatteryCharging
  Battery_FC_Charging battery_FC_Charging(bms(N_parallelCells=
          battery_FC_Charging.batteryPack.N_parallelCells, N_cells=battery_FC_Charging.batteryPack.N_x*battery_FC_Charging.batteryPack.N_y))
    annotation (Placement(transformation(extent={{-26,-2},{2,20}})));
  Modelica.Electrical.Analog.Sources.CosineVoltage cosineVoltage(
    V=400,
    freqHz=0.1,
    offset=700) annotation (Placement(transformation(extent={{-18,26},{2,46}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-18,-40},{2,-20}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=100) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,2})));
  Modelica.Blocks.Sources.BooleanExpression FuelCell_State(y=true)
    annotation (Placement(transformation(extent={{100,0},{80,20}})));
equation
  connect(cosineVoltage.p, ground.p) annotation (Line(points={{-18,36},{-36,36},
          {-36,-20},{-8,-20}}, color={0,0,255}));
  connect(battery_FC_Charging.p1, ground.p) annotation (Line(points={{-15,18},{
          -16,18},{-16,24},{-36,24},{-36,-20},{-8,-20}}, color={0,0,255}));
  connect(cosineVoltage.n, resistor.p)
    annotation (Line(points={{2,36},{20,36},{20,12}}, color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{20,-8},{20,-20},{-8,-20}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, resistor.p) annotation (Line(points={{-5,18},
          {-4,18},{-4,22},{20,22},{20,12}}, color={0,0,255}));
  connect(battery_FC_Charging.u1, FuelCell_State.y)
    annotation (Line(points={{4,10},{79,10}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
end BatteryCharging;
