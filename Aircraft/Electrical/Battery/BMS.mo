within CHEETA.Aircraft.Electrical.Battery;
model BMS
  import Battery;
  parameter Real batteryVoltage = 900 "Minimum bus voltage needed to turn on the battery";
  parameter Real minSOC = 0.3 "minimum charge current";
  parameter Real nominalVoltage = 1000 "Nominal bus voltage";
public
  Battery.BMS.Variants.PerformanceAndObserverBMS exampleBMS(N_parallelCells=
        N_parallelCells, N_cells=N_cells)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,22})));
  Battery.BMS.Adapters.FromBus.MaxDischargeCurrent maxDischargeCurrent
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={32,-36})));
  Battery.BMS.Adapters.FromBus.MaxChargeCurrent maxChargeCurrent
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={32,-8})));
  Battery.Common.Interfaces.PackBus packBus1
    annotation (Placement(transformation(extent={{-20,80},{20,120}}),
        iconTransformation(extent={{-20,80},{20,120}})));
  Battery.Packs.Adapters.FromBus.MinSOC SOC annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={32,60})));
  Modelica.Electrical.Analog.Interfaces.Pin pin
    annotation (Placement(transformation(extent={{-108,-10},{-88,10}}),
        iconTransformation(extent={{-108,-10},{-88,10}})));
  parameter Integer N_parallelCells
    "number of parallel battery cells in the pack";
  parameter Integer N_cells
    "number of cells in the pack";
  Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-100,-20})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-110,-60},{-90,-40}})));
  Modelica.Blocks.Interfaces.RealOutput i
    annotation (Placement(transformation(extent={{100,-10},{120,10}}),
        iconTransformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.BooleanInput u
    annotation (Placement(transformation(extent={{-140,40},{-100,80}}),
        iconTransformation(extent={{-140,40},{-100,80}})));
  Battery.Packs.Adapters.FromBus.MinVoltage minVoltage
    annotation (Placement(transformation(extent={{22,70},{42,90}})));
  Battery.Packs.Adapters.FromBus.MaxVoltage maxVoltage
    annotation (Placement(transformation(extent={{22,92},{42,112}})));
equation
  if SOC.y <= minSOC then
    i = maxChargeCurrent.y;
  elseif potentialSensor.phi <= 0.9*nominalVoltage then
    i = maxDischargeCurrent.y;
  elseif potentialSensor.phi >= 0.9*nominalVoltage and  potentialSensor.phi <= 1.1*nominalVoltage then
    i = 0;
  elseif potentialSensor.phi >= 1.1*nominalVoltage then
    i = maxChargeCurrent.y;
  elseif potentialSensor.phi <= batteryVoltage then
    i = maxDischargeCurrent.y;
  elseif u == false then
    i = maxDischargeCurrent.y;
  else
    i = 0;
  end if;
  connect(exampleBMS.bmsBus,maxChargeCurrent. bmsBus)
    annotation (Line(
      points={{0,12},{0,-8},{22,-8}},
      color={0,140,72},
      thickness=0.5));
  connect(maxDischargeCurrent.bmsBus,exampleBMS. bmsBus)
    annotation (Line(
      points={{22,-36},{0,-36},{0,12}},
      color={0,140,72},
      thickness=0.5));
  connect(exampleBMS.packBus, packBus1) annotation (Line(
      points={{0,32},{0,100}},
      color={83,189,255},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(exampleBMS.packBus, SOC.packBus) annotation (Line(
      points={{0,32},{0,60},{22,60}},
      color={83,189,255},
      thickness=0.5));
  connect(pin, potentialSensor.p)
    annotation (Line(points={{-98,0},{-80,0}},  color={0,0,255}));
  connect(pin, resistor.p)
    annotation (Line(points={{-98,0},{-100,0},{-100,-10}},  color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{-100,-30},{-100,-40}}, color={0,0,255}));
  connect(minVoltage.packBus, exampleBMS.packBus) annotation (Line(
      points={{22,80},{1.77636e-15,80},{1.77636e-15,32}},
      color={83,189,255},
      thickness=0.5));
  connect(exampleBMS.packBus, maxVoltage.packBus) annotation (Line(
      points={{0,32},{0,102},{22,102}},
      color={83,189,255},
      thickness=0.5));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}), graphics={
                Rectangle(
          extent={{-100,-100},{100,100}},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid), Rectangle(
          extent={{-98,-98},{98,98}},
          lineColor={0,0,0},
          pattern=LinePattern.Solid,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-70,-24},{-64,-20},{-64,2},{-72,-4},{-70,-24}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-76,0},{-68,4},{-52,0},{-60,-6},{-76,0}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{-60,-8},{-60,-6},{-76,0},{-72,-24},{-70,-24},{-72,-4},{-60,
              -8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{-46,-8},{-40,-4},{-40,18},{-48,12},{-46,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-52,16},{-44,20},{-28,16},{-36,10},{-52,16}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{-36,8},{-36,10},{-52,16},{-48,-8},{-46,-8},{-48,12},{-36,8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{-20,6},{-14,10},{-14,32},{-22,26},{-20,6}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-26,30},{-18,34},{-2,30},{-10,24},{-26,30}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{-10,22},{-10,24},{-26,30},{-22,6},{-20,6},{-22,26},{-10,22}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{-70,-8},{-70,-24},{-4,-42},{-4,-26},{-70,-8}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-4,-42},{66,18},{66,32},{-4,-26},{-4,-42}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={48,48,48}),
        Polygon(
          points={{-70,-8},{-4,-26},{66,32},{4,42},{-70,-8}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Polygon(
          points={{4,-30},{4,-32},{16,-34},{12,-56},{14,-58},{20,-32},{4,-30}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{4,-30},{12,-24},{28,-26},{20,-32},{4,-30}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{14,-58},{20,-54},{28,-26},{20,-32},{14,-58}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{28,-8},{28,-10},{40,-12},{36,-34},{38,-36},{44,-10},{28,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{38,-36},{44,-32},{52,-4},{44,-10},{38,-36}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{28,-8},{36,-2},{52,-4},{44,-10},{28,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{52,12},{52,10},{64,8},{60,-14},{62,-16},{68,10},{52,12}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{62,-16},{68,-12},{76,16},{68,10},{62,-16}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{52,12},{60,18},{76,16},{68,10},{52,12}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175})}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}})),
    experiment(StopTime=10800, __Dymola_Algorithm="Dassl"));
end BMS;
