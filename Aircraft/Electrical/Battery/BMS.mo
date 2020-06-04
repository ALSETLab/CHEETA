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
        origin={0,20})));
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
    annotation (Placement(transformation(extent={{-20,100},{20,140}})));
  Battery.Packs.Adapters.FromBus.MinSOC SOC annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={32,60})));
  Modelica.Electrical.Analog.Interfaces.Pin pin
    annotation (Placement(transformation(extent={{-130,-10},{-110,10}})));
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
    annotation (Placement(transformation(extent={{120,-10},{140,10}})));
  Modelica.Blocks.Interfaces.BooleanInput u
    annotation (Placement(transformation(extent={{-162,40},{-122,80}})));
equation
  if SOC.y <= minSOC then
    i = maxChargeCurrent.y;
  elseif potentialSensor.phi <= 0.9*nominalVoltage then
    i = maxDischargeCurrent.y;
  elseif u == false then
    i = maxDischargeCurrent.y;
  else
    i = 0;
  end if;
  connect(exampleBMS.bmsBus,maxChargeCurrent. bmsBus)
    annotation (Line(
      points={{0,10},{0,-8},{22,-8}},
      color={0,140,72},
      thickness=0.5));
  connect(maxDischargeCurrent.bmsBus,exampleBMS. bmsBus)
    annotation (Line(
      points={{22,-36},{0,-36},{0,10}},
      color={0,140,72},
      thickness=0.5));
  connect(exampleBMS.packBus, packBus1) annotation (Line(
      points={{0,30},{0,120}},
      color={83,189,255},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(exampleBMS.packBus, SOC.packBus) annotation (Line(
      points={{0,30},{0,60},{22,60}},
      color={83,189,255},
      thickness=0.5));
  connect(pin, potentialSensor.p)
    annotation (Line(points={{-120,0},{-80,0}}, color={0,0,255}));
  connect(pin, resistor.p)
    annotation (Line(points={{-120,0},{-100,0},{-100,-10}}, color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{-100,-30},{-100,-40}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,120}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,
            120}})),
    experiment(StopTime=10800, __Dymola_Algorithm="Dassl"));
end BMS;
