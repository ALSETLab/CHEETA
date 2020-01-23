within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model SimulinkFuelCellExample
  "Single branch model with averaged half bridge DCAC converter model"
  SimulinkFuelCell                                simulinkFuelCell(R=0, V=1000)
             annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={-156,116})));
  Records.NotionalPowerSystem.Plant plant(Vd=1000)
    annotation (Placement(transformation(extent={{140,102},{160,122}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1)
    annotation (Placement(transformation(extent={{-130,114},{-110,134}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=1)
    annotation (Placement(transformation(extent={{-130,98},{-110,118}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-114,82},{-94,102}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
    annotation (Placement(transformation(extent={{10,-80},{-10,-60}})));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={70,-10})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={-24,-92})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM(
    useConstantDutyCycle=true,
    constantDutyCycle=0.5,
    f=100)                             annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={-40,-20})));
  Modelica.Blocks.Math.Harmonic fundamentalWaveVoltage(f=60, k=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,50})));
  Modelica.Blocks.Math.Harmonic fundamentalWaveCurrent(f=60, k=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={30,-90})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=100)
                                                          annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,10})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_n(V=500)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-112,2})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_p(V=500)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-112,42})));
  Modelica.Electrical.Analog.Basic.Ground ground2
                                                 annotation (Placement(
        transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={-142,-2})));
equation
  connect(simulinkFuelCell.pin_p, resistor.p)
    annotation (Line(points={{-142,124},{-130,124}}, color={0,0,255}));
  connect(simulinkFuelCell.pin_p1, resistor1.p)
    annotation (Line(points={{-142,108},{-130,108}}, color={0,0,255}));
  connect(resistor1.n, ground.p) annotation (Line(points={{-110,108},{-104,108},
          {-104,102}}, color={0,0,255}));
  connect(resistor.n, ground.p) annotation (Line(points={{-110,124},{-104,124},
          {-104,102}}, color={0,0,255}));
  connect(voltageSensor.n,currentSensor. p) annotation (Line(
      points={{70,-20},{70,-70},{10,-70}}, color={0,0,255}));
  connect(signalPWM.fire,inverter. fire_p) annotation (Line(
      points={{-46,-9},{-46,18}}, color={255,0,255}));
  connect(currentSensor.n, ground1.p)
    annotation (Line(points={{-10,-70},{-24,-70},{-24,-84}}, color={0,0,255}));
  connect(voltageSensor.p,inverter. ac) annotation (Line(
      points={{70,0},{70,30},{-30,30}}, color={0,0,255}));
  connect(inverter.fire_n,signalPWM. notFire) annotation (Line(
      points={{-34,18},{-34,-9}}, color={255,0,255}));
  connect(currentSensor.i,fundamentalWaveCurrent. u) annotation (Line(
      points={{0,-81},{0,-90},{18,-90}}, color={0,0,127}));
  connect(voltageSensor.v,fundamentalWaveVoltage. u) annotation (Line(
      points={{81,-10},{90,-10},{90,38}}, color={0,0,127}));
  connect(resistor2.p, inverter.ac)
    annotation (Line(points={{40,20},{40,30},{-30,30}}, color={0,0,255}));
  connect(resistor2.n, currentSensor.p)
    annotation (Line(points={{40,0},{40,-70},{10,-70}}, color={0,0,255}));
  connect(constantVoltage_p.p, inverter.dc_p) annotation (Line(points={{-112,52},
          {-112,66},{-58,66},{-58,36},{-50,36}}, color={0,0,255}));
  connect(inverter.dc_n, constantVoltage_n.n) annotation (Line(points={{-50,24},
          {-82,24},{-82,-8},{-112,-8}}, color={0,0,255}));
  connect(constantVoltage_p.n, constantVoltage_n.p)
    annotation (Line(points={{-112,32},{-112,12}}, color={0,0,255}));
  connect(ground2.p, constantVoltage_n.p) annotation (Line(points={{-142,6},{
          -142,22},{-112,22},{-112,12}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,-140},{180,
            140}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-180,-140},{
            180,140}})),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=100));
end SimulinkFuelCellExample;
