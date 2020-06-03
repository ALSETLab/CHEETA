within CHEETA.Aircraft.Electrical.Battery;
model Battery_FC_Charging
  import Battery;
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{-106,
            -106},{-86,-86}})));
  Battery.Packs.Scaled.ScaledPackCylindric batteryPack(
    N_verticalElements=3,
    SOC_init=0.6,
    redeclare Battery.Cells.Variants.DemoCell3dSDF cell,
    N_serialCells=250,
    N_parallelCells=5,
    redeclare Battery.Packs.Scaled.Housings.IdealHousingCylindric housing,
    T_init=300.15)
    annotation (Placement(transformation(extent={{4,50},{28,74}})));
  Modelica.Blocks.Logical.Switch switch
    "Switch between charging and discharging current"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-58,12})));
  Modelica.Blocks.Logical.Hysteresis cycling(
    pre_y_start=true,
    uLow=0.0,
    uHigh=1)   "True if Discharging" annotation (Placement(transformation(extent={{-24,-48},
            {-44,-28}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature     [batteryPack.N_x, batteryPack.N_y]
    packBottomTemperatureBoundary(T=298.15)
                                  "Fixed housing temperature boundary"
    annotation (Placement(transformation(extent={{108,-8},{88,12}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_x, batteryPack.N_y]
    packTopTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary on the top"
    annotation (Placement(transformation(extent={{108,32},{88,52}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packLeftRightTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary left and right"
    annotation (Placement(transformation(extent={{108,-48},{88,-28}})));
public
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packFrontBackTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary front and back"
    annotation (Placement(transformation(extent={{108,-88},{88,-68}})));
public
  Battery.BMS.Variants.PerformanceAndObserverBMS exampleBMS(N_parallelCells=
        batteryPack.N_parallelCells, N_cells=batteryPack.N_x*batteryPack.N_y)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-14,22})));
  Battery.BMS.Adapters.FromBus.MaxDischargeCurrent maxDischargeCurrent
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-34,-68})));
  Battery.BMS.Adapters.FromBus.MaxChargeCurrent maxChargeCurrent
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-32,-8})));
  Battery.Packs.Adapters.FromBus.MinSOC minSOC
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={16,22})));
  ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.CurrentInputConstantEfficiency
                                                         converterVoltageInput1(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.Records.Data.ConstantEfficiency.Constant100percent
      data)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-68,78})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
    "Positive pin of the left port (potential p.v > n.v for positive voltage drop v)"
    annotation (Placement(transformation(extent={{-60,90},{-40,110}}),
        iconTransformation(extent={{-60,90},{-40,110}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 "Negative pin of the left port"
    annotation (Placement(transformation(extent={{40,90},{60,110}}),
        iconTransformation(extent={{40,90},{60,110}})));
  Modelica.Blocks.Logical.LessEqual lessEqual annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-124,10})));
  Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor
    annotation (Placement(transformation(extent={{-90,-32},{-110,-12}})));
  Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1
    annotation (Placement(transformation(extent={{-88,-58},{-108,-38}})));
protected
  Battery.Common.Interfaces.HousingHeatPort housingHeatPort(
    N_x=batteryPack.N_x,
    N_y=batteryPack.N_y,
    N_z=batteryPack.N_z,
    pinHeatTransfer=false) annotation (Placement(transformation(extent={{30,90},
            {90,110}})));
equation
  connect(packTopTemperatureBoundary.port,housingHeatPort. top)
    annotation (Line(points={{88,42},{60.0625,42},{60.0625,100.05}},       color={191,0,0}));
  connect(batteryPack.housingHeatPort,housingHeatPort)
    annotation (Line(points={{16,74},{16,100},{60,100}},       color={255,0,0}));
  connect(packBottomTemperatureBoundary.port,housingHeatPort. bottom)
    annotation (Line(points={{88,2},{60.0625,2},{60.0625,100.05}},                           color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. left)
    annotation (Line(points={{88,-38},{60.0625,-38},{60.0625,100.05}},                 color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. right)
    annotation (Line(points={{88,-38},{60.0625,-38},{60.0625,100.05}},       color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. front)
    annotation (Line(points={{88,-78},{60.0625,-78},{60.0625,100.05}},       color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. back)
    annotation (Line(points={{88,-78},{60.0625,-78},{60.0625,100.05}},                                  color={191,0,0}));
  connect(batteryPack.packBus,exampleBMS. packBus)
    annotation (Line(
      points={{16,50},{16,42},{-14,42},{-14,32}},
      color={83,189,255},
      thickness=0.5));
  connect(exampleBMS.bmsBus,maxChargeCurrent. bmsBus)
    annotation (Line(
      points={{-14,12},{-14,-8},{-22,-8}},
      color={0,140,72},
      thickness=0.5));
  connect(maxDischargeCurrent.bmsBus,exampleBMS. bmsBus)
    annotation (Line(
      points={{-24,-68},{-14,-68},{-14,12}},
      color={0,140,72},
      thickness=0.5));
  connect(maxChargeCurrent.y,switch. u3) annotation (Line(points={{-43,-8},{-50,
          -8},{-50,0}},                                                                 color={0,0,0}));
  connect(maxDischargeCurrent.y,switch. u1) annotation (Line(points={{-45,-68},
          {-66,-68},{-66,0}},                                                                    color={0,0,0}));
  connect(batteryPack.packBus,minSOC. packBus)
    annotation (Line(
      points={{16,50},{16,32}},
      color={83,189,255},
      thickness=0.5));
  connect(minSOC.y,cycling. u) annotation (Line(points={{16,11},{16,-38},{-22,
          -38}},                                                                    color={0,127,0}));
  connect(batteryPack.n, ground.p) annotation (Line(points={{28,62},{38,62},{38,
          -84},{-96,-84},{-96,-86}},     color={0,0,255}));
  connect(converterVoltageInput1.p2, batteryPack.p)
    annotation (Line(points={{-58,84},{-28,84},{-28,62},{4,62}},
                                                 color={0,0,255}));
  connect(converterVoltageInput1.n2, ground.p) annotation (Line(points={{-58,72},
          {-8,72},{-8,44},{38,44},{38,-84},{-96,-84},{-96,-86}},     color={0,0,
          255}));
  connect(converterVoltageInput1.p1, p1) annotation (Line(points={{-78,84},{
          -106,84},{-106,100},{-50,100}}, color={0,0,255}));
  connect(converterVoltageInput1.n1, n1) annotation (Line(points={{-78,72},{-92,
          72},{-92,100},{50,100}}, color={0,0,255}));
  connect(lessEqual.u2, potentialSensor.phi) annotation (Line(points={{-116,-2},
          {-116,-22},{-111,-22}}, color={0,0,127}));
  connect(potentialSensor.p, batteryPack.p) annotation (Line(points={{-90,-22},
          {-80,-22},{-80,44},{-28,44},{-28,62},{4,62}}, color={0,0,255}));
  connect(switch.y, converterVoltageInput1.i) annotation (Line(points={{-58,23},
          {-60,23},{-60,66},{-60,66}}, color={0,0,127}));
  connect(potentialSensor1.p, p1) annotation (Line(points={{-88,-48},{-82,-48},
          {-82,84},{-106,84},{-106,100},{-50,100}}, color={0,0,255}));
  connect(potentialSensor1.phi, lessEqual.u1) annotation (Line(points={{-109,
          -48},{-124,-48},{-124,-2}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{120,
            120}}), graphics={
        Rectangle(
          extent={{-80,72},{80,-68}},
          lineColor={95,95,95},
          fillPattern=FillPattern.Solid,
          fillColor={215,215,215}),
        Line(
          points={{-50,102},{-50,72}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{50,102},{50,72}},
          color={95,95,95},
          smooth=Smooth.None),
        Text(
          extent={{-50,66},{-90,106}},
          lineColor={95,95,95},
          textString="+"),
        Text(
          extent={{92,68},{52,108}},
          lineColor={95,95,95},
          textString="-"),
        Line(
          points={{-60,-46},{-60,34}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-40,-46},{-40,34}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-20,-46},{-20,34}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{0,-46},{0,34}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{20,-46},{20,34}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{40,-46},{40,34}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-46},{60,34}},
          color={95,95,95},
          smooth=Smooth.None)}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
            120,120}})),
    experiment(StopTime=10800, __Dymola_Algorithm="Dassl"));
end Battery_FC_Charging;
