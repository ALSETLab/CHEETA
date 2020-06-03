within CHEETA.Architectures;
model Battery_Charging
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{138,-98},
            {158,-78}})));
  Battery.Packs.Scaled.ScaledPackCylindric batteryPack(
    N_verticalElements=3,
    SOC_init=0.6,
    redeclare Battery.Cells.Variants.DemoCell3dSDF cell,
    N_serialCells=250,
    N_parallelCells=4,
    redeclare
      Battery.Packs.Scaled.Housings.DynamicMaterialBasedHousingCylindric
      housing(
      redeclare Battery.Common.Material.Data.Aluminium housingMaterial,
      redeclare Battery.Common.Material.Data.Aluminium embeddingMaterial,
      thickness=0.005),
    T_init=300.15) annotation (Placement(transformation(extent={{248,58},{272,
            82}})));
  Modelica.Blocks.Logical.Switch switch
    "Switch between charging and discharging current"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={184,30})));
  Modelica.Blocks.Logical.Hysteresis cycling(
    pre_y_start=true,
    uLow=0.0,
    uHigh=1)   "True if Discharging" annotation (Placement(transformation(extent={{220,-40},
            {200,-20}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature     [batteryPack.N_x, batteryPack.N_y]
    packBottomTemperatureBoundary(T=298.15)
                                  "Fixed housing temperature boundary"
    annotation (Placement(transformation(extent={{352,0},{332,20}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_x, batteryPack.N_y]
    packTopTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary on the top"
    annotation (Placement(transformation(extent={{352,40},{332,60}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packLeftRightTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary left and right"
    annotation (Placement(transformation(extent={{352,-40},{332,-20}})));
public
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packFrontBackTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary front and back"
    annotation (Placement(transformation(extent={{352,-80},{332,-60}})));
public
  Battery.BMS.Variants.PerformanceAndObserverBMS exampleBMS(N_parallelCells=
        batteryPack.N_parallelCells, N_cells=batteryPack.N_x*batteryPack.N_y)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={230,30})));
  Battery.BMS.Adapters.FromBus.MaxDischargeCurrent maxDischargeCurrent
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={210,-60})));
  Battery.BMS.Adapters.FromBus.MaxChargeCurrent maxChargeCurrent
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={212,0})));
  Battery.Packs.Adapters.FromBus.MinSOC minSOC
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={260,30})));
  ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.CurrentInputConstantEfficiency
                                                         converterVoltageInput1(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.Records.Data.ConstantEfficiency.Constant100percent
      data)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={176,64})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=15)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={108,10})));
protected
  Battery.Common.Interfaces.HousingHeatPort housingHeatPort(
    N_x=batteryPack.N_x,
    N_y=batteryPack.N_y,
    N_z=batteryPack.N_z,
    pinHeatTransfer=false) annotation (Placement(transformation(extent={{274,98},
            {334,118}})));
equation
  connect(cycling.y,switch. u2)
    annotation (Line(
      points={{199,-30},{184,-30},{184,18}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(packTopTemperatureBoundary.port,housingHeatPort. top)
    annotation (Line(points={{332,50},{304.063,50},{304.063,108.05}},      color={191,0,0}));
  connect(batteryPack.housingHeatPort,housingHeatPort)
    annotation (Line(points={{260,82},{260,108},{304,108}},    color={255,0,0}));
  connect(packBottomTemperatureBoundary.port,housingHeatPort. bottom)
    annotation (Line(points={{332,10},{304.063,10},{304.063,108.05}},                        color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. left)
    annotation (Line(points={{332,-30},{304.063,-30},{304.063,108.05}},                color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. right)
    annotation (Line(points={{332,-30},{304.063,-30},{304.063,108.05}},      color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. front)
    annotation (Line(points={{332,-70},{304.063,-70},{304.063,108.05}},      color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. back)
    annotation (Line(points={{332,-70},{304.063,-70},{304.063,108.05}},                                 color={191,0,0}));
  connect(batteryPack.packBus,exampleBMS. packBus)
    annotation (Line(
      points={{260,58},{260,50},{230,50},{230,40}},
      color={83,189,255},
      thickness=0.5));
  connect(exampleBMS.bmsBus,maxChargeCurrent. bmsBus)
    annotation (Line(
      points={{230,20},{230,0},{222,0}},
      color={0,140,72},
      thickness=0.5));
  connect(maxDischargeCurrent.bmsBus,exampleBMS. bmsBus)
    annotation (Line(
      points={{220,-60},{230,-60},{230,20}},
      color={0,140,72},
      thickness=0.5));
  connect(maxChargeCurrent.y,switch. u3) annotation (Line(points={{201,0},{192,
          0},{192,18}},                                                                 color={0,0,0}));
  connect(maxDischargeCurrent.y,switch. u1) annotation (Line(points={{199,-60},
          {176,-60},{176,18}},                                                                   color={0,0,0}));
  connect(batteryPack.packBus,minSOC. packBus)
    annotation (Line(
      points={{260,58},{260,40}},
      color={83,189,255},
      thickness=0.5));
  connect(minSOC.y,cycling. u) annotation (Line(points={{260,19},{260,-30},{222,
          -30}},                                                                    color={0,127,0}));
  connect(batteryPack.n, ground.p) annotation (Line(points={{272,70},{282,70},{
          282,-76},{148,-76},{148,-78}}, color={0,0,255}));
  connect(converterVoltageInput1.p2, batteryPack.p)
    annotation (Line(points={{186,70},{248,70}}, color={0,0,255}));
  connect(converterVoltageInput1.n2, ground.p) annotation (Line(points={{186,58},
          {236,58},{236,52},{282,52},{282,-76},{148,-76},{148,-78}}, color={0,0,
          255}));
  connect(converterVoltageInput1.n1, ground.p)
    annotation (Line(points={{166,58},{148,58},{148,-78}}, color={0,0,255}));
  connect(constantVoltage.n, ground.p) annotation (Line(points={{108,
          -3.55271e-15},{108,-24},{148,-24},{148,-78}}, color={0,0,255}));
  connect(converterVoltageInput1.p1, constantVoltage.p)
    annotation (Line(points={{166,70},{108,70},{108,20}}, color={0,0,255}));
  connect(switch.y, converterVoltageInput1.i)
    annotation (Line(points={{184,41},{184,52}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
            -100},{380,120}})), Diagram(coordinateSystem(preserveAspectRatio=
            false, extent={{-140,-100},{380,120}})));
end Battery_Charging;
