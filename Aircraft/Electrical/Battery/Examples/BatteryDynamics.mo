within CHEETA.Aircraft.Electrical.Battery.Examples;
model BatteryDynamics
  "Scaled battery pack with imposed electrical power"
  import Battery;
  extends DymolaModels.Icons.Basic.Example;
 parameter Modelica.Units.SI.Temperature T_test=313.15 "Test temperature";
 parameter Modelica.Units.SI.Temperature T_test20=293.15 "Test temperature";
 parameter Modelica.Units.SI.Temperature T_test0=273.15 "Test temperature";
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{-176,
            -96},{-156,-76}})));
public
  Modelica.Blocks.Sources.Step  step(
    height=9,
    startTime=10,
    offset=1)   annotation (Placement(transformation(extent={{-136,-46},{-156,-26}})));
  Battery.Packs.Scaled.ScaledPackCylindric scaledBatteryPack(
    N_serialCells=150,
    N_parallelCells=30,
    N_verticalElements=5,
    redeclare Battery.Cells.Variants.DemoCell3dDAF cell,
    SOC_init=1)
    annotation (Placement(transformation(extent={{-160,-10},{-132,18}})));
  Modelica.Electrical.Analog.Basic.VariableResistor resistor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-180,-36})));
  Battery.Packs.ThermalBoundaries.ConstantTemp constantTemp(
    packType=scaledBatteryPack.packType,
    N_x=scaledBatteryPack.N_x,
    N_y=scaledBatteryPack.N_y,
    N_z=scaledBatteryPack.N_z,
    T_top=T_test,
    T_bottom=T_test,
    T_front=T_test,
    T_back=T_test,
    T_left=T_test,
    T_right=T_test) annotation (Placement(transformation(extent={{-160,40},{-132,
            68}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(transformation(extent={{-30,-96},
            {-10,-76}})));
public
  Modelica.Blocks.Sources.Step  step1(
    height=9,
    startTime=10,
    offset=1)   annotation (Placement(transformation(extent={{10,-46},{-10,-26}})));
  Battery.Packs.Scaled.ScaledPackCylindric scaledBatteryPack1(
    N_serialCells=150,
    N_parallelCells=30,
    N_verticalElements=5,
    redeclare Battery.Cells.Variants.DemoCell3dDAF cell,
    SOC_init=1)
    annotation (Placement(transformation(extent={{-14,-10},{14,18}})));
  Modelica.Electrical.Analog.Basic.VariableResistor resistor1
                                                             annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-34,-36})));
  Battery.Packs.ThermalBoundaries.ConstantTemp constantTemp1(
    packType=scaledBatteryPack.packType,
    N_x=scaledBatteryPack.N_x,
    N_y=scaledBatteryPack.N_y,
    N_z=scaledBatteryPack.N_z,
    T_top=T_test20,
    T_bottom=T_test20,
    T_front=T_test20,
    T_back=T_test20,
    T_left=T_test20,
    T_right=T_test20)
                    annotation (Placement(transformation(extent={{-14,40},{14,68}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
                                                 annotation (Placement(transformation(extent={{110,-98},
            {130,-78}})));
public
  Modelica.Blocks.Sources.Step  step2(
    height=9,
    startTime=10,
    offset=1)   annotation (Placement(transformation(extent={{150,-48},{130,-28}})));
  Battery.Packs.Scaled.ScaledPackCylindric scaledBatteryPack2(
    N_serialCells=150,
    N_parallelCells=30,
    N_verticalElements=5,
    redeclare Battery.Cells.Variants.DemoCell3dDAF cell,
    SOC_init=1)
    annotation (Placement(transformation(extent={{126,-12},{154,16}})));
  Modelica.Electrical.Analog.Basic.VariableResistor resistor2
                                                             annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={106,-38})));
  Battery.Packs.ThermalBoundaries.ConstantTemp constantTemp2(
    packType=scaledBatteryPack.packType,
    N_x=scaledBatteryPack.N_x,
    N_y=scaledBatteryPack.N_y,
    N_z=scaledBatteryPack.N_z,
    T_top=T_test0,
    T_bottom=T_test0,
    T_front=T_test0,
    T_back=T_test0,
    T_left=T_test0,
    T_right=T_test0)
                    annotation (Placement(transformation(extent={{126,38},{154,66}})));
equation
  connect(scaledBatteryPack.n, ground.p) annotation (Line(points={{-132,4},{-104,
          4},{-104,-66},{-166,-66},{-166,-76}},            color={0,0,255}));
  connect(step.y, resistor.R)
    annotation (Line(points={{-157,-36},{-168,-36}},
                                                   color={0,0,127}));
  connect(resistor.p, scaledBatteryPack.p) annotation (Line(points={{-180,-26},{
          -180,4},{-160,4}},               color={0,0,255}));
  connect(resistor.n, ground.p) annotation (Line(points={{-180,-46},{-180,-66},{
          -166,-66},{-166,-76}},
                           color={0,0,255}));
  connect(scaledBatteryPack.housingHeatPort, constantTemp.housingHeatPort)
    annotation (Line(points={{-146,18},{-146,40}},
                                                 color={191,0,0}));
  connect(scaledBatteryPack1.n, ground1.p) annotation (Line(points={{14,4},{42,4},
          {42,-66},{-20,-66},{-20,-76}}, color={0,0,255}));
  connect(step1.y, resistor1.R)
    annotation (Line(points={{-11,-36},{-22,-36}}, color={0,0,127}));
  connect(resistor1.p, scaledBatteryPack1.p)
    annotation (Line(points={{-34,-26},{-34,4},{-14,4}}, color={0,0,255}));
  connect(resistor1.n, ground1.p) annotation (Line(points={{-34,-46},{-34,-66},{
          -20,-66},{-20,-76}}, color={0,0,255}));
  connect(scaledBatteryPack1.housingHeatPort, constantTemp1.housingHeatPort)
    annotation (Line(points={{0,18},{0,40}}, color={191,0,0}));
  connect(scaledBatteryPack2.n,ground2. p) annotation (Line(points={{154,2},{182,
          2},{182,-68},{120,-68},{120,-78}},
                                         color={0,0,255}));
  connect(step2.y,resistor2. R)
    annotation (Line(points={{129,-38},{118,-38}}, color={0,0,127}));
  connect(resistor2.p,scaledBatteryPack2. p)
    annotation (Line(points={{106,-28},{106,2},{126,2}}, color={0,0,255}));
  connect(resistor2.n,ground2. p) annotation (Line(points={{106,-48},{106,-68},{
          120,-68},{120,-78}}, color={0,0,255}));
  connect(scaledBatteryPack2.housingHeatPort,constantTemp2. housingHeatPort)
    annotation (Line(points={{140,16},{140,38}},
                                             color={191,0,0}));
  annotation (
    experiment(
      StopTime=20,
      Interval=0.001,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>Power imposed on  a scaled battery pack.</p>
</html>"),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-120},{
            240,100}}),
                    graphics={
        Rectangle(extent={{-220,80},{-80,-100}}, lineColor={28,108,200}),
        Rectangle(extent={{-70,80},{70,-100}}, lineColor={28,108,200}),
        Rectangle(extent={{82,80},{222,-100}}, lineColor={28,108,200}),
        Text(
          extent={{-136,-74},{-86,-108}},
          textColor={28,108,200},
          textString="Cold plate = 40 C"),
        Text(
          extent={{14,-72},{64,-106}},
          textColor={28,108,200},
          textString="Cold plate = 20 C"),
        Text(
          extent={{166,-74},{216,-108}},
          textColor={28,108,200},
          textString="Cold plate = 0 C")}),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
      Evaluate=false,
      OutputCPUtime=false,
      OutputFlatModelica=false),
    Icon(coordinateSystem(extent={{-240,-120},{240,100}})),
    __Dymola_Commands(file=
          "Aircraft/Electrical/Battery/Examples/CompareBatteryVI.mos"
        "CompareBatteryVI", file=
          "Aircraft/Electrical/Battery/Examples/CompareBatteryRtotal.mos"
        "CompareBatteryRtotal"));
end BatteryDynamics;
