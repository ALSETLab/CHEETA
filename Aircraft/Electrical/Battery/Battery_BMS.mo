within CHEETA.Aircraft.Electrical.Battery;
model Battery_BMS
  import Battery;
  parameter Real batteryVoltage = 900 "Minimum bus voltage needed to turn on the battery";
  Battery.Packs.Scaled.ScaledPackCylindric batteryPack(
    redeclare Battery.Cells.Variants.DemoCell3dDAF cell(redeclare replaceable
        Battery.Cells.Thermal.Variants.CylindricDiscretizedMaterialBased
        thermalModel(
        redeclare Battery.Common.Material.Data.Aluminium pinMaterial,
        redeclare Battery.Common.Material.Data.Aluminium sheetMaterial,
        redeclare Battery.Common.Material.Data.LithiumIonSanyo coreMaterial,
        D=0.0181,
        height=0.0648,
        sheetThickness=5e-05,
        positivePinDiameter=0.009,
        negativePinDiameter=0.009,
        positivePinHeight=0.004,
        negativePinHeight=0.004), redeclare replaceable
        Battery.Cells.Electric.Variants.ElectricTableBased3dDAF electricModel(
        fileName=Modelica.Utilities.Files.loadResource(
            "modelica://Battery/Resources/Data/DemoRoundCell_3dDAF.mat"),
        useChargeConservation=false,
        readFromFile=true,
        C_nominalOriginal=3*3600)),
    N_serialCells=5*16*3,
    N_parallelCells=2*20,
    redeclare Battery.Packs.Scaled.Housings.IdealHousingCylindric housing,
    SOC_init=1,
    T_init=293.15)
    annotation (Placement(transformation(extent={{-14,-18},{10,6}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature     [batteryPack.N_x, batteryPack.N_y]
    packBottomTemperatureBoundary(T=298.15)
                                  "Fixed housing temperature boundary"
    annotation (Placement(transformation(extent={{106,-22},{86,-2}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_x, batteryPack.N_y]
    packTopTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary on the top"
    annotation (Placement(transformation(extent={{106,8},{86,28}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packLeftRightTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary left and right"
    annotation (Placement(transformation(extent={{106,-52},{86,-32}})));
public
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packFrontBackTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary front and back"
    annotation (Placement(transformation(extent={{106,-82},{86,-62}})));
  BrushlessDCDrives.Inverter.Averaged averaged
    annotation (Placement(transformation(extent={{-12,-58},{8,-38}})));
  Modelica.Blocks.Sources.Constant const(k=0.9)
    annotation (Placement(transformation(extent={{-48,-40},{-28,-20}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{6,-50},{26,-30}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
    "Positive pin of the left port (potential p.v > n.v for positive voltage drop v)"
    annotation (Placement(transformation(extent={{-62,52},{-42,72}}),
        iconTransformation(extent={{-60,78},{-40,98}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 "Negative pin of the left port"
    annotation (Placement(transformation(extent={{38,52},{58,72}}),
        iconTransformation(extent={{40,78},{60,98}})));
  Modelica.Blocks.Interfaces.BooleanInput u1 annotation (Placement(
        transformation(
        extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-102,-26}),
                          iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,-10})));
protected
  Battery.Common.Interfaces.HousingHeatPort housingHeatPort(
    N_x=batteryPack.N_x,
    N_y=batteryPack.N_y,
    N_z=batteryPack.N_z,
    pinHeatTransfer=false) annotation (Placement(transformation(extent={{28,-6},
            {88,14}})));
equation
  connect(packTopTemperatureBoundary.port,housingHeatPort. top)
    annotation (Line(points={{86,18},{80,18},{80,4.05},{58.0625,4.05}},    color={191,0,0}));
  connect(batteryPack.housingHeatPort,housingHeatPort)
    annotation (Line(points={{-2,6},{-2,18},{58,18},{58,4}},   color={255,0,0}));
  connect(packBottomTemperatureBoundary.port,housingHeatPort. bottom)
    annotation (Line(points={{86,-12},{58.0625,-12},{58.0625,4.05}},                         color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. left)
    annotation (Line(points={{86,-42},{58.0625,-42},{58.0625,4.05}},                   color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. right)
    annotation (Line(points={{86,-42},{58.0625,-42},{58.0625,4.05}},         color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. front)
    annotation (Line(points={{86,-72},{58.0625,-72},{58.0625,4.05}},         color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. back)
    annotation (Line(points={{86,-72},{58.0625,-72},{58.0625,4.05}},                                    color={191,0,0}));
  connect(housingHeatPort, housingHeatPort)
    annotation (Line(points={{58,4},{58,4}}, color={191,0,0}));
  connect(averaged.pin_p, batteryPack.p) annotation (Line(points={{-8,-38},{-8,
          -26},{-20,-26},{-20,-6},{-14,-6}}, color={0,0,255}));
  connect(batteryPack.n, averaged.pin_n) annotation (Line(points={{10,-6},{16,
          -6},{16,-26},{4,-26},{4,-38}}, color={0,0,255}));
  connect(averaged.pin_p_Out, p1) annotation (Line(points={{7.8,-42},{26,-42},{
          26,40},{-52,40},{-52,62}}, color={0,0,255}));
  connect(averaged.pin_n_Out, n1) annotation (Line(points={{8,-54},{36,-54},{36,
          40},{48,40},{48,62}}, color={0,0,255}));
  connect(averaged.rotateCW, u1) annotation (Line(points={{-14,-48},{-66,-48},{
          -66,-26},{-102,-26}}, color={255,0,255}));
  connect(const.y, averaged.dutyCycleIn) annotation (Line(points={{-27,-30},{
          -20,-30},{-20,-42},{-14,-42}}, color={0,0,127}));
  connect(batteryPack.n, ground1.p)
    annotation (Line(points={{10,-6},{16,-6},{16,-30}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-100},{120,60}}),
                    graphics={
        Rectangle(
          extent={{-80,60},{80,-80}},
          lineColor={95,95,95},
          fillPattern=FillPattern.Solid,
          fillColor={215,215,215}),
        Line(
          points={{-50,90},{-50,60}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{50,90},{50,60}},
          color={95,95,95},
          smooth=Smooth.None),
        Text(
          extent={{-50,54},{-90,94}},
          lineColor={95,95,95},
          textString="+"),
        Text(
          extent={{92,56},{52,96}},
          lineColor={95,95,95},
          textString="-"),
        Line(
          points={{-60,-58},{-60,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-40,-58},{-40,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-20,-58},{-20,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{0,-58},{0,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{20,-58},{20,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{40,-58},{40,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-58},{60,22}},
          color={95,95,95},
          smooth=Smooth.None)}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-80,-100},{120,
            60}})),
    experiment(StopTime=10800, __Dymola_Algorithm="Dassl"));
end Battery_BMS;
