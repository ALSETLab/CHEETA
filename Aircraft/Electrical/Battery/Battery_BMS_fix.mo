within CHEETA.Aircraft.Electrical.Battery;
model Battery_BMS_fix
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
    annotation (Placement(transformation(extent={{-12,8},{12,32}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature     [batteryPack.N_x, batteryPack.N_y]
    packBottomTemperatureBoundary(T=298.15)
                                  "Fixed housing temperature boundary"
    annotation (Placement(transformation(extent={{108,4},{88,24}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_x, batteryPack.N_y]
    packTopTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary on the top"
    annotation (Placement(transformation(extent={{108,34},{88,54}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packLeftRightTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary left and right"
    annotation (Placement(transformation(extent={{108,-26},{88,-6}})));
public
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packFrontBackTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary front and back"
    annotation (Placement(transformation(extent={{108,-56},{88,-36}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
    "Positive pin of the left port (potential p.v > n.v for positive voltage drop v)"
    annotation (Placement(transformation(extent={{-60,78},{-40,98}}),
        iconTransformation(extent={{-60,78},{-40,98}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 "Negative pin of the left port"
    annotation (Placement(transformation(extent={{40,78},{60,98}}),
        iconTransformation(extent={{40,78},{60,98}})));
  BrushlessDCDrives.Inverter.Averaged averaged
    annotation (Placement(transformation(extent={{-10,-32},{10,-12}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=false)
    annotation (Placement(transformation(extent={{-46,-48},{-26,-28}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1
    annotation (Placement(transformation(extent={{-28,0},{-48,20}})));
  Modelica.Blocks.Sources.Constant const(k=0.9)
    annotation (Placement(transformation(extent={{-46,-68},{-26,-48}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{4,-20},{24,0}})));
  BMS bMS(N_parallelCells=batteryPack.N_parallelCells, N_cells=batteryPack.N_serialCells)
    annotation (Placement(transformation(extent={{-66,-24},{-46,-4}})));
protected
  Battery.Common.Interfaces.HousingHeatPort housingHeatPort(
    N_x=batteryPack.N_x,
    N_y=batteryPack.N_y,
    N_z=batteryPack.N_z,
    pinHeatTransfer=false) annotation (Placement(transformation(extent={{30,20},
            {90,40}})));
equation
  connect(packTopTemperatureBoundary.port,housingHeatPort. top)
    annotation (Line(points={{88,44},{82,44},{82,30.05},{60.0625,30.05}},  color={191,0,0}));
  connect(batteryPack.housingHeatPort,housingHeatPort)
    annotation (Line(points={{0,32},{0,44},{60,44},{60,30}},   color={255,0,0}));
  connect(packBottomTemperatureBoundary.port,housingHeatPort. bottom)
    annotation (Line(points={{88,14},{60.0625,14},{60.0625,30.05}},                          color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. left)
    annotation (Line(points={{88,-16},{60.0625,-16},{60.0625,30.05}},                  color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. right)
    annotation (Line(points={{88,-16},{60.0625,-16},{60.0625,30.05}},        color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. front)
    annotation (Line(points={{88,-46},{60.0625,-46},{60.0625,30.05}},        color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. back)
    annotation (Line(points={{88,-46},{60.0625,-46},{60.0625,30.05}},                                   color={191,0,0}));
  connect(housingHeatPort, housingHeatPort)
    annotation (Line(points={{60,30},{60,30}}, color={191,0,0}));
  connect(averaged.pin_p, batteryPack.p) annotation (Line(points={{-6,-12},{-6,
          0},{-18,0},{-18,20},{-12,20}}, color={0,0,255}));
  connect(batteryPack.n, averaged.pin_n) annotation (Line(points={{12,20},{18,
          20},{18,0},{6,0},{6,-12}}, color={0,0,255}));
  connect(averaged.pin_p_Out, p1) annotation (Line(points={{9.8,-16},{28,-16},{
          28,66},{-50,66},{-50,88}}, color={0,0,255}));
  connect(averaged.pin_n_Out, n1) annotation (Line(points={{10,-28},{38,-28},{
          38,66},{50,66},{50,88}}, color={0,0,255}));
  connect(averaged.rotateCW, booleanExpression.y) annotation (Line(points={{-12,
          -22},{-20,-22},{-20,-38},{-25,-38}}, color={255,0,255}));
  connect(batteryPack.n, ground.p)
    annotation (Line(points={{12,20},{18,20},{18,0},{14,0}}, color={0,0,255}));
  connect(averaged.dutyCycleIn, bMS.DC)
    annotation (Line(points={{-12,-16},{-12,-14},{-45,-14}}, color={0,0,127}));
  connect(bMS.packBus1, batteryPack.packBus) annotation (Line(
      points={{-56,-4},{-56,26},{-22,26},{-22,-2},{0,-2},{0,8}},
      color={83,189,255},
      thickness=0.5));
  connect(bMS.u, booleanExpression1.y) annotation (Line(points={{-68,-8},{-72,
          -8},{-72,10},{-49,10}}, color={255,0,255}));
  connect(bMS.pin, p1) annotation (Line(points={{-65.8,-14},{-74,-14},{-74,66},
          {-50,66},{-50,88}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},{120,80}}),
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
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},{120,
            80}})),
    experiment(StopTime=10800, __Dymola_Algorithm="Dassl"));
end Battery_BMS_fix;
