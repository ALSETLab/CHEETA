within CHEETA.Aircraft.Electrical.Battery;
model Battery_noControl
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
    annotation (Placement(transformation(extent={{6,48},{30,72}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature     [batteryPack.N_x, batteryPack.N_y]
    packBottomTemperatureBoundary(T=298.15)
                                  "Fixed housing temperature boundary"
    annotation (Placement(transformation(extent={{112,30},{92,50}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_x, batteryPack.N_y]
    packTopTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary on the top"
    annotation (Placement(transformation(extent={{110,60},{90,80}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packLeftRightTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary left and right"
    annotation (Placement(transformation(extent={{112,-6},{92,14}})));
public
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature[batteryPack.N_y, batteryPack.N_z]
    packFrontBackTemperatureBoundary(each T(displayUnit="degC") = 298.15)
    "Fixed housing temperature boundary front and back"
    annotation (Placement(transformation(extent={{108,-88},{88,-68}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
    "Positive pin of the left port (potential p.v > n.v for positive voltage drop v)"
    annotation (Placement(transformation(extent={{-60,80},{-40,100}}),
        iconTransformation(extent={{-60,80},{-40,100}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 "Negative pin of the left port"
    annotation (Placement(transformation(extent={{40,80},{60,100}}),
        iconTransformation(extent={{40,80},{60,100}})));
protected
  Battery.Common.Interfaces.HousingHeatPort housingHeatPort(
    N_x=batteryPack.N_x,
    N_y=batteryPack.N_y,
    N_z=batteryPack.N_z,
    pinHeatTransfer=false) annotation (Placement(transformation(extent={{30,72},
            {90,92}})));
equation
  connect(packTopTemperatureBoundary.port,housingHeatPort. top)
    annotation (Line(points={{90,70},{60.0625,70},{60.0625,82.05}},        color={191,0,0}));
  connect(batteryPack.housingHeatPort,housingHeatPort)
    annotation (Line(points={{18,72},{18,82},{60,82}},         color={255,0,0}));
  connect(packBottomTemperatureBoundary.port,housingHeatPort. bottom)
    annotation (Line(points={{92,40},{60.0625,40},{60.0625,82.05}},                          color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. left)
    annotation (Line(points={{92,4},{60.0625,4},{60.0625,82.05}},                      color={191,0,0}));
  connect(packLeftRightTemperatureBoundary.port,housingHeatPort. right)
    annotation (Line(points={{92,4},{60.0625,4},{60.0625,82.05}},            color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. front)
    annotation (Line(points={{88,-78},{60.0625,-78},{60.0625,82.05}},        color={191,0,0}));
  connect(packFrontBackTemperatureBoundary.port,housingHeatPort. back)
    annotation (Line(points={{88,-78},{60.0625,-78},{60.0625,82.05}},                                   color={191,0,0}));
  connect(batteryPack.p, p1) annotation (Line(points={{6,60},{-24,60},{-24,68},
          {-50,68},{-50,90}}, color={0,0,255}));
  connect(n1, batteryPack.n)
    annotation (Line(points={{50,90},{50,60},{30,60}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},{80,60}}),
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
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},{80,
            60}})),
    experiment(StopTime=10800, __Dymola_Algorithm="Dassl"));
end Battery_noControl;
