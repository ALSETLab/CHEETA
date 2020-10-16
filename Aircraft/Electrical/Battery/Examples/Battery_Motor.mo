within CHEETA.Aircraft.Electrical.Battery.Examples;
model Battery_Motor "Battery powering the CHEETA powertrain system"
  Modelica.Blocks.Sources.Ramp tauRef(
    height=-733.038285,
    duration=200,
    offset=0,
    startTime=2800)
               annotation (Placement(transformation(extent={{-116,40},{-96,60}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                annotation (Placement(transformation(extent={{110,40},
            {122,52}})));
  Battery_FC_Charging                             battery_FC_Charging(
      batteryPack(N_serialCells=5500, N_parallelCells=10))
    annotation (Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-70.5,1.5})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-68,-28},{-48,-8}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-96,-34},{-76,-14}})));
  Modelica.Blocks.Sources.Ramp tauRef1(
    height=733.038285,
    duration=200,
    offset=1,
    startTime=0)
               annotation (Placement(transformation(extent={{-116,70},{-96,90}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-68,46},{-48,66}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.EnergyAnalyser machineAnalyser(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{56,-42},{36,-22}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{80,-4},
            {92,8}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{78,-40},{98,-20}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC speedFOC_ESM(
    useThermalPort=false,
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Records.Base.Speed
        data(redeclare
          CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
          machineData, tau_max=100)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant100percent
        data),
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{-4,-8},{16,12}})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
      tau_nominal=-7000, w_nominal(displayUnit="rpm") = 733.03828583762)
                                          annotation (Placement(transformation(extent={{128,-40},
            {108,-20}})));
equation
  connect(battery_FC_Charging.n1,ground9. p)
    annotation (Line(points={{-61.0909,-2.38235},{-58,-2.38235},{-58,-8}},
                                                             color={0,0,255}));
  connect(booleanExpression.y,battery_FC_Charging. u1) annotation (Line(points={{-75,-24},
          {-69.4545,-24},{-69.4545,-11.1176}},       color={255,0,255}));
  connect(tauRef1.y,add. u1) annotation (Line(points={{-95,80},{-82,80},{-82,62},
          {-70,62}},
                   color={0,0,127}));
  connect(add.u2,tauRef. y)
    annotation (Line(points={{-70,50},{-95,50}},
                                               color={0,0,127}));
  connect(inertia.flange_a,multiSensor. flange_b) annotation (Line(points={{78,-30},
          {70,-30},{70,-10},{106,-10},{106,2},{92,2}},         color={0,0,0}));
  connect(speedFOC_ESM.pin_p, battery_FC_Charging.p1) annotation (Line(points={
          {-4,8},{-32,8},{-32,7.32353},{-61.0909,7.32353}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, speedFOC_ESM.pin_n) annotation (Line(points={
          {-61.0909,-2.38235},{-32.5455,-2.38235},{-32.5455,-4},{-4,-4}}, color=
         {0,0,255}));
  connect(multiSensor.flange_a, speedFOC_ESM.flange)
    annotation (Line(points={{80,2},{16,2}}, color={0,0,0}));
  connect(speedFOC_ESM.electricDriveBus, machineAnalyser.electricDriveBus)
    annotation (Line(
      points={{6,-8},{6,-54},{46,-54},{46,-42}},
      color={0,86,166},
      thickness=0.5));
  connect(speedFOC_ESM.desiredSpeed, add.y)
    annotation (Line(points={{6,14},{8,14},{8,56},{-47,56}}, color={0,0,127}));
  connect(load.flange, inertia.flange_b)
    annotation (Line(points={{108,-30},{98,-30}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Battery_Motor;
