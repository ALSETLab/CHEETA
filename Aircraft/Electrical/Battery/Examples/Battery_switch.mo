within CHEETA.Aircraft.Electrical.Battery.Examples;
model Battery_switch "Battery powering the CHEETA powertrain system"
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM data))
               annotation (Placement(transformation(extent={{-2,12},{18,32}})));
  Modelica.Blocks.Sources.Constant const2(k=733.038285)
    annotation (Placement(transformation(extent={{40,38},{26,52}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM1(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM data))
               annotation (Placement(transformation(extent={{-2,-34},{18,-14}})));
  Modelica.Blocks.Sources.Constant const1(k=733.038285)
    annotation (Placement(transformation(extent={{40,-12},{26,2}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{12,106},{-8,126}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{64,106},{44,126}})));
  Modelica.Blocks.Sources.Constant const4(k=733.038285)
    annotation (Placement(transformation(extent={{40,-54},{26,-40}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1e-6)
    annotation (Placement(transformation(extent={{34,12},{54,32}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1e-6)
    annotation (Placement(transformation(extent={{34,-34},{54,-14}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{88,12},{68,32}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque2(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{88,-34},{68,-14}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM2(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM data))
               annotation (Placement(transformation(extent={{-2,-78},{18,-58}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1e-6)
    annotation (Placement(transformation(extent={{34,-78},{54,-58}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque3(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{88,-78},{68,-58}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-124,60},{-104,80}})));
  Modelica.Blocks.Sources.Constant const6(k=353.15)
    annotation (Placement(transformation(extent={{-156,60},{-136,80}})));
  FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-104,96},{-80,118}})));
  CHEETA.Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage_Hydrogen
                                                HTS(
    l=10,
    n=20,
    I_c0=3700,
    A=0.1,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm"),
    capacitor(v(start=1230)))
    annotation (Placement(transformation(extent={{-72,112},{-56,104}})));
  Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch
    annotation (Placement(transformation(extent={{-22,24},{-42,44}})));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=5, startValue=false)
    annotation (Placement(transformation(extent={{16,56},{-4,76}})));
  Battery_BMS_fix battery_BMS_fix annotation (Placement(transformation(
        extent={{-8,-7},{8,7}},
        rotation=270,
        origin={-82,-15})));
equation
  connect(const2.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{25.3,45},{8,45},{8,34}}, color={0,0,127}));
  connect(const1.y,speedFOC_ESM1. desiredSpeed)
    annotation (Line(points={{25.3,-5},{8,-5},{8,-12}},   color={0,0,127}));
  connect(speedFOC_ESM1.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{-2,-18},
          {-10,-18},{-10,28},{-2,28}},         color={0,0,255}));
  connect(inertia.flange_a,speedFOC_ESM. flange)
    annotation (Line(points={{34,22},{18,22}},color={0,0,0}));
  connect(inertia1.flange_a,speedFOC_ESM1. flange)
    annotation (Line(points={{34,-24},{18,-24}},
                                              color={0,0,0}));
  connect(constantTorque.flange,inertia. flange_b)
    annotation (Line(points={{68,22},{54,22}}, color={0,0,0}));
  connect(inertia1.flange_b,constantTorque2. flange)
    annotation (Line(points={{54,-24},{68,-24}},
                                               color={0,0,0}));
  connect(inertia2.flange_a,speedFOC_ESM2. flange)
    annotation (Line(points={{34,-68},{18,-68}},color={0,0,0}));
  connect(inertia2.flange_b,constantTorque3. flange)
    annotation (Line(points={{54,-68},{68,-68}}, color={0,0,0}));
  connect(const4.y,speedFOC_ESM2. desiredSpeed)
    annotation (Line(points={{25.3,-47},{8,-47},{8,-56}}, color={0,0,127}));
  connect(speedFOC_ESM2.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{-2,-62},
          {-10,-62},{-10,28},{-2,28}},           color={0,0,255}));
  connect(const6.y,prescribedTemperature4. T)
    annotation (Line(points={{-135,70},{-126,70}},
                                               color={0,0,127}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a,prescribedTemperature4. port)
    annotation (Line(points={{-92,96},{-92,70},{-104,70}},
                           color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1,HTS. pin_p) annotation (Line(
        points={{-80,108.1},{-73,108}},                       color={0,0,255}));
  connect(prescribedTemperature3.port,HTS. port_a) annotation (Line(points={{-8,116},
          {-63.8,116},{-63.8,112}},         color={191,0,0}));
  connect(const3.y, prescribedTemperature3.T)
    annotation (Line(points={{43,116},{14,116}}, color={0,0,127}));
  connect(speedFOC_ESM1.pin_n, speedFOC_ESM2.pin_n) annotation (Line(points={{
          -2,-30},{-28,-30},{-28,-74},{-2,-74}}, color={0,0,255}));
  connect(speedFOC_ESM.pin_n, speedFOC_ESM2.pin_n) annotation (Line(points={{-2,
          16},{-28,16},{-28,-74},{-2,-74}}, color={0,0,255}));
  connect(switch.p, speedFOC_ESM.pin_p) annotation (Line(points={{-22,34},{-14,
          34},{-14,28},{-2,28}}, color={0,0,255}));
  connect(booleanStep.y, switch.control)
    annotation (Line(points={{-5,66},{-32,66},{-32,46}}, color={255,0,255}));
  connect(battery_BMS_fix.n1, speedFOC_ESM2.pin_n) annotation (Line(points={{
          -74.3,-17.4},{-48,-17.4},{-48,-74},{-2,-74}}, color={0,0,255}));
  connect(switch.n, battery_BMS_fix.p1) annotation (Line(points={{-42,34},{-66,
          34},{-66,-9.4},{-74.3,-9.4}}, color={0,0,255}));
  connect(HTS.pin_n, speedFOC_ESM.pin_p) annotation (Line(points={{-55,108},{
          -14,108},{-14,28},{-2,28}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -100},{100,140}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{100,
            140}})),
    experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
end Battery_switch;
