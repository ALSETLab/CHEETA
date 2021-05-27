within CHEETA.Aircraft.Electrical.Battery.Examples;
model Body_Taxi
  import Battery;
  Battery_BMS_fix   battery_BMS_fix   annotation (Placement(transformation(
        extent={{-8,-7},{8,7}},
        rotation=270,
        origin={-56,11})));
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
               annotation (Placement(transformation(extent={{24,0},{44,20}})));
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
               annotation (Placement(transformation(extent={{24,-40},{44,-20}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1e-6)
    annotation (Placement(transformation(extent={{60,0},{80,20}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1e-6)
    annotation (Placement(transformation(extent={{60,-40},{80,-20}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=-300)
    annotation (Placement(transformation(extent={{114,0},{94,20}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque2(
      tau_constant=-300)
    annotation (Placement(transformation(extent={{114,-40},{94,-20}})));
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
               annotation (Placement(transformation(extent={{24,-84},{44,-64}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1e-6)
    annotation (Placement(transformation(extent={{60,-84},{80,-64}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque3(
      tau_constant=-300)
    annotation (Placement(transformation(extent={{114,-84},{94,-64}})));
  Modelica.Blocks.Sources.Ramp     ramp(height=20, duration=10)
    annotation (Placement(transformation(extent={{-2,24},{12,38}})));
  Modelica.Blocks.Sources.Ramp     ramp1(height=20, duration=10)
    annotation (Placement(transformation(extent={{-4,-20},{10,-6}})));
  Modelica.Blocks.Sources.Ramp     ramp2(height=20, duration=10)
    annotation (Placement(transformation(extent={{-6,-64},{8,-50}})));
equation
  connect(speedFOC_ESM1.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{24,-24},
          {16,-24},{16,16},{24,16}},           color={0,0,255}));
  connect(inertia.flange_a,speedFOC_ESM. flange)
    annotation (Line(points={{60,10},{44,10}},color={0,0,0}));
  connect(inertia1.flange_a,speedFOC_ESM1. flange)
    annotation (Line(points={{60,-30},{44,-30}},
                                              color={0,0,0}));
  connect(constantTorque.flange,inertia. flange_b)
    annotation (Line(points={{94,10},{80,10}}, color={0,0,0}));
  connect(inertia1.flange_b,constantTorque2. flange)
    annotation (Line(points={{80,-30},{94,-30}},
                                               color={0,0,0}));
  connect(inertia2.flange_a,speedFOC_ESM2. flange)
    annotation (Line(points={{60,-74},{44,-74}},color={0,0,0}));
  connect(inertia2.flange_b,constantTorque3. flange)
    annotation (Line(points={{80,-74},{94,-74}}, color={0,0,0}));
  connect(speedFOC_ESM2.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{24,-68},
          {16,-68},{16,16},{24,16}},             color={0,0,255}));
  connect(battery_BMS_fix.p1, speedFOC_ESM.pin_p) annotation (Line(points={{
          -48.3,16.6},{-12,16.6},{-12,16},{24,16}}, color={0,0,255}));
  connect(speedFOC_ESM2.pin_n, battery_BMS_fix.n1) annotation (Line(points={{24,
          -80},{-42,-80},{-42,8.6},{-48.3,8.6}}, color={0,0,255}));
  connect(speedFOC_ESM.pin_n, battery_BMS_fix.n1) annotation (Line(points={{24,
          4},{-42,4},{-42,8.6},{-48.3,8.6}}, color={0,0,255}));
  connect(speedFOC_ESM1.pin_n, battery_BMS_fix.n1) annotation (Line(points={{24,
          -36},{-42,-36},{-42,8.6},{-48.3,8.6}}, color={0,0,255}));
  connect(ramp.y, speedFOC_ESM.desiredSpeed)
    annotation (Line(points={{12.7,31},{34,31},{34,22}}, color={0,0,127}));
  connect(ramp1.y, speedFOC_ESM1.desiredSpeed)
    annotation (Line(points={{10.7,-13},{34,-13},{34,-18}}, color={0,0,127}));
  connect(ramp2.y, speedFOC_ESM2.desiredSpeed)
    annotation (Line(points={{8.7,-57},{34,-57},{34,-62}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
            -160},{120,60}})),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-160},{120,60}})),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end Body_Taxi;
