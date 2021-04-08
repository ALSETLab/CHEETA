within CHEETA.Aircraft.Electrical.Battery.Examples;
model Wing_Taxi
  import Battery;
  Battery_noControl battery_noControl annotation (Placement(transformation(
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
  Modelica.Blocks.Sources.Constant const2(k=0)
    annotation (Placement(transformation(extent={{-6,32},{8,46}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-30,-150},{-10,-130}})));
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
  Modelica.Blocks.Sources.Constant const1(k=0)
    annotation (Placement(transformation(extent={{-6,-18},{8,-4}})));
  Modelica.Blocks.Sources.Constant const4(k=0)
    annotation (Placement(transformation(extent={{-4,-62},{10,-48}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1e-6)
    annotation (Placement(transformation(extent={{60,0},{80,20}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1e-6)
    annotation (Placement(transformation(extent={{60,-40},{80,-20}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{114,0},{94,20}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque2(
      tau_constant=-20)
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
      tau_constant=-20)
    annotation (Placement(transformation(extent={{114,-84},{94,-64}})));
  Modelica.Blocks.Sources.Constant const3(k=0)
    annotation (Placement(transformation(extent={{-4,-104},{10,-90}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM3(
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
               annotation (Placement(transformation(extent={{24,-126},{44,-106}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia3(J=1e-6)
    annotation (Placement(transformation(extent={{62,-126},{82,-106}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque1(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{116,-126},{96,-106}})));
equation
  connect(const2.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{8.7,39},{34,39},{34,22}},color={0,0,127}));
  connect(speedFOC_ESM.pin_n, ground1.p)
    annotation (Line(points={{24,4},{-20,4},{-20,-130}}, color={0,0,255}));
  connect(const1.y,speedFOC_ESM1. desiredSpeed)
    annotation (Line(points={{8.7,-11},{34,-11},{34,-18}},color={0,0,127}));
  connect(speedFOC_ESM1.pin_n, ground1.p)
    annotation (Line(points={{24,-36},{-20,-36},{-20,-130}}, color={0,0,255}));
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
  connect(const4.y,speedFOC_ESM2. desiredSpeed)
    annotation (Line(points={{10.7,-55},{34,-55},{34,-62}},
                                                          color={0,0,127}));
  connect(speedFOC_ESM2.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{24,-68},
          {16,-68},{16,16},{24,16}},             color={0,0,255}));
  connect(speedFOC_ESM2.pin_n, ground1.p)
    annotation (Line(points={{24,-80},{-20,-80},{-20,-130}}, color={0,0,255}));
  connect(battery_noControl.p1, speedFOC_ESM.pin_p)
    annotation (Line(points={{-46.4,16},{24,16}}, color={0,0,255}));
  connect(battery_noControl.n1, ground1.p) annotation (Line(points={{-46.4,6},{
          -32,6},{-32,4},{-20,4},{-20,-130}}, color={0,0,255}));
  connect(inertia3.flange_a,speedFOC_ESM3. flange)
    annotation (Line(points={{62,-116},{44,-116}},
                                                color={0,0,0}));
  connect(inertia3.flange_b,constantTorque1. flange)
    annotation (Line(points={{82,-116},{96,-116}},
                                                 color={0,0,0}));
  connect(const3.y,speedFOC_ESM3. desiredSpeed)
    annotation (Line(points={{10.7,-97},{34,-97},{34,-104}},
                                                          color={0,0,127}));
  connect(speedFOC_ESM3.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{24,-110},
          {16,-110},{16,16},{24,16}},            color={0,0,255}));
  connect(speedFOC_ESM3.pin_n, ground1.p) annotation (Line(points={{24,-122},{
          -20,-122},{-20,-130}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
            -160},{120,60}})),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-160},{120,60}})),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end Wing_Taxi;