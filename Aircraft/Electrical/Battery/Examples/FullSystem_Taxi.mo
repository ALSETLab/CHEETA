within CHEETA.Aircraft.Electrical.Battery.Examples;
model FullSystem_Taxi
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
  Battery_noControl battery_noControl1 annotation (Placement(transformation(
        extent={{-8,-7},{8,7}},
        rotation=270,
        origin={142,15})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM4(
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
               annotation (Placement(transformation(extent={{210,4},{230,24}})));
  Modelica.Blocks.Sources.Constant const5(k=0)
    annotation (Placement(transformation(extent={{180,36},{194,50}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{156,-146},{176,-126}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM5(
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
               annotation (Placement(transformation(extent={{210,-36},{230,-16}})));
  Modelica.Blocks.Sources.Constant const6(k=0)
    annotation (Placement(transformation(extent={{180,-14},{194,0}})));
  Modelica.Blocks.Sources.Constant const7(k=0)
    annotation (Placement(transformation(extent={{182,-58},{196,-44}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia4(J=1e-6)
    annotation (Placement(transformation(extent={{246,4},{266,24}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia5(J=1e-6)
    annotation (Placement(transformation(extent={{246,-36},{266,-16}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque4(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{300,4},{280,24}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque5(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{300,-36},{280,-16}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM6(
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
               annotation (Placement(transformation(extent={{210,-80},{230,-60}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia6(J=1e-6)
    annotation (Placement(transformation(extent={{246,-80},{266,-60}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque6(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{300,-80},{280,-60}})));
  Battery_noControl battery_noControl2 annotation (Placement(transformation(
        extent={{-8,-7},{8,7}},
        rotation=270,
        origin={320,15})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM7(
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
               annotation (Placement(transformation(extent={{386,4},{406,24}})));
  Modelica.Blocks.Sources.Constant const8(k=0)
    annotation (Placement(transformation(extent={{356,36},{370,50}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{332,-146},{352,-126}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM8(
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
               annotation (Placement(transformation(extent={{386,-36},{406,-16}})));
  Modelica.Blocks.Sources.Constant const9(k=0)
    annotation (Placement(transformation(extent={{356,-14},{370,0}})));
  Modelica.Blocks.Sources.Constant const10(k=0)
    annotation (Placement(transformation(extent={{358,-58},{372,-44}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia7(J=1e-6)
    annotation (Placement(transformation(extent={{422,4},{442,24}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia8(J=1e-6)
    annotation (Placement(transformation(extent={{422,-36},{442,-16}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque7(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{476,4},{456,24}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque8(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{476,-36},{456,-16}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM9(
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
               annotation (Placement(transformation(extent={{386,-80},{406,-60}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia9(J=1e-6)
    annotation (Placement(transformation(extent={{422,-80},{442,-60}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque9(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{476,-80},{456,-60}})));
  Modelica.Blocks.Sources.Constant const11(k=0)
    annotation (Placement(transformation(extent={{358,-100},{372,-86}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM10(
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
               annotation (Placement(transformation(extent={{386,-122},{406,
            -102}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia10(J=1e-6)
    annotation (Placement(transformation(extent={{424,-122},{444,-102}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque10(
      tau_constant=-20)
    annotation (Placement(transformation(extent={{478,-122},{458,-102}})));
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
    annotation (Line(points={{-46,16},{-14,16},{-14,16},{24,16}},
                                                  color={0,0,255}));
  connect(battery_noControl.n1, ground1.p) annotation (Line(points={{-46,6},{
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
  connect(const5.y, speedFOC_ESM4.desiredSpeed)
    annotation (Line(points={{194.7,43},{220,43},{220,26}}, color={0,0,127}));
  connect(speedFOC_ESM4.pin_n, ground2.p)
    annotation (Line(points={{210,8},{166,8},{166,-126}}, color={0,0,255}));
  connect(const6.y,speedFOC_ESM5. desiredSpeed)
    annotation (Line(points={{194.7,-7},{220,-7},{220,-14}},
                                                          color={0,0,127}));
  connect(speedFOC_ESM5.pin_n, ground2.p) annotation (Line(points={{210,-32},{
          166,-32},{166,-126}}, color={0,0,255}));
  connect(speedFOC_ESM5.pin_p, speedFOC_ESM4.pin_p) annotation (Line(points={{
          210,-20},{202,-20},{202,20},{210,20}}, color={0,0,255}));
  connect(inertia4.flange_a, speedFOC_ESM4.flange)
    annotation (Line(points={{246,14},{230,14}}, color={0,0,0}));
  connect(inertia5.flange_a,speedFOC_ESM5. flange)
    annotation (Line(points={{246,-26},{230,-26}},
                                              color={0,0,0}));
  connect(constantTorque4.flange, inertia4.flange_b)
    annotation (Line(points={{280,14},{266,14}}, color={0,0,0}));
  connect(inertia5.flange_b,constantTorque5. flange)
    annotation (Line(points={{266,-26},{280,-26}},
                                               color={0,0,0}));
  connect(inertia6.flange_a,speedFOC_ESM6. flange)
    annotation (Line(points={{246,-70},{230,-70}},
                                                color={0,0,0}));
  connect(inertia6.flange_b,constantTorque6. flange)
    annotation (Line(points={{266,-70},{280,-70}},
                                                 color={0,0,0}));
  connect(const7.y,speedFOC_ESM6. desiredSpeed)
    annotation (Line(points={{196.7,-51},{220,-51},{220,-58}},
                                                          color={0,0,127}));
  connect(speedFOC_ESM6.pin_p, speedFOC_ESM4.pin_p) annotation (Line(points={{
          210,-64},{202,-64},{202,20},{210,20}}, color={0,0,255}));
  connect(speedFOC_ESM6.pin_n, ground2.p) annotation (Line(points={{210,-76},{
          166,-76},{166,-126}}, color={0,0,255}));
  connect(battery_noControl1.p1, speedFOC_ESM4.pin_p)
    annotation (Line(points={{152,20},{178,20},{178,20},{210,20}},
                                                   color={0,0,255}));
  connect(battery_noControl1.n1, ground2.p) annotation (Line(points={{152,10},{
          166,10},{166,-126}},  color={0,0,255}));
  connect(const8.y, speedFOC_ESM7.desiredSpeed)
    annotation (Line(points={{370.7,43},{396,43},{396,26}}, color={0,0,127}));
  connect(speedFOC_ESM7.pin_n, ground3.p)
    annotation (Line(points={{386,8},{342,8},{342,-126}}, color={0,0,255}));
  connect(const9.y,speedFOC_ESM8. desiredSpeed)
    annotation (Line(points={{370.7,-7},{396,-7},{396,-14}},
                                                          color={0,0,127}));
  connect(speedFOC_ESM8.pin_n, ground3.p) annotation (Line(points={{386,-32},{
          342,-32},{342,-126}}, color={0,0,255}));
  connect(speedFOC_ESM8.pin_p, speedFOC_ESM7.pin_p) annotation (Line(points={{
          386,-20},{378,-20},{378,20},{386,20}}, color={0,0,255}));
  connect(inertia7.flange_a, speedFOC_ESM7.flange)
    annotation (Line(points={{422,14},{406,14}}, color={0,0,0}));
  connect(inertia8.flange_a,speedFOC_ESM8. flange)
    annotation (Line(points={{422,-26},{406,-26}},
                                              color={0,0,0}));
  connect(constantTorque7.flange, inertia7.flange_b)
    annotation (Line(points={{456,14},{442,14}}, color={0,0,0}));
  connect(inertia8.flange_b,constantTorque8. flange)
    annotation (Line(points={{442,-26},{456,-26}},
                                               color={0,0,0}));
  connect(inertia9.flange_a,speedFOC_ESM9. flange)
    annotation (Line(points={{422,-70},{406,-70}},
                                                color={0,0,0}));
  connect(inertia9.flange_b,constantTorque9. flange)
    annotation (Line(points={{442,-70},{456,-70}},
                                                 color={0,0,0}));
  connect(const10.y, speedFOC_ESM9.desiredSpeed) annotation (Line(points={{
          372.7,-51},{396,-51},{396,-58}}, color={0,0,127}));
  connect(speedFOC_ESM9.pin_p, speedFOC_ESM7.pin_p) annotation (Line(points={{
          386,-64},{378,-64},{378,20},{386,20}}, color={0,0,255}));
  connect(speedFOC_ESM9.pin_n, ground3.p) annotation (Line(points={{386,-76},{
          342,-76},{342,-126}}, color={0,0,255}));
  connect(battery_noControl2.p1, speedFOC_ESM7.pin_p)
    annotation (Line(points={{330,20},{356,20},{356,20},{386,20}},
                                                   color={0,0,255}));
  connect(battery_noControl2.n1, ground3.p) annotation (Line(points={{330,10},{
          342,10},{342,-126}},  color={0,0,255}));
  connect(inertia10.flange_a, speedFOC_ESM10.flange)
    annotation (Line(points={{424,-112},{406,-112}}, color={0,0,0}));
  connect(inertia10.flange_b, constantTorque10.flange)
    annotation (Line(points={{444,-112},{458,-112}}, color={0,0,0}));
  connect(const11.y, speedFOC_ESM10.desiredSpeed) annotation (Line(points={{
          372.7,-93},{396,-93},{396,-100}}, color={0,0,127}));
  connect(speedFOC_ESM10.pin_p, speedFOC_ESM7.pin_p) annotation (Line(points={{
          386,-106},{378,-106},{378,20},{386,20}}, color={0,0,255}));
  connect(speedFOC_ESM10.pin_n, ground3.p) annotation (Line(points={{386,-118},
          {342,-118},{342,-126}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
            -160},{500,60}})),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-160},{500,60}}),
        graphics={
        Rectangle(extent={{-76,58},{128,-154}}, lineColor={28,108,200}),
        Rectangle(extent={{130,58},{308,-154}}, lineColor={28,108,200}),
        Rectangle(extent={{310,58},{484,-154}}, lineColor={28,108,200}),
        Text(
          extent={{44,-138},{108,-150}},
          textColor={28,108,200},
          textString="Left wing"),
        Text(
          extent={{230,-138},{294,-150}},
          textColor={28,108,200},
          textString="Body"),
        Text(
          extent={{414,-136},{478,-148}},
          textColor={28,108,200},
          textString="Right wing")}),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end FullSystem_Taxi;
