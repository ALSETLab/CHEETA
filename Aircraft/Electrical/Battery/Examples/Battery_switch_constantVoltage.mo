within CHEETA.Aircraft.Electrical.Battery.Examples;
model Battery_switch_constantVoltage
  "Battery powering the CHEETA powertrain system"
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM(
    useThermalPort=false,
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
               annotation (Placement(transformation(extent={{-4,20},{16,40}})));
  Modelica.Blocks.Sources.Constant const2(k=733.038285)
    annotation (Placement(transformation(extent={{38,46},{24,60}})));
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
               annotation (Placement(transformation(extent={{-4,-26},{16,-6}})));
  Modelica.Blocks.Sources.Constant const1(k=733.038285)
    annotation (Placement(transformation(extent={{38,-4},{24,10}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{10,114},{-10,134}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{62,114},{42,134}})));
  Modelica.Blocks.Sources.Constant const4(k=733.038285)
    annotation (Placement(transformation(extent={{38,-46},{24,-32}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1e-6)
    annotation (Placement(transformation(extent={{32,20},{52,40}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1e-6)
    annotation (Placement(transformation(extent={{32,-26},{52,-6}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{86,20},{66,40}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque2(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{86,-26},{66,-6}})));
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
               annotation (Placement(transformation(extent={{-4,-70},{16,-50}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1e-6)
    annotation (Placement(transformation(extent={{32,-70},{52,-50}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque3(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{86,-70},{66,-50}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-126,68},{-106,88}})));
  Modelica.Blocks.Sources.Constant const6(k=353.15)
    annotation (Placement(transformation(extent={{-158,68},{-138,88}})));
  FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-106,104},{-82,126}})));
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
    annotation (Placement(transformation(extent={{-74,120},{-58,112}})));
  Modelica.Electrical.Analog.Ideal.IdealTwoWaySwitch switch
    annotation (Placement(transformation(extent={{-24,32},{-44,52}})));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=5, startValue=true)
    annotation (Placement(transformation(extent={{14,64},{-6,84}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-70,-14})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-80,-86},{-60,-66}})));
equation
  connect(const2.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{23.3,53},{6,53},{6,42}}, color={0,0,127}));
  connect(const1.y,speedFOC_ESM1. desiredSpeed)
    annotation (Line(points={{23.3,3},{6,3},{6,-4}},      color={0,0,127}));
  connect(speedFOC_ESM1.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{-4,-10},
          {-12,-10},{-12,36},{-4,36}},         color={0,0,255}));
  connect(inertia.flange_a,speedFOC_ESM. flange)
    annotation (Line(points={{32,30},{16,30}},color={0,0,0}));
  connect(inertia1.flange_a,speedFOC_ESM1. flange)
    annotation (Line(points={{32,-16},{16,-16}},
                                              color={0,0,0}));
  connect(constantTorque.flange,inertia. flange_b)
    annotation (Line(points={{66,30},{52,30}}, color={0,0,0}));
  connect(inertia1.flange_b,constantTorque2. flange)
    annotation (Line(points={{52,-16},{66,-16}},
                                               color={0,0,0}));
  connect(inertia2.flange_a,speedFOC_ESM2. flange)
    annotation (Line(points={{32,-60},{16,-60}},color={0,0,0}));
  connect(inertia2.flange_b,constantTorque3. flange)
    annotation (Line(points={{52,-60},{66,-60}}, color={0,0,0}));
  connect(const4.y,speedFOC_ESM2. desiredSpeed)
    annotation (Line(points={{23.3,-39},{6,-39},{6,-48}}, color={0,0,127}));
  connect(speedFOC_ESM2.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{-4,-54},
          {-12,-54},{-12,36},{-4,36}},           color={0,0,255}));
  connect(const6.y,prescribedTemperature4. T)
    annotation (Line(points={{-137,78},{-128,78}},
                                               color={0,0,127}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a,prescribedTemperature4. port)
    annotation (Line(points={{-94,104},{-94,78},{-106,78}},
                           color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1,HTS. pin_p) annotation (Line(
        points={{-82,116.1},{-75,116}},                       color={0,0,255}));
  connect(prescribedTemperature3.port,HTS. port_a) annotation (Line(points={{-10,124},
          {-65.8,124},{-65.8,120}},         color={191,0,0}));
  connect(const3.y, prescribedTemperature3.T)
    annotation (Line(points={{41,124},{12,124}}, color={0,0,127}));
  connect(speedFOC_ESM1.pin_n, speedFOC_ESM2.pin_n) annotation (Line(points={{
          -4,-22},{-30,-22},{-30,-66},{-4,-66}}, color={0,0,255}));
  connect(speedFOC_ESM.pin_n, speedFOC_ESM2.pin_n) annotation (Line(points={{-4,
          24},{-30,24},{-30,-66},{-4,-66}}, color={0,0,255}));
  connect(switch.n1, HTS.pin_n) annotation (Line(points={{-44,46},{-50,46},{-50,
          116},{-57,116}}, color={0,0,255}));
  connect(switch.p, speedFOC_ESM.pin_p) annotation (Line(points={{-24,42},{-16,
          42},{-16,36},{-4,36}}, color={0,0,255}));
  connect(booleanStep.y, switch.control)
    annotation (Line(points={{-7,74},{-34,74},{-34,54}}, color={255,0,255}));
  connect(constantVoltage.n, speedFOC_ESM2.pin_n)
    annotation (Line(points={{-70,-24},{-70,-66},{-4,-66}}, color={0,0,255}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-70,-24},{-70,-66}}, color={0,0,255}));
  connect(switch.n2, constantVoltage.p)
    annotation (Line(points={{-44,42},{-70,42},{-70,-4}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -100},{100,140}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{100,
            140}})),
    experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
end Battery_switch_constantVoltage;
