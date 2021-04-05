within CHEETA.Examples.CHEETAElectricalSystem;
model Body_System "CHEETA Electrical System for one wing"
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{-16,54},{4,74}})));
  Modelica.Blocks.Sources.Constant const2(k=733.038285)
    annotation (Placement(transformation(extent={{-46,86},{-32,100}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-70,-96},{-50,-76}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM1(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{-16,14},{4,34}})));
  Modelica.Blocks.Sources.Constant const1(k=733.038285)
    annotation (Placement(transformation(extent={{-46,36},{-32,50}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{-8,104},{-28,124}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{50,104},{30,124}})));
  Modelica.Blocks.Sources.Constant const4(k=733.038285)
    annotation (Placement(transformation(extent={{-46,-8},{-32,6}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1e-6)
    annotation (Placement(transformation(extent={{20,54},{40,74}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1e-6)
    annotation (Placement(transformation(extent={{20,14},{40,34}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{74,54},{54,74}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque2(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{74,14},{54,34}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM2(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{-16,-30},{4,-10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1e-6)
    annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque3(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{74,-30},{54,-10}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-140,24},{-120,44}})));
  Modelica.Blocks.Sources.Constant const6(k=353.15)
    annotation (Placement(transformation(extent={{-174,24},{-154,44}})));
  Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-102,72},{-88,86}})));
  Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage_Hydrogen
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
    annotation (Placement(transformation(extent={{-74,84},{-58,76}})));
equation
  connect(const2.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{-31.3,93},{-6,93},{-6,76}},
                                                       color={0,0,127}));
  connect(speedFOC_ESM.pin_n, ground.p)
    annotation (Line(points={{-16,58},{-60,58},{-60,-76}}, color={0,0,255}));
  connect(const1.y, speedFOC_ESM1.desiredSpeed)
    annotation (Line(points={{-31.3,43},{-6,43},{-6,36}}, color={0,0,127}));
  connect(speedFOC_ESM1.pin_n, ground.p)
    annotation (Line(points={{-16,18},{-60,18},{-60,-76}}, color={0,0,255}));
  connect(speedFOC_ESM1.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{
          -16,30},{-24,30},{-24,70},{-16,70}}, color={0,0,255}));
  connect(const3.y, prescribedTemperature3.T)
    annotation (Line(points={{29,114},{-6,114}},
                                               color={0,0,127}));
  connect(inertia.flange_a, speedFOC_ESM.flange)
    annotation (Line(points={{20,64},{4,64}}, color={0,0,0}));
  connect(inertia1.flange_a, speedFOC_ESM1.flange)
    annotation (Line(points={{20,24},{4,24}}, color={0,0,0}));
  connect(constantTorque.flange, inertia.flange_b)
    annotation (Line(points={{54,64},{40,64}}, color={0,0,0}));
  connect(inertia1.flange_b, constantTorque2.flange)
    annotation (Line(points={{40,24},{54,24}}, color={0,0,0}));
  connect(inertia2.flange_a, speedFOC_ESM2.flange)
    annotation (Line(points={{20,-20},{4,-20}}, color={0,0,0}));
  connect(inertia2.flange_b, constantTorque3.flange)
    annotation (Line(points={{40,-20},{54,-20}}, color={0,0,0}));
  connect(const4.y, speedFOC_ESM2.desiredSpeed)
    annotation (Line(points={{-31.3,-1},{-6,-1},{-6,-8}}, color={0,0,127}));
  connect(speedFOC_ESM2.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{
          -16,-14},{-24,-14},{-24,70},{-16,70}}, color={0,0,255}));
  connect(speedFOC_ESM2.pin_n, ground.p)
    annotation (Line(points={{-16,-26},{-60,-26},{-60,-76}}, color={0,0,255}));
  connect(const6.y,prescribedTemperature4. T)
    annotation (Line(points={{-153,34},{-142,34}},
                                               color={0,0,127}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a,prescribedTemperature4. port)
    annotation (Line(points={{-95,72},{-94,72},{-94,34},{-120,34}},
                           color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1, HTS.pin_p) annotation (Line(
        points={{-88,79.7},{-81.5,79.7},{-81.5,80},{-75,80}}, color={0,0,255}));
  connect(HTS.pin_n, speedFOC_ESM.pin_p) annotation (Line(points={{-57,80},{-26,
          79.7},{-26,70},{-16,70}}, color={0,0,255}));
  connect(prescribedTemperature3.port, HTS.port_a) annotation (Line(points={{
          -28,114},{-65.8,114},{-65.8,84}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            140}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,140}})),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end Body_System;
