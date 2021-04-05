within CHEETA.Examples.CHEETAElectricalSystem;
model System "CHEETA Electrical System"
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
    annotation (Placement(transformation(extent={{-46,84},{-32,98}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-70,-96},{-50,-76}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,50})));
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
    b(displayUnit="mm"))
    annotation (Placement(transformation(extent={{-60,78},{-44,70}})));
  Modelica.Blocks.Sources.Constant const4(k=733.038285)
    annotation (Placement(transformation(extent={{-46,-8},{-32,6}})));
equation
  connect(const2.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{-31.3,91},{-6,91},{-6,76}},
                                                       color={0,0,127}));
  connect(speedFOC_ESM.pin_n, ground.p)
    annotation (Line(points={{-16,58},{-60,58},{-60,-76}}, color={0,0,255}));
  connect(constantVoltage.n, ground.p) annotation (Line(points={{-80,40},{-80,
          -72},{-60,-72},{-60,-76}}, color={0,0,255}));
  connect(const1.y, speedFOC_ESM1.desiredSpeed)
    annotation (Line(points={{-31.3,43},{-6,43},{-6,36}}, color={0,0,127}));
  connect(speedFOC_ESM1.pin_n, ground.p)
    annotation (Line(points={{-16,18},{-60,18},{-60,-76}}, color={0,0,255}));
  connect(speedFOC_ESM1.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{
          -16,30},{-24,30},{-24,70},{-16,70}}, color={0,0,255}));
  connect(HTS.pin_p, constantVoltage.p)
    annotation (Line(points={{-61,74},{-80,74},{-80,60}}, color={0,0,255}));
  connect(HTS.pin_n, speedFOC_ESM.pin_p) annotation (Line(points={{-43,74},{-26,
          74},{-26,70},{-16,70}}, color={0,0,255}));
  connect(prescribedTemperature3.port, HTS.port_a) annotation (Line(points={{-28,114},
          {-52,114},{-52,80},{-51.8,80},{-51.8,78}},
                                                 color={191,0,0}));
  connect(const3.y, prescribedTemperature3.T)
    annotation (Line(points={{29,114},{-6,114}},
                                               color={0,0,127}));
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
end System;
