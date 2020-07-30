within CHEETA.Aircraft.Electrical.HTS.Examples;
model Inverter_Fault

  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-84,-14})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-94,-50},{-74,-30}})));
  Modelica.Blocks.Sources.Constant
                               tauRef(k=1000)
               annotation (Placement(transformation(extent={{-88,20},{-68,40}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-18,-90},{-38,-70}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{16,-90},{-4,-70}})));
  LiquidCooled.HTS_filmboiling_Voltage2 hTS_filmboiling3_2(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=10000,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0,
    a=0.1,
    b=0.5,
    P=10) annotation (Placement(transformation(extent={{-60,-4},{-44,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-44,-60},{-24,-40}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.LossyLinearized
                                                         inverter(
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Losses.Records.Data.LossyLinearized.IGBT_ModuleInfineon_650V_550A
      data,
    fs=20e3)
    annotation (Placement(transformation(extent={{-18,-16},{2,4}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
                                                             modulationMethod
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
              ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Speed
                                                                          controller(data(
      kp_FW=1,
      Ti_FW=4e-3,
      redeclare
        ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Records.Data.Linear.Industrial_550W
        machineData,
      i_s_max=1.1))
    annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
  ElectrifiedPowertrains.ElectricDrives.Common.Blocks.EnergyAnalyser driveEfficiencyComputation(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{20,-52},{40,-32}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.ElectricQuantities machineVariables(
      terminalConnection=machine.data.terminalConnection)
    annotation (Placement(transformation(extent={{46,-52},{66,-32}})));
              ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Linear
                                                                               machine(
      useThermalPort=false, redeclare
      PowerElectronics.Examples.Records.VSI_20kW data)
    annotation (Placement(transformation(extent={{34,-16},{54,4}})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
    TorqueDirection=false,
    tau_nominal=-500,
    w_nominal(displayUnit="rpm") = 78.539816339745)
                                          annotation (Placement(transformation(extent={{164,-16},
            {144,4}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.01)
    annotation (Placement(transformation(extent={{104,-16},{124,4}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{68,-12},
            {80,0}})));
  Modelica.Electrical.MultiPhase.Basic.Star star annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-6,-34})));
  Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R={0.1,0.1,0.1})
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={16,-22})));
equation
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-16,-80},{-5,-80}},color={0,0,127}));
  connect(constantVoltage.p, hTS_filmboiling3_2.pin_p)
    annotation (Line(points={{-84,-4},{-84,0},{-61,0}}, color={0,0,255}));
  connect(hTS_filmboiling3_2.port_a, prescribedTemperature.port) annotation (
      Line(points={{-51.8,-4},{-52,-4},{-52,-80},{-38,-80}}, color={191,0,0}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-84,-24},{-84,-30}}, color={0,0,255}));
  connect(controller.actuatingVoltages, modulationMethod.phaseVoltages)
    annotation (Line(points={{-31,30},{-22,30}}, color={0,0,127}));
  connect(modulationMethod.normalizedPhaseVoltages, inverter.normalizedPhaseVoltages)
    annotation (Line(points={{1,30},{6,30},{6,8},{-36,8},{-36,-6},{-20,-6}},
        color={0,0,127}));
  connect(hTS_filmboiling3_2.pin_n, inverter.pin_p)
    annotation (Line(points={{-43,0},{-18,0}}, color={0,0,255}));
  connect(inverter.pin_n, ground1.p)
    annotation (Line(points={{-18,-12},{-34,-12},{-34,-40}}, color={0,0,255}));
  connect(machine.plug_p, inverter.plug)
    annotation (Line(points={{34,-6},{2,-6}}, color={0,0,255}));
  connect(controller.electricDriveBus, modulationMethod.electricDriveBus)
    annotation (Line(
      points={{-42,20},{-42,14},{-10,14},{-10,20}},
      color={0,86,166},
      thickness=0.5));
  connect(inverter.electricDriveBus, machine.electricDriveBus) annotation (Line(
      points={{-8,-16},{-8,-20},{44,-20},{44,-16}},
      color={0,86,166},
      thickness=0.5));
  connect(modulationMethod.electricDriveBus, machine.electricDriveBus)
    annotation (Line(
      points={{-10,20},{-10,14},{12,14},{12,-20},{44,-20},{44,-16}},
      color={0,86,166},
      thickness=0.5));
  connect(driveEfficiencyComputation.electricDriveBus, machine.electricDriveBus)
    annotation (Line(
      points={{30,-52},{30,-60},{12,-60},{12,-20},{44,-20},{44,-16}},
      color={0,86,166},
      thickness=0.5));
  connect(machineVariables.electricDriveBus, machine.electricDriveBus)
    annotation (Line(
      points={{56,-52},{56,-60},{12,-60},{12,-20},{44,-20},{44,-16}},
      color={0,86,166},
      thickness=0.5));
  connect(inertia.flange_b,load. flange)
    annotation (Line(
      points={{124,-6},{144,-6}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(inertia.flange_a,multiSensor. flange_b) annotation (Line(points={{104,-6},
          {80,-6}},                                                                                  color={0,0,0}));
  connect(machine.flange,multiSensor. flange_a) annotation (Line(points={{54,-6},
          {68,-6}},                                                                             color={0,0,0}));
  connect(star.pin_n, ground1.p)
    annotation (Line(points={{-16,-34},{-34,-34},{-34,-40}}, color={0,0,255}));
  connect(resistor.plug_p, inverter.plug)
    annotation (Line(points={{16,-12},{16,-6},{2,-6}}, color={0,0,255}));
  connect(resistor.plug_n, star.plug_p)
    annotation (Line(points={{16,-32},{16,-34},{4,-34}}, color={0,0,255}));
  connect(controller.desiredSpeed, tauRef.y)
    annotation (Line(points={{-54,30},{-67,30}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,60}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,60}})));
end Inverter_Fault;
