within CHEETA.Aircraft.Electrical.HTS.Examples;
model Loss_of_Load

  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-88,-14})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-98,-50},{-78,-30}})));
  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{-32,12},{-12,32}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-24,-78},{-44,-58}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{22,-78},{2,-58}})));
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
    P=10) annotation (Placement(transformation(extent={{-64,-4},{-48,4}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        Machines.Records.CHEETA_Records.CHEETA_1MW_controller data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Machines.Records.CHEETA_Records.CHEETA_1MW data))
    annotation (Placement(transformation(extent={{4,-16},{24,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-20,-52},{0,-32}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{36,-48},{56,-28}})));
  Modelica.Mechanics.Rotational.Sources.TorqueStep                 load(
    stepTorque=-30000,
    offsetTorque=-500,
    startTime=5)                          annotation (Placement(transformation(extent={{96,-48},
            {76,-28}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{38,-12},
            {50,0}})));
equation
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-88,-24},{-88,-30}}, color={0,0,255}));
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-22,-68},{1,-68}}, color={0,0,127}));
  connect(ground1.p,electricDrive. pin_n)
    annotation (Line(points={{-10,-32},{-10,-12},{4,-12}},   color={0,0,255}));
  connect(constantVoltage.p, hTS_filmboiling3_2.pin_p)
    annotation (Line(points={{-88,-4},{-88,0},{-65,0}}, color={0,0,255}));
  connect(hTS_filmboiling3_2.port_a, prescribedTemperature.port) annotation (
      Line(points={{-55.8,-4},{-56,-4},{-56,-68},{-44,-68}}, color={191,0,0}));
  connect(hTS_filmboiling3_2.pin_n, electricDrive.pin_p)
    annotation (Line(points={{-47,0},{4,0}}, color={0,0,255}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{-11,22},{14,22},{14,6}}, color={0,0,127}));
  connect(multiSensor.flange_a, electricDrive.flange)
    annotation (Line(points={{38,-6},{24,-6}}, color={0,0,0}));
  connect(inertia.flange_a, multiSensor.flange_b) annotation (Line(points={{36,
          -38},{30,-38},{30,-20},{66,-20},{66,-6},{50,-6}}, color={0,0,0}));
  connect(inertia.flange_b, load.flange)
    annotation (Line(points={{56,-38},{76,-38}}, color={0,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,60}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,60}})),
    experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
end Loss_of_Load;
