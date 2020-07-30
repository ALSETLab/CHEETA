within CHEETA.Aircraft.Electrical.HTS.Examples;
model Fixed_Rotor_5MW

  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-84,-14})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-94,-50},{-74,-30}})));
  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{-28,12},{-8,32}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-26,-38},{-46,-18}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{20,-38},{0,-18}})));
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
    annotation (Placement(transformation(extent={{62,-16},{82,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{42,-38},{62,-18}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{94,-48},{114,-28}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{96,-12},
            {108,0}})));
  Modelica.Mechanics.Rotational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{114,-48},{134,-28}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive1(
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
    annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1)
    annotation (Placement(transformation(extent={{96,-122},{116,-102}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor1
                                                                annotation (Placement(transformation(extent={{98,-86},
            {110,-74}})));
  Modelica.Mechanics.Rotational.Components.Fixed fixed1
    annotation (Placement(transformation(extent={{116,-122},{136,-102}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive2(
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
    annotation (Placement(transformation(extent={{62,-162},{82,-142}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1)
    annotation (Placement(transformation(extent={{94,-194},{114,-174}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor2
                                                                annotation (Placement(transformation(extent={{96,-158},
            {108,-146}})));
  Modelica.Mechanics.Rotational.Components.Fixed fixed2
    annotation (Placement(transformation(extent={{114,-194},{134,-174}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive3(
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
    annotation (Placement(transformation(extent={{60,-238},{80,-218}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia3(J=1)
    annotation (Placement(transformation(extent={{92,-270},{112,-250}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor3
                                                                annotation (Placement(transformation(extent={{94,-234},
            {106,-222}})));
  Modelica.Mechanics.Rotational.Components.Fixed fixed3
    annotation (Placement(transformation(extent={{112,-270},{132,-250}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive4(
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
    annotation (Placement(transformation(extent={{56,-302},{76,-282}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia4(J=1)
    annotation (Placement(transformation(extent={{88,-334},{108,-314}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor4
                                                                annotation (Placement(transformation(extent={{90,-298},
            {102,-286}})));
  Modelica.Mechanics.Rotational.Components.Fixed fixed4
    annotation (Placement(transformation(extent={{120,-330},{140,-310}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{44,-106},{64,-86}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{44,-186},{64,-166}})));
  Modelica.Electrical.Analog.Basic.Ground ground4
    annotation (Placement(transformation(extent={{38,-254},{58,-234}})));
  Modelica.Electrical.Analog.Basic.Ground ground5
    annotation (Placement(transformation(extent={{40,-326},{60,-306}})));
  Modelica.Blocks.Sources.Constant
                               tauRef1(k=733.038285)
               annotation (Placement(transformation(extent={{40,-62},{60,-42}})));
  Modelica.Blocks.Sources.Constant
                               tauRef2(k=733.038285)
               annotation (Placement(transformation(extent={{38,-134},{58,-114}})));
  Modelica.Blocks.Sources.Constant
                               tauRef3(k=733.038285)
               annotation (Placement(transformation(extent={{32,-214},{52,-194}})));
  Modelica.Blocks.Sources.Constant
                               tauRef4(k=733.038285)
               annotation (Placement(transformation(extent={{34,-278},{54,-258}})));
equation
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-24,-28},{-1,-28}},color={0,0,127}));
  connect(ground1.p,electricDrive. pin_n)
    annotation (Line(points={{52,-18},{52,-12},{62,-12}},    color={0,0,255}));
  connect(constantVoltage.p, hTS_filmboiling3_2.pin_p)
    annotation (Line(points={{-84,-4},{-84,0},{-61,0}}, color={0,0,255}));
  connect(hTS_filmboiling3_2.port_a, prescribedTemperature.port) annotation (
      Line(points={{-51.8,-4},{-52,-4},{-52,-28},{-46,-28}}, color={191,0,0}));
  connect(hTS_filmboiling3_2.pin_n, electricDrive.pin_p)
    annotation (Line(points={{-43,0},{62,0}}, color={0,0,255}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{-7,22},{72,22},{72,6}}, color={0,0,127}));
  connect(multiSensor.flange_a, electricDrive.flange)
    annotation (Line(points={{96,-6},{82,-6}}, color={0,0,0}));
  connect(inertia.flange_a, multiSensor.flange_b) annotation (Line(points={{94,
          -38},{88,-38},{88,-20},{124,-20},{124,-6},{108,-6}}, color={0,0,0}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-84,-24},{-84,-30}}, color={0,0,255}));
  connect(inertia.flange_b, fixed.flange)
    annotation (Line(points={{114,-38},{124,-38}}, color={0,0,0}));
  connect(inertia1.flange_a, multiSensor1.flange_b) annotation (Line(points={{
          96,-112},{90,-112},{90,-94},{126,-94},{126,-80},{110,-80}}, color={0,
          0,0}));
  connect(inertia1.flange_b, fixed1.flange)
    annotation (Line(points={{116,-112},{126,-112}}, color={0,0,0}));
  connect(inertia2.flange_a, multiSensor2.flange_b) annotation (Line(points={{
          94,-184},{88,-184},{88,-166},{124,-166},{124,-152},{108,-152}}, color=
         {0,0,0}));
  connect(inertia2.flange_b, fixed2.flange)
    annotation (Line(points={{114,-184},{124,-184}}, color={0,0,0}));
  connect(inertia3.flange_a, multiSensor3.flange_b) annotation (Line(points={{
          92,-260},{86,-260},{86,-242},{122,-242},{122,-228},{106,-228}}, color=
         {0,0,0}));
  connect(inertia3.flange_b, fixed3.flange)
    annotation (Line(points={{112,-260},{122,-260}}, color={0,0,0}));
  connect(inertia4.flange_a, multiSensor4.flange_b) annotation (Line(points={{
          88,-324},{82,-324},{82,-306},{118,-306},{118,-292},{102,-292}}, color=
         {0,0,0}));
  connect(inertia4.flange_b, fixed4.flange) annotation (Line(points={{108,-324},
          {108,-320},{130,-320}}, color={0,0,0}));
  connect(multiSensor1.flange_a, electricDrive1.flange)
    annotation (Line(points={{98,-80},{84,-80}}, color={0,0,0}));
  connect(multiSensor2.flange_a, electricDrive2.flange)
    annotation (Line(points={{96,-152},{82,-152},{82,-152}}, color={0,0,0}));
  connect(multiSensor3.flange_a, electricDrive3.flange)
    annotation (Line(points={{94,-228},{80,-228}}, color={0,0,0}));
  connect(multiSensor4.flange_a, electricDrive4.flange) annotation (Line(points=
         {{90,-292},{84,-292},{84,-292},{76,-292}}, color={0,0,0}));
  connect(electricDrive1.pin_n, ground2.p)
    annotation (Line(points={{64,-86},{54,-86}}, color={0,0,255}));
  connect(electricDrive2.pin_n, ground3.p)
    annotation (Line(points={{62,-158},{54,-158},{54,-166}}, color={0,0,255}));
  connect(electricDrive3.pin_n, ground4.p)
    annotation (Line(points={{60,-234},{48,-234}}, color={0,0,255}));
  connect(electricDrive4.pin_n, ground5.p) annotation (Line(points={{56,-298},{
          54,-298},{54,-306},{50,-306}}, color={0,0,255}));
  connect(electricDrive1.pin_p, electricDrive.pin_p) annotation (Line(points={{
          64,-74},{26,-74},{26,0},{62,0}}, color={0,0,255}));
  connect(electricDrive2.pin_p, electricDrive.pin_p) annotation (Line(points={{
          62,-146},{26,-146},{26,0},{62,0}}, color={0,0,255}));
  connect(electricDrive3.pin_p, electricDrive.pin_p) annotation (Line(points={{
          60,-222},{26,-222},{26,0},{62,0}}, color={0,0,255}));
  connect(electricDrive4.pin_p, electricDrive.pin_p) annotation (Line(points={{
          56,-286},{26,-286},{26,0},{62,0}}, color={0,0,255}));
  connect(tauRef1.y, electricDrive1.desiredSpeed)
    annotation (Line(points={{61,-52},{74,-52},{74,-68}}, color={0,0,127}));
  connect(tauRef2.y, electricDrive2.desiredSpeed)
    annotation (Line(points={{59,-124},{72,-124},{72,-140}}, color={0,0,127}));
  connect(tauRef3.y, electricDrive3.desiredSpeed)
    annotation (Line(points={{53,-204},{70,-204},{70,-216}}, color={0,0,127}));
  connect(tauRef4.y, electricDrive4.desiredSpeed)
    annotation (Line(points={{55,-268},{66,-268},{66,-280}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-100,-340},{160,40}})), Icon(
        coordinateSystem(extent={{-100,-340},{160,40}})));
end Fixed_Rotor_5MW;
