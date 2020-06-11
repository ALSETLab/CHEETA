within CHEETA.Architectures;
model Battery_FuelCell_CommonBus_noDCDCconverter
  "ii. Centralized architecture in i., but without DC/DC converter"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{28,30},{48,50}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{140,4},
            {152,16}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{186,44},
            {198,56}})));
  Aircraft.Mechanical.Loads.Fan fan
    annotation (Placement(transformation(extent={{186,6},{194,14}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_1MW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{80,0},{100,20}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor1
                                                                annotation (Placement(transformation(extent={{154,-60},
            {166,-48}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive1(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{96,-64},{116,
            -44}})));
  Aircraft.Mechanical.Loads.Fan fan1
    annotation (Placement(transformation(extent={{182,-58},{190,-50}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=tauRef.y)
    annotation (Placement(transformation(extent={{76,-40},{96,-20}})));
  Aircraft.Electrical.BusExt busExt(nn=8)
    annotation (Placement(transformation(extent={{24,0},{20,-152}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor2
                                                                annotation (Placement(transformation(extent={{152,
            -116},{164,-104}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive2(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{94,-120},{114,
            -100}})));
  Aircraft.Mechanical.Loads.Fan fan2
    annotation (Placement(transformation(extent={{180,-114},{188,-106}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=tauRef.y)
    annotation (Placement(transformation(extent={{78,-98},{98,-78}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{64,-24},{84,-4}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{76,-84},{96,-64}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{76,-140},{96,-120}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor3
                                                                annotation (Placement(transformation(extent={{154,
            -178},{166,-166}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive3(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{96,-182},{116,
            -162}})));
  Aircraft.Mechanical.Loads.Fan fan3
    annotation (Placement(transformation(extent={{182,-176},{190,-168}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=tauRef.y)
    annotation (Placement(transformation(extent={{80,-160},{100,-140}})));
  Modelica.Electrical.Analog.Basic.Ground ground4
    annotation (Placement(transformation(extent={{78,-202},{98,-182}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor4
                                                                annotation (Placement(transformation(extent={{154,
            -242},{166,-230}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive4(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{96,-246},{116,
            -226}})));
  Aircraft.Mechanical.Loads.Fan fan4
    annotation (Placement(transformation(extent={{182,-240},{190,-232}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=tauRef.y)
    annotation (Placement(transformation(extent={{80,-224},{100,-204}})));
  Modelica.Electrical.Analog.Basic.Ground ground5
    annotation (Placement(transformation(extent={{78,-266},{98,-246}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor5
                                                                annotation (Placement(transformation(extent={{154,
            -306},{166,-294}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive5(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{96,-310},{116,
            -290}})));
  Aircraft.Mechanical.Loads.Fan fan5
    annotation (Placement(transformation(extent={{182,-304},{190,-296}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=tauRef.y)
    annotation (Placement(transformation(extent={{80,-288},{100,-268}})));
  Modelica.Electrical.Analog.Basic.Ground ground6
    annotation (Placement(transformation(extent={{78,-330},{98,-310}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor6
                                                                annotation (Placement(transformation(extent={{154,
            -374},{166,-362}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive6(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{96,-378},{116,
            -358}})));
  Aircraft.Mechanical.Loads.Fan fan6
    annotation (Placement(transformation(extent={{182,-372},{190,-364}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=tauRef.y)
    annotation (Placement(transformation(extent={{80,-356},{100,-336}})));
  Modelica.Electrical.Analog.Basic.Ground ground7
    annotation (Placement(transformation(extent={{78,-398},{98,-378}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor7
                                                                annotation (Placement(transformation(extent={{150,
            -438},{162,-426}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive7(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare Aircraft.Electrical.Machines.Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{92,-442},{112,
            -422}})));
  Aircraft.Mechanical.Loads.Fan fan7
    annotation (Placement(transformation(extent={{178,-436},{186,-428}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=tauRef.y)
    annotation (Placement(transformation(extent={{76,-420},{96,-400}})));
  Modelica.Electrical.Analog.Basic.Ground ground8
    annotation (Placement(transformation(extent={{74,-462},{94,-442}})));
  Modelica.Electrical.Analog.Basic.Ground ground10
                                                 annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=0,
        origin={-29,-133})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-28,-94})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-12,-11},{12,11}},
        rotation=270,
        origin={-54,-51})));
equation
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{152,10},{185,10}},
                                              color={0,0,0}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{49,40},{90,40},{90,22}}, color={0,0,127}));
  connect(multiSensor.flange_a, electricDrive.flange) annotation (Line(points={{140,10},
          {100,10}},                            color={0,0,0}));
  connect(multiSensor1.flange_a, electricDrive1.flange)
    annotation (Line(points={{154,-54},{116,-54}}, color={0,0,0}));
  connect(multiSensor1.flange_b, fan1.flange_a1)
    annotation (Line(points={{166,-54},{181,-54}}, color={0,0,0}));
  connect(electricDrive1.desiredSpeed, realExpression.y)
    annotation (Line(points={{106,-42},{106,-30},{97,-30}}, color={0,0,127}));
  connect(multiSensor2.flange_a,electricDrive2. flange)
    annotation (Line(points={{152,-110},{114,-110}},
                                                   color={0,0,0}));
  connect(multiSensor2.flange_b,fan2. flange_a1)
    annotation (Line(points={{164,-110},{179,-110}},
                                                   color={0,0,0}));
  connect(electricDrive2.desiredSpeed, realExpression1.y)
    annotation (Line(points={{104,-98},{104,-88},{99,-88}}, color={0,0,127}));
  connect(electricDrive.pin_n, ground1.p)
    annotation (Line(points={{80,4},{74,4},{74,-4}}, color={0,0,255}));
  connect(electricDrive1.pin_n, ground2.p)
    annotation (Line(points={{96,-60},{86,-60},{86,-64}}, color={0,0,255}));
  connect(electricDrive2.pin_n, ground3.p)
    annotation (Line(points={{94,-116},{86,-116},{86,-120}}, color={0,0,255}));
  connect(multiSensor3.flange_a,electricDrive3. flange)
    annotation (Line(points={{154,-172},{116,-172}},
                                                   color={0,0,0}));
  connect(multiSensor3.flange_b,fan3. flange_a1)
    annotation (Line(points={{166,-172},{181,-172}},
                                                   color={0,0,0}));
  connect(electricDrive3.desiredSpeed, realExpression2.y) annotation (Line(
        points={{106,-160},{106,-150},{101,-150}}, color={0,0,127}));
  connect(electricDrive3.pin_n, ground4.p)
    annotation (Line(points={{96,-178},{88,-178},{88,-182}}, color={0,0,255}));
  connect(multiSensor4.flange_a,electricDrive4. flange)
    annotation (Line(points={{154,-236},{116,-236}},
                                                   color={0,0,0}));
  connect(multiSensor4.flange_b,fan4. flange_a1)
    annotation (Line(points={{166,-236},{181,-236}},
                                                   color={0,0,0}));
  connect(electricDrive4.desiredSpeed, realExpression3.y) annotation (Line(
        points={{106,-224},{106,-214},{101,-214}}, color={0,0,127}));
  connect(electricDrive4.pin_n, ground5.p)
    annotation (Line(points={{96,-242},{88,-242},{88,-246}}, color={0,0,255}));
  connect(multiSensor5.flange_a,electricDrive5. flange)
    annotation (Line(points={{154,-300},{116,-300}},
                                                   color={0,0,0}));
  connect(multiSensor5.flange_b,fan5. flange_a1)
    annotation (Line(points={{166,-300},{181,-300}},
                                                   color={0,0,0}));
  connect(electricDrive5.desiredSpeed, realExpression4.y) annotation (Line(
        points={{106,-288},{106,-278},{101,-278}}, color={0,0,127}));
  connect(electricDrive5.pin_n, ground6.p)
    annotation (Line(points={{96,-306},{88,-306},{88,-310}}, color={0,0,255}));
  connect(multiSensor6.flange_a,electricDrive6. flange)
    annotation (Line(points={{154,-368},{116,-368}},
                                                   color={0,0,0}));
  connect(multiSensor6.flange_b,fan6. flange_a1)
    annotation (Line(points={{166,-368},{181,-368}},
                                                   color={0,0,0}));
  connect(electricDrive6.desiredSpeed, realExpression5.y) annotation (Line(
        points={{106,-356},{106,-346},{101,-346}}, color={0,0,127}));
  connect(electricDrive6.pin_n, ground7.p)
    annotation (Line(points={{96,-374},{88,-374},{88,-378}}, color={0,0,255}));
  connect(electricDrive.pin_p, busExt.n[1]) annotation (Line(points={{80,16},{
          60,16},{60,-36.1},{24,-36.1}}, color={0,0,255}));
  connect(electricDrive1.pin_p, busExt.n[2]) annotation (Line(points={{96,-48},
          {70,-48},{70,-47.5},{24,-47.5}}, color={0,0,255}));
  connect(electricDrive2.pin_p, busExt.n[3]) annotation (Line(points={{94,-104},
          {72,-104},{72,-58.9},{24,-58.9}}, color={0,0,255}));
  connect(electricDrive3.pin_p, busExt.n[4]) annotation (Line(points={{96,-166},
          {68,-166},{68,-70.3},{24,-70.3}}, color={0,0,255}));
  connect(electricDrive4.pin_p, busExt.n[5]) annotation (Line(points={{96,-230},
          {62,-230},{62,-81.7},{24,-81.7}}, color={0,0,255}));
  connect(electricDrive5.pin_p, busExt.n[6]) annotation (Line(points={{96,-294},
          {54,-294},{54,-93.1},{24,-93.1}}, color={0,0,255}));
  connect(electricDrive6.pin_p, busExt.n[7]) annotation (Line(points={{96,-362},
          {46,-362},{46,-104.5},{24,-104.5}}, color={0,0,255}));
  connect(multiSensor7.flange_a,electricDrive7. flange)
    annotation (Line(points={{150,-432},{112,-432}},
                                                   color={0,0,0}));
  connect(multiSensor7.flange_b,fan7. flange_a1)
    annotation (Line(points={{162,-432},{177,-432}},
                                                   color={0,0,0}));
  connect(electricDrive7.desiredSpeed, realExpression6.y) annotation (Line(
        points={{102,-420},{102,-410},{97,-410}}, color={0,0,127}));
  connect(electricDrive7.pin_n, ground8.p)
    annotation (Line(points={{92,-438},{84,-438},{84,-442}}, color={0,0,255}));
  connect(electricDrive7.pin_p, busExt.n[8]) annotation (Line(points={{92,-426},
          {38,-426},{38,-115.9},{24,-115.9}}, color={0,0,255}));
  connect(ground10.p, constantVoltage.n) annotation (Line(points={{-29,-124},{
          -28,-124},{-28,-104}}, color={0,0,255}));
  connect(constantVoltage.p, busExt.p[1]) annotation (Line(points={{-28,-84},{
          -28,-46},{20,-46},{20,-76}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, constantVoltage.n) annotation (Line(points={{-45,
          -53.8235},{-42,-53.8235},{-42,-108},{-28,-108},{-28,-104}},
                                                                color={0,0,255}));
  connect(battery_FC_Charging.p1, constantVoltage.p)
    annotation (Line(points={{-45,-46.7647},{-28,-46.7647},{-28,-84}},
                                                             color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-460},{
            200,60}})),
    Icon(coordinateSystem(extent={{-100,-460},{200,60}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=100,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Battery_FuelCell_CommonBus_noDCDCconverter;
