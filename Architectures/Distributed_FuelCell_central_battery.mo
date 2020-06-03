within CHEETA.Architectures;
model Distributed_FuelCell_central_battery
  "iii. Distributed architecture, with one set of fuel cell stacks routed to one individual propulsor motor (power augmented from battery DC bus)"
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
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{62,-26},{82,-6}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={58,10})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor1
                                                                annotation (Placement(transformation(extent={{140,-56},
            {152,-44}})));
  Aircraft.Mechanical.Loads.Fan fan1
    annotation (Placement(transformation(extent={{186,-54},{194,-46}})));
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
                    annotation (Placement(transformation(extent={{80,-60},{100,
            -40}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{66,-86},{86,-66}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage1(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={58,-52})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=tauRef.y)
    annotation (Placement(transformation(extent={{46,-42},{66,-22}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor2
                                                                annotation (Placement(transformation(extent={{140,
            -116},{152,-104}})));
  Aircraft.Mechanical.Loads.Fan fan2
    annotation (Placement(transformation(extent={{186,-114},{194,-106}})));
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
                    annotation (Placement(transformation(extent={{80,-120},{100,
            -100}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{66,-146},{86,-126}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage2(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={58,-112})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=tauRef.y)
    annotation (Placement(transformation(extent={{46,-102},{66,-82}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor3
                                                                annotation (Placement(transformation(extent={{140,
            -176},{152,-164}})));
  Aircraft.Mechanical.Loads.Fan fan3
    annotation (Placement(transformation(extent={{186,-174},{194,-166}})));
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
                    annotation (Placement(transformation(extent={{80,-180},{100,
            -160}})));
  Modelica.Electrical.Analog.Basic.Ground ground4
    annotation (Placement(transformation(extent={{66,-206},{86,-186}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage3(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={58,-172})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=tauRef.y)
    annotation (Placement(transformation(extent={{46,-162},{66,-142}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor4
                                                                annotation (Placement(transformation(extent={{140,
            -238},{152,-226}})));
  Aircraft.Mechanical.Loads.Fan fan4
    annotation (Placement(transformation(extent={{186,-236},{194,-228}})));
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
                    annotation (Placement(transformation(extent={{80,-242},{100,
            -222}})));
  Modelica.Electrical.Analog.Basic.Ground ground5
    annotation (Placement(transformation(extent={{66,-268},{86,-248}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage4(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={58,-234})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=tauRef.y)
    annotation (Placement(transformation(extent={{46,-224},{66,-204}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor5
                                                                annotation (Placement(transformation(extent={{142,
            -296},{154,-284}})));
  Aircraft.Mechanical.Loads.Fan fan5
    annotation (Placement(transformation(extent={{188,-294},{196,-286}})));
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
                    annotation (Placement(transformation(extent={{82,-300},{102,
            -280}})));
  Modelica.Electrical.Analog.Basic.Ground ground6
    annotation (Placement(transformation(extent={{68,-326},{88,-306}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage5(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={60,-292})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=tauRef.y)
    annotation (Placement(transformation(extent={{48,-282},{68,-262}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor6
                                                                annotation (Placement(transformation(extent={{140,
            -356},{152,-344}})));
  Aircraft.Mechanical.Loads.Fan fan6
    annotation (Placement(transformation(extent={{186,-354},{194,-346}})));
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
                    annotation (Placement(transformation(extent={{80,-360},{100,
            -340}})));
  Modelica.Electrical.Analog.Basic.Ground ground7
    annotation (Placement(transformation(extent={{66,-386},{86,-366}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage6(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={58,-352})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=tauRef.y)
    annotation (Placement(transformation(extent={{46,-342},{66,-322}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor7
                                                                annotation (Placement(transformation(extent={{140,
            -416},{152,-404}})));
  Aircraft.Mechanical.Loads.Fan fan7
    annotation (Placement(transformation(extent={{186,-414},{194,-406}})));
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
                    annotation (Placement(transformation(extent={{80,-420},{100,
            -400}})));
  Modelica.Electrical.Analog.Basic.Ground ground8
    annotation (Placement(transformation(extent={{66,-446},{86,-426}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage7(V=10)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={58,-412})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=tauRef.y)
    annotation (Placement(transformation(extent={{46,-402},{66,-382}})));
  Aircraft.Electrical.BusExt busExt(nn=1, np=1)
    annotation (Placement(transformation(extent={{-70,48},{-74,-104}})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-12,-11},{12,11}},
        rotation=270,
        origin={-116,-45})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-108,-96},{-88,-76}})));
equation
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{152,10},{185,10}},
                                              color={0,0,0}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{49,40},{90,40},{90,22}}, color={0,0,127}));
  connect(multiSensor.flange_a, electricDrive.flange) annotation (Line(points={{140,10},
          {100,10}},                            color={0,0,0}));
  connect(electricDrive.pin_n, ground1.p)
    annotation (Line(points={{80,4},{72,4},{72,-6}}, color={0,0,255}));
  connect(electricDrive.pin_p, constantVoltage.p)
    annotation (Line(points={{80,16},{58,16}}, color={0,0,255}));
  connect(constantVoltage.n, ground1.p)
    annotation (Line(points={{58,4},{72,4},{72,-6}}, color={0,0,255}));
  connect(multiSensor1.flange_b, fan1.flange_a1)
    annotation (Line(points={{152,-50},{185,-50}}, color={0,0,0}));
  connect(multiSensor1.flange_a, electricDrive1.flange)
    annotation (Line(points={{140,-50},{100,-50}}, color={0,0,0}));
  connect(electricDrive1.pin_n, ground2.p)
    annotation (Line(points={{80,-56},{76,-56},{76,-66}}, color={0,0,255}));
  connect(electricDrive1.pin_p, constantVoltage1.p)
    annotation (Line(points={{80,-44},{58,-44},{58,-46}}, color={0,0,255}));
  connect(constantVoltage1.n, ground2.p) annotation (Line(points={{58,-58},{58,
          -62},{76,-62},{76,-66}}, color={0,0,255}));
  connect(realExpression.y, electricDrive1.desiredSpeed)
    annotation (Line(points={{67,-32},{90,-32},{90,-38}}, color={0,0,127}));
  connect(multiSensor2.flange_b, fan2.flange_a1)
    annotation (Line(points={{152,-110},{185,-110}}, color={0,0,0}));
  connect(multiSensor2.flange_a, electricDrive2.flange)
    annotation (Line(points={{140,-110},{100,-110}}, color={0,0,0}));
  connect(electricDrive2.pin_n, ground3.p)
    annotation (Line(points={{80,-116},{76,-116},{76,-126}}, color={0,0,255}));
  connect(electricDrive2.pin_p, constantVoltage2.p)
    annotation (Line(points={{80,-104},{58,-104},{58,-106}}, color={0,0,255}));
  connect(constantVoltage2.n, ground3.p) annotation (Line(points={{58,-118},{58,
          -122},{76,-122},{76,-126}}, color={0,0,255}));
  connect(realExpression1.y, electricDrive2.desiredSpeed)
    annotation (Line(points={{67,-92},{90,-92},{90,-98}}, color={0,0,127}));
  connect(multiSensor3.flange_b, fan3.flange_a1)
    annotation (Line(points={{152,-170},{185,-170}}, color={0,0,0}));
  connect(multiSensor3.flange_a, electricDrive3.flange)
    annotation (Line(points={{140,-170},{100,-170}}, color={0,0,0}));
  connect(electricDrive3.pin_n, ground4.p)
    annotation (Line(points={{80,-176},{76,-176},{76,-186}}, color={0,0,255}));
  connect(electricDrive3.pin_p, constantVoltage3.p)
    annotation (Line(points={{80,-164},{58,-164},{58,-166}}, color={0,0,255}));
  connect(constantVoltage3.n, ground4.p) annotation (Line(points={{58,-178},{58,
          -182},{76,-182},{76,-186}}, color={0,0,255}));
  connect(realExpression2.y, electricDrive3.desiredSpeed)
    annotation (Line(points={{67,-152},{90,-152},{90,-158}}, color={0,0,127}));
  connect(multiSensor4.flange_b, fan4.flange_a1)
    annotation (Line(points={{152,-232},{185,-232}}, color={0,0,0}));
  connect(multiSensor4.flange_a, electricDrive4.flange)
    annotation (Line(points={{140,-232},{100,-232}}, color={0,0,0}));
  connect(electricDrive4.pin_n, ground5.p)
    annotation (Line(points={{80,-238},{76,-238},{76,-248}}, color={0,0,255}));
  connect(electricDrive4.pin_p, constantVoltage4.p)
    annotation (Line(points={{80,-226},{58,-226},{58,-228}}, color={0,0,255}));
  connect(constantVoltage4.n, ground5.p) annotation (Line(points={{58,-240},{58,
          -244},{76,-244},{76,-248}}, color={0,0,255}));
  connect(realExpression3.y, electricDrive4.desiredSpeed)
    annotation (Line(points={{67,-214},{90,-214},{90,-220}}, color={0,0,127}));
  connect(multiSensor5.flange_b, fan5.flange_a1)
    annotation (Line(points={{154,-290},{187,-290}}, color={0,0,0}));
  connect(multiSensor5.flange_a, electricDrive5.flange)
    annotation (Line(points={{142,-290},{102,-290}}, color={0,0,0}));
  connect(electricDrive5.pin_n, ground6.p)
    annotation (Line(points={{82,-296},{78,-296},{78,-306}}, color={0,0,255}));
  connect(electricDrive5.pin_p, constantVoltage5.p)
    annotation (Line(points={{82,-284},{60,-284},{60,-286}}, color={0,0,255}));
  connect(constantVoltage5.n, ground6.p) annotation (Line(points={{60,-298},{60,
          -302},{78,-302},{78,-306}}, color={0,0,255}));
  connect(realExpression4.y, electricDrive5.desiredSpeed)
    annotation (Line(points={{69,-272},{92,-272},{92,-278}}, color={0,0,127}));
  connect(multiSensor6.flange_b, fan6.flange_a1)
    annotation (Line(points={{152,-350},{185,-350}}, color={0,0,0}));
  connect(multiSensor6.flange_a, electricDrive6.flange)
    annotation (Line(points={{140,-350},{100,-350}}, color={0,0,0}));
  connect(electricDrive6.pin_n, ground7.p)
    annotation (Line(points={{80,-356},{76,-356},{76,-366}}, color={0,0,255}));
  connect(electricDrive6.pin_p, constantVoltage6.p)
    annotation (Line(points={{80,-344},{58,-344},{58,-346}}, color={0,0,255}));
  connect(constantVoltage6.n, ground7.p) annotation (Line(points={{58,-358},{58,
          -362},{76,-362},{76,-366}}, color={0,0,255}));
  connect(realExpression5.y, electricDrive6.desiredSpeed)
    annotation (Line(points={{67,-332},{90,-332},{90,-338}}, color={0,0,127}));
  connect(multiSensor7.flange_b, fan7.flange_a1)
    annotation (Line(points={{152,-410},{185,-410}}, color={0,0,0}));
  connect(multiSensor7.flange_a, electricDrive7.flange)
    annotation (Line(points={{140,-410},{100,-410}}, color={0,0,0}));
  connect(electricDrive7.pin_n, ground8.p)
    annotation (Line(points={{80,-416},{76,-416},{76,-426}}, color={0,0,255}));
  connect(electricDrive7.pin_p, constantVoltage7.p)
    annotation (Line(points={{80,-404},{58,-404},{58,-406}}, color={0,0,255}));
  connect(constantVoltage7.n, ground8.p) annotation (Line(points={{58,-418},{58,
          -422},{76,-422},{76,-426}}, color={0,0,255}));
  connect(realExpression6.y, electricDrive7.desiredSpeed)
    annotation (Line(points={{67,-392},{90,-392},{90,-398}}, color={0,0,127}));
  connect(busExt.n[1], constantVoltage.p) annotation (Line(points={{-70,-28},{
          20,-28},{20,16},{58,16}},  color={0,0,255}));
  connect(battery_FC_Charging.p1, busExt.p[1]) annotation (Line(points={{-107,
          -40},{-74,-40},{-74,-28}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, ground9.p) annotation (Line(points={{-107,-50},
          {-98,-50},{-98,-76}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-460},{
            200,60}})),
    Icon(coordinateSystem(extent={{-240,-460},{200,60}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=100,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Distributed_FuelCell_central_battery;
