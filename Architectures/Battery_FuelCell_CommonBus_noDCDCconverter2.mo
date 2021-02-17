within CHEETA.Architectures;
model Battery_FuelCell_CommonBus_noDCDCconverter2
  "ii. Centralized architecture in i., but without DC/DC converter"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{36,24},{56,44}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{138,-4},
            {150,8}})));
  Aircraft.Mechanical.Loads.Fan fan(J=10)
    annotation (Placement(transformation(extent={{164,-2},{172,6}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{76,-8},{96,12}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{60,-34},{80,-14}})));
  Modelica.Electrical.Analog.Sources.CosineVoltage cosineVoltage(
    V=450,
    f=0.1,
    offset=550) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-62,-6})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-92.5,17.5})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-90,-12},{-70,8}})));
  Aircraft.Electrical.BusExt busExt(np=2, nn=2)
    annotation (Placement(transformation(extent={{-26,-70},{-28,42}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-118,-18},{-98,2}})));
  Aircraft.Electrical.BusExt busExt1(nn=4, np=1)
    annotation (Placement(transformation(extent={{26,42},{24,-34}})));
  Aircraft.Electrical.BusExt busExt2(np=1, nn=4)
    annotation (Placement(transformation(extent={{26,-324},{24,-400}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor1
                                                                annotation (Placement(transformation(extent={{140,-72},
            {152,-60}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive1(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{82,-76},{102,-56}})));
  Aircraft.Mechanical.Loads.Fan fan1
    annotation (Placement(transformation(extent={{166,-70},{174,-62}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=tauRef.y)
    annotation (Placement(transformation(extent={{62,-52},{82,-32}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{62,-96},{82,-76}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor2
                                                                annotation (Placement(transformation(extent={{138,
            -138},{150,-126}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive2(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{80,-142},{100,-122}})));
  Aircraft.Mechanical.Loads.Fan fan2
    annotation (Placement(transformation(extent={{164,-136},{172,-128}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=tauRef.y)
    annotation (Placement(transformation(extent={{60,-118},{80,-98}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{60,-162},{80,-142}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor3
                                                                annotation (Placement(transformation(extent={{138,
            -204},{150,-192}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive3(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{80,-208},{100,-188}})));
  Aircraft.Mechanical.Loads.Fan fan3
    annotation (Placement(transformation(extent={{164,-202},{172,-194}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=tauRef.y)
    annotation (Placement(transformation(extent={{60,-184},{80,-164}})));
  Modelica.Electrical.Analog.Basic.Ground ground4
    annotation (Placement(transformation(extent={{60,-228},{80,-208}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor4
                                                                annotation (Placement(transformation(extent={{138,
            -276},{150,-264}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive4(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{80,-280},{100,-260}})));
  Aircraft.Mechanical.Loads.Fan fan4
    annotation (Placement(transformation(extent={{164,-274},{172,-266}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=tauRef.y)
    annotation (Placement(transformation(extent={{60,-256},{80,-236}})));
  Modelica.Electrical.Analog.Basic.Ground ground5
    annotation (Placement(transformation(extent={{60,-300},{80,-280}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor5
                                                                annotation (Placement(transformation(extent={{138,
            -344},{150,-332}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive5(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{80,-348},{100,-328}})));
  Aircraft.Mechanical.Loads.Fan fan5
    annotation (Placement(transformation(extent={{164,-342},{172,-334}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=tauRef.y)
    annotation (Placement(transformation(extent={{60,-324},{80,-304}})));
  Modelica.Electrical.Analog.Basic.Ground ground6
    annotation (Placement(transformation(extent={{60,-368},{80,-348}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor6
                                                                annotation (Placement(transformation(extent={{138,
            -408},{150,-396}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive6(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{80,-412},{100,-392}})));
  Aircraft.Mechanical.Loads.Fan fan6
    annotation (Placement(transformation(extent={{164,-406},{172,-398}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=tauRef.y)
    annotation (Placement(transformation(extent={{60,-388},{80,-368}})));
  Modelica.Electrical.Analog.Basic.Ground ground7
    annotation (Placement(transformation(extent={{60,-432},{80,-412}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor7
                                                                annotation (Placement(transformation(extent={{140,
            -480},{152,-468}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC
    electricDrive7(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare
          Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
    annotation (Placement(transformation(extent={{82,-484},{102,-464}})));
  Aircraft.Mechanical.Loads.Fan fan7
    annotation (Placement(transformation(extent={{168,-478},{176,-470}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=tauRef.y)
    annotation (Placement(transformation(extent={{62,-460},{82,-440}})));
  Modelica.Electrical.Analog.Basic.Ground ground8
    annotation (Placement(transformation(extent={{62,-504},{82,-484}})));
equation
  connect(multiSensor.flange_b,fan. flange_a1)
    annotation (Line(points={{150,2},{163,2}},color={0,0,0}));
  connect(tauRef.y,electricDrive. desiredSpeed)
    annotation (Line(points={{57,34},{86,34},{86,14}}, color={0,0,127}));
  connect(multiSensor.flange_a,electricDrive. flange) annotation (Line(points={{138,2},
          {96,2}},                              color={0,0,0}));
  connect(electricDrive.pin_n,ground1. p)
    annotation (Line(points={{76,-4},{70,-4},{70,-14}},
                                                     color={0,0,255}));
  connect(battery_FC_Charging.n1,ground9. p)
    annotation (Line(points={{-83.0909,13.6176},{-80,13.6176},{-80,8}},
                                                             color={0,0,255}));
  connect(battery_FC_Charging.n1,cosineVoltage. n) annotation (Line(points={{
          -83.0909,13.6176},{-70,13.6176},{-70,-6},{-68,-6}},
                                                   color={0,0,255}));
  connect(cosineVoltage.p,busExt. p[1]) annotation (Line(points={{-56,-6},{-52,
          -6},{-52,-30.8},{-28,-30.8}}, color={0,0,255}));
  connect(battery_FC_Charging.p1,busExt. p[2]) annotation (Line(points={{
          -83.0909,23.3235},{-66,23.3235},{-66,2.8},{-28,2.8}},
                                                color={0,0,255}));
  connect(booleanExpression.y,battery_FC_Charging. u1) annotation (Line(points={{-97,-8},
          {-91.4545,-8},{-91.4545,4.88235}},         color={255,0,255}));
  connect(busExt1.n[1], electricDrive.pin_p) annotation (Line(points={{26,21.1},
          {52,21.1},{52,8},{76,8}}, color={0,0,255}));
  connect(multiSensor1.flange_a,electricDrive1. flange)
    annotation (Line(points={{140,-66},{102,-66}}, color={0,0,0}));
  connect(multiSensor1.flange_b,fan1. flange_a1)
    annotation (Line(points={{152,-66},{165,-66}}, color={0,0,0}));
  connect(electricDrive1.desiredSpeed,realExpression. y)
    annotation (Line(points={{92,-54},{92,-42},{83,-42}},   color={0,0,127}));
  connect(electricDrive1.pin_n,ground2. p)
    annotation (Line(points={{82,-72},{72,-72},{72,-76}}, color={0,0,255}));
  connect(electricDrive1.pin_p, busExt1.n[2]) annotation (Line(points={{82,-60},
          {66,-60},{66,-54},{48,-54},{48,9.7},{26,9.7}}, color={0,0,255}));
  connect(multiSensor2.flange_a,electricDrive2. flange)
    annotation (Line(points={{138,-132},{100,-132}},
                                                   color={0,0,0}));
  connect(multiSensor2.flange_b,fan2. flange_a1)
    annotation (Line(points={{150,-132},{163,-132}},
                                                   color={0,0,0}));
  connect(electricDrive2.desiredSpeed, realExpression1.y)
    annotation (Line(points={{90,-120},{90,-108},{81,-108}}, color={0,0,127}));
  connect(electricDrive2.pin_n,ground3. p)
    annotation (Line(points={{80,-138},{70,-138},{70,-142}},
                                                          color={0,0,255}));
  connect(busExt1.p[1], busExt.n[1]) annotation (Line(points={{24,4},{-2,4},{-2,
          -30.8},{-26,-30.8}}, color={0,0,255}));
  connect(busExt2.p[1], busExt.n[2]) annotation (Line(points={{24,-362},{12,
          -362},{12,-16},{-26,-16},{-26,2.8}}, color={0,0,255}));
  connect(multiSensor3.flange_a,electricDrive3. flange)
    annotation (Line(points={{138,-198},{100,-198}},
                                                   color={0,0,0}));
  connect(multiSensor3.flange_b,fan3. flange_a1)
    annotation (Line(points={{150,-198},{163,-198}},
                                                   color={0,0,0}));
  connect(electricDrive3.desiredSpeed, realExpression2.y)
    annotation (Line(points={{90,-186},{90,-174},{81,-174}}, color={0,0,127}));
  connect(electricDrive3.pin_n,ground4. p)
    annotation (Line(points={{80,-204},{70,-204},{70,-208}},
                                                          color={0,0,255}));
  connect(multiSensor4.flange_a,electricDrive4. flange)
    annotation (Line(points={{138,-270},{100,-270}},
                                                   color={0,0,0}));
  connect(multiSensor4.flange_b,fan4. flange_a1)
    annotation (Line(points={{150,-270},{163,-270}},
                                                   color={0,0,0}));
  connect(electricDrive4.desiredSpeed, realExpression3.y)
    annotation (Line(points={{90,-258},{90,-246},{81,-246}}, color={0,0,127}));
  connect(electricDrive4.pin_n,ground5. p)
    annotation (Line(points={{80,-276},{70,-276},{70,-280}},
                                                          color={0,0,255}));
  connect(electricDrive2.pin_p, busExt1.n[3]) annotation (Line(points={{80,-126},
          {50,-126},{50,-84},{40,-84},{40,-1.7},{26,-1.7}}, color={0,0,255}));
  connect(electricDrive3.pin_p, busExt1.n[4]) annotation (Line(points={{80,-192},
          {52,-192},{52,-156},{34,-156},{34,-13.1},{26,-13.1}}, color={0,0,255}));
  connect(multiSensor5.flange_a,electricDrive5. flange)
    annotation (Line(points={{138,-338},{100,-338}},
                                                   color={0,0,0}));
  connect(multiSensor5.flange_b,fan5. flange_a1)
    annotation (Line(points={{150,-338},{163,-338}},
                                                   color={0,0,0}));
  connect(electricDrive5.desiredSpeed, realExpression4.y)
    annotation (Line(points={{90,-326},{90,-314},{81,-314}}, color={0,0,127}));
  connect(electricDrive5.pin_n,ground6. p)
    annotation (Line(points={{80,-344},{70,-344},{70,-348}},
                                                          color={0,0,255}));
  connect(multiSensor6.flange_a,electricDrive6. flange)
    annotation (Line(points={{138,-402},{100,-402}},
                                                   color={0,0,0}));
  connect(multiSensor6.flange_b,fan6. flange_a1)
    annotation (Line(points={{150,-402},{163,-402}},
                                                   color={0,0,0}));
  connect(electricDrive6.desiredSpeed, realExpression5.y)
    annotation (Line(points={{90,-390},{90,-378},{81,-378}}, color={0,0,127}));
  connect(electricDrive6.pin_n,ground7. p)
    annotation (Line(points={{80,-408},{70,-408},{70,-412}},
                                                          color={0,0,255}));
  connect(multiSensor7.flange_a,electricDrive7. flange)
    annotation (Line(points={{140,-474},{102,-474}},
                                                   color={0,0,0}));
  connect(multiSensor7.flange_b,fan7. flange_a1)
    annotation (Line(points={{152,-474},{167,-474}},
                                                   color={0,0,0}));
  connect(electricDrive7.desiredSpeed, realExpression6.y)
    annotation (Line(points={{92,-462},{92,-450},{83,-450}}, color={0,0,127}));
  connect(electricDrive7.pin_n,ground8. p)
    annotation (Line(points={{82,-480},{72,-480},{72,-484}},
                                                          color={0,0,255}));
  connect(electricDrive4.pin_p, busExt2.n[1]) annotation (Line(points={{80,-264},
          {54,-264},{54,-344.9},{26,-344.9}}, color={0,0,255}));
  connect(electricDrive5.pin_p, busExt2.n[2]) annotation (Line(points={{80,-332},
          {58,-332},{58,-356.3},{26,-356.3}}, color={0,0,255}));
  connect(electricDrive6.pin_p, busExt2.n[3]) annotation (Line(points={{80,-396},
          {50,-396},{50,-367.7},{26,-367.7}}, color={0,0,255}));
  connect(electricDrive7.pin_p, busExt2.n[4]) annotation (Line(points={{82,-468},
          {42,-468},{42,-379.1},{26,-379.1}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-500},{
            180,60}})),
    Icon(coordinateSystem(extent={{-120,-500},{180,60}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=100,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Battery_FuelCell_CommonBus_noDCDCconverter2;
