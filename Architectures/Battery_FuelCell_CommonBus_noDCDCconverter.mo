within CHEETA.Architectures;
model Battery_FuelCell_CommonBus_noDCDCconverter
  "ii. Centralized architecture in i., but without DC/DC converter"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{48,20},{68,40}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{150,-8},
            {162,4}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel(J=10)
    annotation (Placement(transformation(extent={{176,-6},{184,2}})));
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
    annotation (Placement(transformation(extent={{88,-12},{108,8}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{72,-38},{92,-18}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-50,-10})));
  Aircraft.Electrical.Battery.Battery_BMS battery_FC_Charging annotation (
      Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-80.5,13.5})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-78,-16},{-58,4}})));
  Aircraft.Electrical.BusExt busExt(np=2, nn=2)
    annotation (Placement(transformation(extent={{-14,-74},{-16,38}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-106,-22},{-86,-2}})));
  Aircraft.Electrical.BusExt busExt1(nn=4, np=1)
    annotation (Placement(transformation(extent={{38,38},{36,-38}})));
  Aircraft.Electrical.BusExt busExt2(np=1, nn=4)
    annotation (Placement(transformation(extent={{38,-328},{36,-404}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor1
                                                                annotation (Placement(transformation(extent={{152,-76},
            {164,-64}})));
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
    annotation (Placement(transformation(extent={{94,-80},{114,-60}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel1
    annotation (Placement(transformation(extent={{178,-74},{186,-66}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=tauRef.y)
    annotation (Placement(transformation(extent={{74,-56},{94,-36}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{74,-100},{94,-80}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor2
                                                                annotation (Placement(transformation(extent={{150,
            -142},{162,-130}})));
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
    annotation (Placement(transformation(extent={{92,-146},{112,-126}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel3
    annotation (Placement(transformation(extent={{176,-140},{184,-132}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=tauRef.y)
    annotation (Placement(transformation(extent={{72,-122},{92,-102}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{72,-166},{92,-146}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor3
                                                                annotation (Placement(transformation(extent={{150,
            -208},{162,-196}})));
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
    annotation (Placement(transformation(extent={{92,-212},{112,-192}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel4
    annotation (Placement(transformation(extent={{176,-206},{184,-198}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=tauRef.y)
    annotation (Placement(transformation(extent={{72,-188},{92,-168}})));
  Modelica.Electrical.Analog.Basic.Ground ground4
    annotation (Placement(transformation(extent={{72,-232},{92,-212}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor4
                                                                annotation (Placement(transformation(extent={{150,
            -280},{162,-268}})));
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
    annotation (Placement(transformation(extent={{92,-284},{112,-264}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel5
    annotation (Placement(transformation(extent={{176,-278},{184,-270}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=tauRef.y)
    annotation (Placement(transformation(extent={{72,-260},{92,-240}})));
  Modelica.Electrical.Analog.Basic.Ground ground5
    annotation (Placement(transformation(extent={{72,-304},{92,-284}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor5
                                                                annotation (Placement(transformation(extent={{150,
            -348},{162,-336}})));
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
    annotation (Placement(transformation(extent={{92,-352},{112,-332}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel6
    annotation (Placement(transformation(extent={{176,-346},{184,-338}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=tauRef.y)
    annotation (Placement(transformation(extent={{72,-328},{92,-308}})));
  Modelica.Electrical.Analog.Basic.Ground ground6
    annotation (Placement(transformation(extent={{72,-372},{92,-352}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor6
                                                                annotation (Placement(transformation(extent={{150,
            -412},{162,-400}})));
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
    annotation (Placement(transformation(extent={{92,-416},{112,-396}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel7
    annotation (Placement(transformation(extent={{176,-410},{184,-402}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=tauRef.y)
    annotation (Placement(transformation(extent={{72,-392},{92,-372}})));
  Modelica.Electrical.Analog.Basic.Ground ground7
    annotation (Placement(transformation(extent={{72,-436},{92,-416}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor7
                                                                annotation (Placement(transformation(extent={{152,
            -484},{164,-472}})));
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
    annotation (Placement(transformation(extent={{94,-488},{114,-468}})));
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel2
    annotation (Placement(transformation(extent={{180,-482},{188,-474}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=tauRef.y)
    annotation (Placement(transformation(extent={{74,-464},{94,-444}})));
  Modelica.Electrical.Analog.Basic.Ground ground10
    annotation (Placement(transformation(extent={{108,-516},{128,-496}})));
equation
  connect(multiSensor.flange_b, pinwheel.flange_a1)
    annotation (Line(points={{162,-2},{176,-2}}, color={0,0,0}));
  connect(tauRef.y,electricDrive. desiredSpeed)
    annotation (Line(points={{69,30},{98,30},{98,10}}, color={0,0,127}));
  connect(multiSensor.flange_a,electricDrive. flange) annotation (Line(points={{150,-2},
          {108,-2}},                            color={0,0,0}));
  connect(electricDrive.pin_n,ground1. p)
    annotation (Line(points={{88,-8},{82,-8},{82,-18}},
                                                     color={0,0,255}));
  connect(battery_FC_Charging.n1,ground9. p)
    annotation (Line(points={{-71.0909,9.61765},{-68,9.61765},{-68,4}},
                                                             color={0,0,255}));
  connect(battery_FC_Charging.n1, constantVoltage.n) annotation (Line(points={{
          -71.0909,9.61765},{-58,9.61765},{-58,-10},{-56,-10}}, color={0,0,255}));
  connect(constantVoltage.p, busExt.p[1]) annotation (Line(points={{-44,-10},{
          -40,-10},{-40,-34.8},{-16,-34.8}}, color={0,0,255}));
  connect(battery_FC_Charging.p1,busExt. p[1]) annotation (Line(points={{
          -71.0909,19.3235},{-30,19.3235},{-30,-34.8},{-16,-34.8}},
                                                color={0,0,255}));
  connect(booleanExpression.y,battery_FC_Charging. u1) annotation (Line(points={{-85,-12},
          {-79.4545,-12},{-79.4545,0.88235}},        color={255,0,255}));
  connect(busExt1.n[1], electricDrive.pin_p) annotation (Line(points={{38,17.1},
          {64,17.1},{64,4},{88,4}}, color={0,0,255}));
  connect(multiSensor1.flange_a,electricDrive1. flange)
    annotation (Line(points={{152,-70},{114,-70}}, color={0,0,0}));
  connect(multiSensor1.flange_b, pinwheel1.flange_a1)
    annotation (Line(points={{164,-70},{178,-70}}, color={0,0,0}));
  connect(electricDrive1.desiredSpeed,realExpression. y)
    annotation (Line(points={{104,-58},{104,-46},{95,-46}}, color={0,0,127}));
  connect(electricDrive1.pin_n,ground2. p)
    annotation (Line(points={{94,-76},{84,-76},{84,-80}}, color={0,0,255}));
  connect(electricDrive1.pin_p, busExt1.n[1]) annotation (Line(points={{94,-64},
          {78,-64},{78,-58},{60,-58},{60,17.1},{38,17.1}}, color={0,0,255}));
  connect(multiSensor2.flange_a,electricDrive2. flange)
    annotation (Line(points={{150,-136},{112,-136}},
                                                   color={0,0,0}));
  connect(multiSensor2.flange_b, pinwheel3.flange_a1)
    annotation (Line(points={{162,-136},{176,-136}}, color={0,0,0}));
  connect(electricDrive2.desiredSpeed, realExpression1.y) annotation (Line(
        points={{102,-124},{102,-112},{93,-112}}, color={0,0,127}));
  connect(electricDrive2.pin_n,ground3. p)
    annotation (Line(points={{92,-142},{82,-142},{82,-146}},
                                                          color={0,0,255}));
  connect(busExt1.p[1], busExt.n[1]) annotation (Line(points={{36,0},{10,0},{10,
          -34.8},{-14,-34.8}}, color={0,0,255}));
  connect(busExt2.p[1], busExt.n[1]) annotation (Line(points={{36,-366},{24,
          -366},{24,-20},{-14,-20},{-14,-34.8}}, color={0,0,255}));
  connect(multiSensor3.flange_a,electricDrive3. flange)
    annotation (Line(points={{150,-202},{112,-202}},
                                                   color={0,0,0}));
  connect(multiSensor3.flange_b, pinwheel4.flange_a1)
    annotation (Line(points={{162,-202},{176,-202}}, color={0,0,0}));
  connect(electricDrive3.desiredSpeed, realExpression2.y) annotation (Line(
        points={{102,-190},{102,-178},{93,-178}}, color={0,0,127}));
  connect(electricDrive3.pin_n,ground4. p)
    annotation (Line(points={{92,-208},{82,-208},{82,-212}},
                                                          color={0,0,255}));
  connect(multiSensor4.flange_a,electricDrive4. flange)
    annotation (Line(points={{150,-274},{112,-274}},
                                                   color={0,0,0}));
  connect(multiSensor4.flange_b, pinwheel5.flange_a1)
    annotation (Line(points={{162,-274},{176,-274}}, color={0,0,0}));
  connect(electricDrive4.desiredSpeed, realExpression3.y) annotation (Line(
        points={{102,-262},{102,-250},{93,-250}}, color={0,0,127}));
  connect(electricDrive4.pin_n,ground5. p)
    annotation (Line(points={{92,-280},{82,-280},{82,-284}},
                                                          color={0,0,255}));
  connect(electricDrive2.pin_p, busExt1.n[2]) annotation (Line(points={{92,-130},
          {62,-130},{62,-88},{52,-88},{52,5.7},{38,5.7}}, color={0,0,255}));
  connect(electricDrive3.pin_p, busExt1.n[2]) annotation (Line(points={{92,-196},
          {64,-196},{64,-160},{46,-160},{46,5.7},{38,5.7}}, color={0,0,255}));
  connect(multiSensor5.flange_a,electricDrive5. flange)
    annotation (Line(points={{150,-342},{112,-342}},
                                                   color={0,0,0}));
  connect(multiSensor5.flange_b, pinwheel6.flange_a1)
    annotation (Line(points={{162,-342},{176,-342}}, color={0,0,0}));
  connect(electricDrive5.desiredSpeed, realExpression4.y) annotation (Line(
        points={{102,-330},{102,-318},{93,-318}}, color={0,0,127}));
  connect(electricDrive5.pin_n,ground6. p)
    annotation (Line(points={{92,-348},{82,-348},{82,-352}},
                                                          color={0,0,255}));
  connect(multiSensor6.flange_a,electricDrive6. flange)
    annotation (Line(points={{150,-406},{112,-406}},
                                                   color={0,0,0}));
  connect(multiSensor6.flange_b, pinwheel7.flange_a1)
    annotation (Line(points={{162,-406},{176,-406}}, color={0,0,0}));
  connect(electricDrive6.desiredSpeed, realExpression5.y) annotation (Line(
        points={{102,-394},{102,-382},{93,-382}}, color={0,0,127}));
  connect(electricDrive6.pin_n,ground7. p)
    annotation (Line(points={{92,-412},{82,-412},{82,-416}},
                                                          color={0,0,255}));
  connect(multiSensor7.flange_a,electricDrive7. flange)
    annotation (Line(points={{152,-478},{114,-478}},
                                                   color={0,0,0}));
  connect(multiSensor7.flange_b, pinwheel2.flange_a1)
    annotation (Line(points={{164,-478},{180,-478}}, color={0,0,0}));
  connect(electricDrive7.desiredSpeed, realExpression6.y) annotation (Line(
        points={{104,-466},{104,-454},{95,-454}}, color={0,0,127}));
  connect(electricDrive7.pin_n, ground10.p) annotation (Line(points={{94,-484},
          {118,-484},{118,-496}}, color={0,0,255}));
  connect(electricDrive4.pin_p, busExt2.n[1]) annotation (Line(points={{92,-268},
          {66,-268},{66,-348.9},{38,-348.9}}, color={0,0,255}));
  connect(electricDrive5.pin_p, busExt2.n[1]) annotation (Line(points={{92,-336},
          {70,-336},{70,-348.9},{38,-348.9}}, color={0,0,255}));
  connect(electricDrive6.pin_p, busExt2.n[2]) annotation (Line(points={{92,-400},
          {62,-400},{62,-360.3},{38,-360.3}}, color={0,0,255}));
  connect(electricDrive7.pin_p, busExt2.n[2]) annotation (Line(points={{94,-472},
          {54,-472},{54,-360.3},{38,-360.3}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-520},{
            220,60}})),
    Icon(coordinateSystem(extent={{-120,-520},{220,60}},  preserveAspectRatio=false), graphics),
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
