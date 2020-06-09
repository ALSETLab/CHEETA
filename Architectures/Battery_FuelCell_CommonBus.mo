within CHEETA.Architectures;
model Battery_FuelCell_CommonBus
  "i. Centralized architecture with battery/fuel cell systems providing power to a common DC bus, which is drawn from by each propulsor"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{-8,24},{12,44}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{104,-2},
            {116,10}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                annotation (Placement(transformation(extent={{150,38},
            {162,50}})));
  Aircraft.Mechanical.Loads.Fan fan
    annotation (Placement(transformation(extent={{150,0},{158,8}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
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
                    annotation (Placement(transformation(extent={{44,-6},{64,14}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor1
                                                                annotation (Placement(transformation(extent={{118,-66},
            {130,-54}})));
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
                    annotation (Placement(transformation(extent={{60,-70},{80,
            -50}})));
  Aircraft.Mechanical.Loads.Fan fan1
    annotation (Placement(transformation(extent={{146,-64},{154,-56}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=tauRef.y)
    annotation (Placement(transformation(extent={{40,-46},{60,-26}})));
  Aircraft.Electrical.BusExt busExt(nn=8, np=1)
    annotation (Placement(transformation(extent={{-12,-6},{-16,-158}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor2
                                                                annotation (Placement(transformation(extent={{116,
            -122},{128,-110}})));
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
                    annotation (Placement(transformation(extent={{58,-126},{78,
            -106}})));
  Aircraft.Mechanical.Loads.Fan fan2
    annotation (Placement(transformation(extent={{144,-120},{152,-112}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=tauRef.y)
    annotation (Placement(transformation(extent={{42,-104},{62,-84}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{28,-30},{48,-10}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{40,-90},{60,-70}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{40,-146},{60,-126}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor3
                                                                annotation (Placement(transformation(extent={{118,
            -184},{130,-172}})));
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
                    annotation (Placement(transformation(extent={{60,-188},{80,
            -168}})));
  Aircraft.Mechanical.Loads.Fan fan3
    annotation (Placement(transformation(extent={{146,-182},{154,-174}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=tauRef.y)
    annotation (Placement(transformation(extent={{44,-166},{64,-146}})));
  Modelica.Electrical.Analog.Basic.Ground ground4
    annotation (Placement(transformation(extent={{42,-208},{62,-188}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor4
                                                                annotation (Placement(transformation(extent={{118,
            -248},{130,-236}})));
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
                    annotation (Placement(transformation(extent={{60,-252},{80,
            -232}})));
  Aircraft.Mechanical.Loads.Fan fan4
    annotation (Placement(transformation(extent={{146,-246},{154,-238}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=tauRef.y)
    annotation (Placement(transformation(extent={{44,-230},{64,-210}})));
  Modelica.Electrical.Analog.Basic.Ground ground5
    annotation (Placement(transformation(extent={{42,-272},{62,-252}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor5
                                                                annotation (Placement(transformation(extent={{118,
            -312},{130,-300}})));
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
                    annotation (Placement(transformation(extent={{60,-316},{80,
            -296}})));
  Aircraft.Mechanical.Loads.Fan fan5
    annotation (Placement(transformation(extent={{146,-310},{154,-302}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=tauRef.y)
    annotation (Placement(transformation(extent={{44,-294},{64,-274}})));
  Modelica.Electrical.Analog.Basic.Ground ground6
    annotation (Placement(transformation(extent={{42,-336},{62,-316}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor6
                                                                annotation (Placement(transformation(extent={{118,
            -380},{130,-368}})));
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
                    annotation (Placement(transformation(extent={{60,-384},{80,
            -364}})));
  Aircraft.Mechanical.Loads.Fan fan6
    annotation (Placement(transformation(extent={{146,-378},{154,-370}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=tauRef.y)
    annotation (Placement(transformation(extent={{44,-362},{64,-342}})));
  Modelica.Electrical.Analog.Basic.Ground ground7
    annotation (Placement(transformation(extent={{42,-404},{62,-384}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor7
                                                                annotation (Placement(transformation(extent={{120,
            -442},{132,-430}})));
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
                    annotation (Placement(transformation(extent={{62,-446},{82,
            -426}})));
  Aircraft.Mechanical.Loads.Fan fan7
    annotation (Placement(transformation(extent={{148,-440},{156,-432}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=tauRef.y)
    annotation (Placement(transformation(extent={{46,-424},{66,-404}})));
  Modelica.Electrical.Analog.Basic.Ground ground8
    annotation (Placement(transformation(extent={{44,-466},{64,-446}})));
  Modelica.Electrical.Analog.Basic.Ground ground10
                                                 annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=0,
        origin={-49,-127})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-48,-88})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-12,-11},{12,11}},
        rotation=270,
        origin={-74,-45})));
equation
  connect(multiSensor.flange_b,fan. flange_a1)
    annotation (Line(points={{116,4},{149,4}},color={0,0,0}));
  connect(tauRef.y,electricDrive. desiredSpeed)
    annotation (Line(points={{13,34},{54,34},{54,16}}, color={0,0,127}));
  connect(multiSensor.flange_a,electricDrive. flange) annotation (Line(points={{104,4},
          {64,4}},                              color={0,0,0}));
  connect(multiSensor1.flange_a,electricDrive1. flange)
    annotation (Line(points={{118,-60},{80,-60}},  color={0,0,0}));
  connect(multiSensor1.flange_b,fan1. flange_a1)
    annotation (Line(points={{130,-60},{145,-60}}, color={0,0,0}));
  connect(electricDrive1.desiredSpeed,realExpression. y)
    annotation (Line(points={{70,-48},{70,-36},{61,-36}},   color={0,0,127}));
  connect(multiSensor2.flange_a,electricDrive2. flange)
    annotation (Line(points={{116,-116},{78,-116}},color={0,0,0}));
  connect(multiSensor2.flange_b,fan2. flange_a1)
    annotation (Line(points={{128,-116},{143,-116}},
                                                   color={0,0,0}));
  connect(electricDrive2.desiredSpeed,realExpression1. y)
    annotation (Line(points={{68,-104},{68,-94},{63,-94}},  color={0,0,127}));
  connect(electricDrive.pin_n,ground1. p)
    annotation (Line(points={{44,-2},{38,-2},{38,-10}},
                                                     color={0,0,255}));
  connect(electricDrive1.pin_n,ground2. p)
    annotation (Line(points={{60,-66},{50,-66},{50,-70}}, color={0,0,255}));
  connect(electricDrive2.pin_n,ground3. p)
    annotation (Line(points={{58,-122},{50,-122},{50,-126}}, color={0,0,255}));
  connect(multiSensor3.flange_a,electricDrive3. flange)
    annotation (Line(points={{118,-178},{80,-178}},color={0,0,0}));
  connect(multiSensor3.flange_b,fan3. flange_a1)
    annotation (Line(points={{130,-178},{145,-178}},
                                                   color={0,0,0}));
  connect(electricDrive3.desiredSpeed,realExpression2. y) annotation (Line(
        points={{70,-166},{70,-156},{65,-156}},    color={0,0,127}));
  connect(electricDrive3.pin_n,ground4. p)
    annotation (Line(points={{60,-184},{52,-184},{52,-188}}, color={0,0,255}));
  connect(multiSensor4.flange_a,electricDrive4. flange)
    annotation (Line(points={{118,-242},{80,-242}},color={0,0,0}));
  connect(multiSensor4.flange_b,fan4. flange_a1)
    annotation (Line(points={{130,-242},{145,-242}},
                                                   color={0,0,0}));
  connect(electricDrive4.desiredSpeed,realExpression3. y) annotation (Line(
        points={{70,-230},{70,-220},{65,-220}},    color={0,0,127}));
  connect(electricDrive4.pin_n,ground5. p)
    annotation (Line(points={{60,-248},{52,-248},{52,-252}}, color={0,0,255}));
  connect(multiSensor5.flange_a,electricDrive5. flange)
    annotation (Line(points={{118,-306},{80,-306}},color={0,0,0}));
  connect(multiSensor5.flange_b,fan5. flange_a1)
    annotation (Line(points={{130,-306},{145,-306}},
                                                   color={0,0,0}));
  connect(electricDrive5.desiredSpeed,realExpression4. y) annotation (Line(
        points={{70,-294},{70,-284},{65,-284}},    color={0,0,127}));
  connect(electricDrive5.pin_n,ground6. p)
    annotation (Line(points={{60,-312},{52,-312},{52,-316}}, color={0,0,255}));
  connect(multiSensor6.flange_a,electricDrive6. flange)
    annotation (Line(points={{118,-374},{80,-374}},color={0,0,0}));
  connect(multiSensor6.flange_b,fan6. flange_a1)
    annotation (Line(points={{130,-374},{145,-374}},
                                                   color={0,0,0}));
  connect(electricDrive6.desiredSpeed,realExpression5. y) annotation (Line(
        points={{70,-362},{70,-352},{65,-352}},    color={0,0,127}));
  connect(electricDrive6.pin_n,ground7. p)
    annotation (Line(points={{60,-380},{52,-380},{52,-384}}, color={0,0,255}));
  connect(electricDrive.pin_p,busExt. n[1]) annotation (Line(points={{44,10},{
          24,10},{24,-42.1},{-12,-42.1}},color={0,0,255}));
  connect(electricDrive1.pin_p,busExt. n[2]) annotation (Line(points={{60,-54},
          {34,-54},{34,-53.5},{-12,-53.5}},color={0,0,255}));
  connect(electricDrive2.pin_p,busExt. n[3]) annotation (Line(points={{58,-110},
          {36,-110},{36,-64.9},{-12,-64.9}},color={0,0,255}));
  connect(electricDrive3.pin_p,busExt. n[4]) annotation (Line(points={{60,-172},
          {32,-172},{32,-76.3},{-12,-76.3}},color={0,0,255}));
  connect(electricDrive4.pin_p,busExt. n[5]) annotation (Line(points={{60,-236},
          {26,-236},{26,-87.7},{-12,-87.7}},color={0,0,255}));
  connect(electricDrive5.pin_p,busExt. n[6]) annotation (Line(points={{60,-300},
          {18,-300},{18,-99.1},{-12,-99.1}},color={0,0,255}));
  connect(electricDrive6.pin_p,busExt. n[7]) annotation (Line(points={{60,-368},
          {10,-368},{10,-110.5},{-12,-110.5}},color={0,0,255}));
  connect(multiSensor7.flange_a,electricDrive7. flange)
    annotation (Line(points={{120,-436},{82,-436}},color={0,0,0}));
  connect(multiSensor7.flange_b,fan7. flange_a1)
    annotation (Line(points={{132,-436},{147,-436}},
                                                   color={0,0,0}));
  connect(electricDrive7.desiredSpeed,realExpression6. y) annotation (Line(
        points={{72,-424},{72,-414},{67,-414}},   color={0,0,127}));
  connect(electricDrive7.pin_n,ground8. p)
    annotation (Line(points={{62,-442},{54,-442},{54,-446}}, color={0,0,255}));
  connect(electricDrive7.pin_p,busExt. n[8]) annotation (Line(points={{62,-430},
          {2,-430},{2,-121.9},{-12,-121.9}},  color={0,0,255}));
  connect(ground10.p, constantVoltage.n) annotation (Line(points={{-49,-118},{
          -48,-118},{-48,-98}}, color={0,0,255}));
  connect(constantVoltage.p, busExt.p[1]) annotation (Line(points={{-48,-78},{
          -48,-64},{-16,-64},{-16,-82}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, constantVoltage.n) annotation (Line(points={{-65,
          -47.8235},{-62,-47.8235},{-62,-102},{-48,-102},{-48,-98}},
                                                               color={0,0,255}));
  connect(battery_FC_Charging.p1, constantVoltage.p)
    annotation (Line(points={{-65,-40.7647},{-48,-40.7647},{-48,-78}},
                                                             color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-480},{
            200,60}})),
    Icon(coordinateSystem(extent={{-200,-480},{200,60}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=100,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Battery_FuelCell_CommonBus;
