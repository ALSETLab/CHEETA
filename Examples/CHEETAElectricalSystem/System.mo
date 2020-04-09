within CHEETA.Examples.CHEETAElectricalSystem;
model System "CHEETA Electrical System"
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc(
    L=0.001,
    i=0,
    C=0.1,
    v=1000)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,50})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-60,8},{-40,28}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
      constantEnable=false)
    annotation (Placement(transformation(extent={{6,40},{26,60}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
    useConstantDutyCycle=false,
    constantDutyCycle=0.5,
    f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{26,8},{6,28}})));
  Electrification.Electrical.DCSplitter  splitterHVDC
    annotation (Placement(transformation(extent={{-78,46},{-70,54}})));
              Electrification.Batteries.Examples.BatteryPack
    batteryPack(internal_ground=true, enable_thermal_port=false)
    annotation (Placement(transformation(extent={{-78,40},{-98,60}})));
  Aircraft.Electrical.Machines.Motors.SimpleMotor simpleMotor1
    annotation (Placement(transformation(extent={{40,40},{60,60}})));
  Aircraft.Electrical.Controls.SpeedDriveController variableSpeedDrive(wref=
        4000, T=10)
    annotation (Placement(transformation(extent={{60,8},{40,28}})));
  Aircraft.Mechanical.Loads.Fan      fan
    annotation (Placement(transformation(extent={{80,40},{100,60}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=0)
    annotation (Placement(transformation(extent={{-86,-36},{-74,-24}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc1(
    L=0.0001,
    i=0,
    C=0.1,
    v=1000)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-48,-30})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm2(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-58,-72},{-38,-52}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter1(
      constantEnable=false)
    annotation (Placement(transformation(extent={{8,-40},{28,-20}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm3(
    useConstantDutyCycle=false,
    constantDutyCycle=0.5,
    f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{28,-72},{8,-52}})));
  Aircraft.Electrical.Machines.Motors.SimpleMotor simpleMotor2
    annotation (Placement(transformation(extent={{42,-40},{62,-20}})));
  Aircraft.Electrical.Controls.SpeedDriveController variableSpeedDrive1(wref=
        4000, T=10)
    annotation (Placement(transformation(extent={{62,-72},{42,-52}})));
  Aircraft.Mechanical.Loads.Fan      fan1
    annotation (Placement(transformation(extent={{82,-40},{102,-20}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly(l=1)
    annotation (Placement(transformation(extent={{-26,52},{-10,60}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly1(l=1)
    annotation (Placement(transformation(extent={{-26,40},{-10,48}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-122,-14},{-102,6}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{-68,-14},{-88,6}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly2(l=1)
    annotation (Placement(transformation(extent={{-22,-20},{-6,-28}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly3(l=1)
    annotation (Placement(transformation(extent={{-22,-32},{-6,-40}})));
equation
  connect(dcdc.fire_p,pwm. fire)
    annotation (Line(points={{-56,38},{-56,29}},   color={255,0,255}));
  connect(splitterHVDC.p,dcdc. dc_p1) annotation (Line(points={{-70,54},{-64,54},
          {-64,56},{-60,56}}, color={0,0,255}));
  connect(dcdc.dc_n1,splitterHVDC. n) annotation (Line(points={{-60,44},{-66,44},
          {-66,46},{-70,46}}, color={0,0,255}));
  connect(variableSpeedDrive.flange1,simpleMotor1. flange1) annotation (Line(
        points={{60.2,18},{74,18},{74,50},{60.4,50}}, color={0,0,0}));
  connect(simpleMotor1.p1,inverter. ac)
    annotation (Line(points={{39.6,50},{26,50}}, color={0,0,255}));
  connect(inverter.fire_p,pwm1. notFire)
    annotation (Line(points={{10,38},{10,29}}, color={255,0,255}));
  connect(inverter.fire_n,pwm1. fire)
    annotation (Line(points={{22,38},{22,29}}, color={255,0,255}));
  connect(variableSpeedDrive.y1,pwm1. dutyCycle)
    annotation (Line(points={{39,18},{28,18}}, color={0,0,127}));
  connect(fan.flange_a1, simpleMotor1.flange1)
    annotation (Line(points={{77.5,50},{60.4,50}}, color={0,0,0}));
  connect(variableSpeedDrive1.flange1, simpleMotor2.flange1) annotation (Line(
        points={{62.2,-62},{76,-62},{76,-30},{62.4,-30}}, color={0,0,0}));
  connect(simpleMotor2.p1, inverter1.ac)
    annotation (Line(points={{41.6,-30},{28,-30}}, color={0,0,255}));
  connect(inverter1.fire_p, pwm3.notFire)
    annotation (Line(points={{12,-42},{12,-51}}, color={255,0,255}));
  connect(inverter1.fire_n, pwm3.fire)
    annotation (Line(points={{24,-42},{24,-51}}, color={255,0,255}));
  connect(variableSpeedDrive1.y1, pwm3.dutyCycle)
    annotation (Line(points={{41,-62},{30,-62}}, color={0,0,127}));
  connect(fan1.flange_a1, simpleMotor2.flange1)
    annotation (Line(points={{79.5,-30},{62.4,-30}}, color={0,0,0}));
  connect(simplifiedFuelCell.pin_p, dcdc1.dc_p1)
    annotation (Line(points={{-73,-26},{-58,-26},{-58,-24}}, color={0,0,255}));
  connect(dcdc1.dc_n1, simplifiedFuelCell.pin_p1) annotation (Line(points={{-58,
          -36},{-66,-36},{-66,-34},{-73,-34}}, color={0,0,255}));
  connect(pwm2.fire, dcdc1.fire_p) annotation (Line(points={{-54,-51},{-54,-51},
          {-54,-42}}, color={255,0,255}));
  connect(inverter.dc_p, stekly.pin_n)
    annotation (Line(points={{6,56},{-9,56}}, color={0,0,255}));
  connect(stekly.pin_p, dcdc.dc_p2)
    annotation (Line(points={{-27,56},{-40,56}}, color={0,0,255}));
  connect(dcdc.dc_n2, stekly1.pin_p)
    annotation (Line(points={{-40,44},{-27,44}}, color={0,0,255}));
  connect(stekly1.pin_n, inverter.dc_n)
    annotation (Line(points={{-9,44},{6,44}}, color={0,0,255}));
  connect(stekly.port_a, thermalConductor.port_a)
    annotation (Line(points={{-18,52},{-18,-4},{-68,-4}}, color={191,0,0}));
  connect(stekly1.port_a, thermalConductor.port_a)
    annotation (Line(points={{-18,40},{-18,-4},{-68,-4}}, color={191,0,0}));
  connect(thermalConductor.port_b, fixedTemperature.port)
    annotation (Line(points={{-88,-4},{-102,-4}}, color={191,0,0}));
  connect(inverter1.dc_p, stekly2.pin_n)
    annotation (Line(points={{8,-24},{-5,-24}}, color={0,0,255}));
  connect(stekly2.pin_p, dcdc1.dc_p2)
    annotation (Line(points={{-23,-24},{-38,-24}}, color={0,0,255}));
  connect(stekly2.port_a, thermalConductor.port_a)
    annotation (Line(points={{-14,-20},{-14,-4},{-68,-4}}, color={191,0,0}));
  connect(dcdc1.dc_n2, stekly3.pin_p)
    annotation (Line(points={{-38,-36},{-23,-36}}, color={0,0,255}));
  connect(inverter1.dc_n, stekly3.pin_n)
    annotation (Line(points={{8,-36},{-5,-36}}, color={0,0,255}));
  connect(stekly3.port_a, stekly2.port_a)
    annotation (Line(points={{-14,-32},{-14,-20}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end System;
