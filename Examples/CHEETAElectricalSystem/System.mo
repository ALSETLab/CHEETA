within CHEETA.Examples.CHEETAElectricalSystem;
model System "Resistive load connected to inverter"
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,50})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
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
  Aircraft.Electrical.Machines.SimpleMotor simpleMotor1
    annotation (Placement(transformation(extent={{40,40},{60,60}})));
  Aircraft.Electrical.Controls.VariableSpeedDrive variableSpeedDrive(wref=4000,
      T=10) annotation (Placement(transformation(extent={{60,8},{40,28}})));
  Aircraft.Mechanical.Loads.Pinwheel pinwheel
    annotation (Placement(transformation(extent={{80,40},{100,60}})));
equation
  connect(dcdc.fire_p,pwm. fire)
    annotation (Line(points={{-56,38},{-56,31}},   color={255,0,255}));
  connect(inverter.dc_p, dcdc.dc_p2)
    annotation (Line(points={{6,56},{-40,56}}, color={0,0,255}));
  connect(dcdc.dc_n2, inverter.dc_n)
    annotation (Line(points={{-40,44},{6,44}}, color={0,0,255}));
  connect(splitterHVDC.p, dcdc.dc_p1) annotation (Line(points={{-70,54},{-64,54},
          {-64,56},{-60,56}}, color={0,0,255}));
  connect(dcdc.dc_n1, splitterHVDC.n) annotation (Line(points={{-60,44},{-66,44},
          {-66,46},{-70,46}}, color={0,0,255}));
  connect(variableSpeedDrive.flange1,simpleMotor1. flange1) annotation (Line(
        points={{60.2,18},{74,18},{74,50},{60.4,50}}, color={0,0,0}));
  connect(simpleMotor1.p1, inverter.ac)
    annotation (Line(points={{39.6,50},{26,50}}, color={0,0,255}));
  connect(inverter.fire_p, pwm1.notFire)
    annotation (Line(points={{10,38},{10,29}}, color={255,0,255}));
  connect(inverter.fire_n, pwm1.fire)
    annotation (Line(points={{22,38},{22,29}}, color={255,0,255}));
  connect(variableSpeedDrive.y1, pwm1.dutyCycle)
    annotation (Line(points={{39,18},{28,18}}, color={0,0,127}));
  connect(pinwheel.flange_a1, simpleMotor1.flange1)
    annotation (Line(points={{80,50},{60.4,50}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end System;
