within CHEETA.Examples.CHEETAElectricalSystem;
model HTS_test "Test for HTS transmission"
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=0)
    annotation (Placement(transformation(extent={{-92,14},{-80,26}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc1
                                                              annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-54,20})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm2(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-64,-22},{-44,-2}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter1(
      constantEnable=false)
    annotation (Placement(transformation(extent={{2,10},{22,30}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm3(
    useConstantDutyCycle=false,
    constantDutyCycle=0.5,
    f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{22,-22},{2,-2}})));
  Aircraft.Electrical.Machines.SimpleMotor simpleMotor2
    annotation (Placement(transformation(extent={{36,10},{56,30}})));
  Aircraft.Electrical.Controls.VariableSpeedDrive variableSpeedDrive1(wref=4000,
      T=10) annotation (Placement(transformation(extent={{56,-22},{36,-2}})));
  Aircraft.Electrical.HTS.HTS_exploss hTS_exploss(temperature=115, l=1)
    annotation (Placement(transformation(extent={{-30,22},{-14,30}})));
  Aircraft.Electrical.HTS.HTS_exploss hTS_exploss1(temperature=115, l=1)
    annotation (Placement(transformation(extent={{-30,10},{-14,18}})));
equation
  connect(variableSpeedDrive1.flange1, simpleMotor2.flange1) annotation (Line(
        points={{56.2,-12},{70,-12},{70,20},{56.4,20}},   color={0,0,0}));
  connect(simpleMotor2.p1, inverter1.ac)
    annotation (Line(points={{35.6,20},{22,20}},   color={0,0,255}));
  connect(inverter1.fire_p, pwm3.notFire)
    annotation (Line(points={{6,8},{6,-1}},      color={255,0,255}));
  connect(inverter1.fire_n, pwm3.fire)
    annotation (Line(points={{18,8},{18,-1}},    color={255,0,255}));
  connect(variableSpeedDrive1.y1, pwm3.dutyCycle)
    annotation (Line(points={{35,-12},{24,-12}}, color={0,0,127}));
  connect(simplifiedFuelCell.pin_p, dcdc1.dc_p1)
    annotation (Line(points={{-79,24},{-64,24},{-64,26}},    color={0,0,255}));
  connect(dcdc1.dc_n1, simplifiedFuelCell.pin_p1) annotation (Line(points={{-64,14},
          {-72,14},{-72,16},{-79,16}},         color={0,0,255}));
  connect(pwm2.fire, dcdc1.fire_p) annotation (Line(points={{-60,-1},{-60,8}},
                      color={255,0,255}));
  connect(dcdc1.dc_p2, hTS_exploss.pin_p)
    annotation (Line(points={{-44,26},{-31,26}}, color={0,0,255}));
  connect(inverter1.dc_p, hTS_exploss.pin_n)
    annotation (Line(points={{2,26},{-13,26}}, color={0,0,255}));
  connect(dcdc1.dc_n2, hTS_exploss1.pin_p)
    annotation (Line(points={{-44,14},{-31,14}}, color={0,0,255}));
  connect(inverter1.dc_n, hTS_exploss1.pin_n)
    annotation (Line(points={{2,14},{-13,14}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end HTS_test;
