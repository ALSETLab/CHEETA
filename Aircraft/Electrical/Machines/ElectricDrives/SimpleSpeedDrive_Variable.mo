within CHEETA.Aircraft.Electrical.Machines.ElectricDrives;
model SimpleSpeedDrive_Variable
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
      constantEnable=false)
    annotation (Placement(transformation(extent={{-26,6},{-6,26}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
    useConstantDutyCycle=false,
    constantDutyCycle=0.5,
    f(displayUnit="kHz") = f)
    annotation (Placement(transformation(extent={{-6,-40},{-26,-20}})));
  Motors.SimpleMotor                              simpleMotor1(
    R=R,
    R_trs=R_trs,
    X_s=X_s,
    R_hyst=R_hyst,
    k=EMF_k)
    annotation (Placement(transformation(extent={{8,6},{28,26}})));
  CHEETA.Aircraft.Controls.SpeedDrives.VariableSpeedDriveController
                                        variableSpeedDrive(T=T, k=k)
    annotation (Placement(transformation(extent={{28,-20},{8,-40}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange1
                                                           "Flange"
    annotation (Placement(transformation(extent={{80,-6},{100,14}}),
        iconTransformation(extent={{80,-6},{100,14}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin dc_p1
    "Positive DC input"
    annotation (Placement(transformation(extent={{-94,32},{-74,52}}),
        iconTransformation(extent={{-94,32},{-74,52}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin dc_n1
    "Negative DC input"
    annotation (Placement(transformation(extent={{-94,-50},{-74,-30}}),
        iconTransformation(extent={{-94,-50},{-74,-30}})));

  parameter Modelica.SIunits.Time T=10
    "Time Constant (T>0 required) for speed drive PID" annotation (Dialog(group="Speed Drive Controller"));
  parameter Real k=1 "Gain for speed drive PID" annotation (Dialog(group="Speed Drive Controller"));
  parameter Modelica.SIunits.Frequency f(displayUnit="kHz") = 100000
    "Switching frequency for inverter" annotation (Dialog(group="Inverter"));
  parameter Modelica.SIunits.Resistance R=149
    "Effective resistance for hysteresis" annotation (Dialog(group="Motor"));
  parameter Modelica.SIunits.Resistance R_trs=1e-6
    "Effective resistance for transport  ac loss"
    annotation (Dialog(group="Motor"));
  parameter Modelica.SIunits.Inductance X_s=0.041 "Reactance"
    annotation (Dialog(group="Motor"));
  parameter Modelica.SIunits.Resistance R_hyst=149
    "Effective resistance for hysteresis ac loss"
    annotation (Dialog(group="Motor"));
  parameter Modelica.SIunits.ElectricalTorqueConstant EMF_k=0.021
    "Transformation coefficient of back EMF" annotation (Dialog(group="Motor"));
  Modelica.Blocks.Interfaces.RealInput wref annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,94})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=wref)
    annotation (Placement(transformation(extent={{70,-46},{50,-26}})));
equation
  connect(simpleMotor1.p1,inverter. ac)
    annotation (Line(points={{7.6,16},{-6,16}},  color={0,0,255}));
  connect(variableSpeedDrive.y1,pwm1. dutyCycle)
    annotation (Line(points={{7,-30},{-4,-30}},color={0,0,127}));
  connect(inverter.fire_p, pwm1.notFire)
    annotation (Line(points={{-22,4},{-22,-19}}, color={255,0,255}));
  connect(inverter.fire_n, pwm1.fire)
    annotation (Line(points={{-10,4},{-10,-19}}, color={255,0,255}));
  connect(simpleMotor1.flange1, variableSpeedDrive.flange1) annotation (Line(
        points={{28.4,16},{40,16},{40,-30},{28.2,-30}}, color={0,0,0}));
  connect(simpleMotor1.flange1, flange1) annotation (Line(points={{28.4,16},{64,
          16},{64,4},{90,4}},  color={0,0,0}));
  connect(inverter.dc_p, dc_p1) annotation (Line(points={{-26,22},{-60,22},{-60,
          42},{-84,42}},  color={0,0,255}));
  connect(inverter.dc_n, dc_n1) annotation (Line(points={{-26,10},{-40,10},{-40,
          -40},{-84,-40}},  color={0,0,255}));
  connect(realExpression.y, variableSpeedDrive.wref)
    annotation (Line(points={{49,-36},{30,-36}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}), graphics={
        Rectangle(
          extent={{-54,64},{66,-56}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,128,255}),
        Rectangle(
          extent={{-54,64},{-74,-56}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={128,128,128}),
        Rectangle(
          extent={{66,14},{86,-6}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Rectangle(
          extent={{-54,74},{26,54}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-64,-86},{-54,-86},{-24,-16},{26,-16},{56,-86},{66,-86},{66,-96},
              {-64,-96},{-64,-86}},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
end SimpleSpeedDrive_Variable;
