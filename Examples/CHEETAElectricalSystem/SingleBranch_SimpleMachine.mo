within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_SimpleMachine
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc(VkneeTransistor=1,
      VkneeDiode=0.7)                                         annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
    VkneeTransistor=1,
    VkneeDiode=0.7,
      constantEnable=false)
    annotation (Placement(transformation(extent={{26,-10},{46,10}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-4},{-4,16}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{26,-42},{46,-22}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-16},{-4,4}})));
  Aircraft.Electrical.Machines.SimpleMotor simpleMotor
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=
        0.001) annotation (Placement(transformation(extent={{-86,-6},{-74,6}})));
equation
  connect(dcdc.dc_p2, inductor.p)
    annotation (Line(points={{-40,6},{-24,6}}, color={0,0,255}));
  connect(inductor.n, inverter.dc_p)
    annotation (Line(points={{-4,6},{26,6}}, color={0,0,255}));
  connect(dcdc.fire_p, pwm.fire)
    annotation (Line(points={{-56,-12},{-56,-19}}, color={255,0,255}));
  connect(inductor1.p, dcdc.dc_n2)
    annotation (Line(points={{-24,-6},{-40,-6}}, color={0,0,255}));
  connect(inductor1.n, inverter.dc_n)
    annotation (Line(points={{-4,-6},{26,-6}}, color={0,0,255}));
  connect(inverter.ac, simpleMotor.p1)
    annotation (Line(points={{46,0},{59.6,0}}, color={0,0,255}));
  connect(pwm1.fire, inverter.fire_p)
    annotation (Line(points={{30,-21},{30,-12}},          color={255,0,255}));
  connect(pwm1.notFire, inverter.fire_n)
    annotation (Line(points={{42,-21},{42,-12}}, color={255,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-73,4},
          {-66,4},{-66,6},{-60,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-73,
          -4},{-68,-4},{-68,-6},{-60,-6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=0.5));
end SingleBranch_SimpleMachine;
