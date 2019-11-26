within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_SimpleMachine
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=
        1e3,
    V=500)   annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-86,0})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
      constantEnable=false)
    annotation (Placement(transformation(extent={{26,-10},{46,10}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-4},{-4,16}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f=10)
    annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
      constantDutyCycle=0.5, f=10)
    annotation (Placement(transformation(extent={{26,-42},{46,-22}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-16},{-4,4}})));
  Aircraft.Electrical.Machines.SimpleMotor simpleMotor
    annotation (Placement(transformation(extent={{58,-10},{78,10}})));
  Aircraft.Mechanical.Loads.Fan fan(J=0.5)
    annotation (Placement(transformation(extent={{88,-4},{96,4}})));
equation
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-79,4},
          {-72,4},{-72,6},{-60,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.n1, dcdc.dc_n1) annotation (Line(points={{-79,-4},
          {-70,-4},{-70,-6},{-60,-6}}, color={0,0,255}));
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
    annotation (Line(points={{46,0},{57.6,0}}, color={0,0,255}));
  connect(pwm1.fire, inverter.fire_p)
    annotation (Line(points={{30,-21},{30,-21},{30,-12}}, color={255,0,255}));
  connect(pwm1.notFire, inverter.fire_n)
    annotation (Line(points={{42,-21},{42,-12}}, color={255,0,255}));
  connect(simpleMotor.flange1, fan.flange_a1)
    annotation (Line(points={{78.4,0},{87,0}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end SingleBranch_SimpleMachine;
