within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_Resistive_load "Resistive load connected to inverter"
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
      constantDutyCycle=0.5, f=1000)
    annotation (Placement(transformation(extent={{26,-42},{46,-22}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-16},{-4,4}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=
        1e-3) annotation (Placement(transformation(extent={{-90,-6},{-78,6}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1e-6)
    annotation (Placement(transformation(extent={{58,-10},{78,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{76,-52},{96,-32}})));
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
  connect(pwm1.fire, inverter.fire_p)
    annotation (Line(points={{30,-21},{30,-21},{30,-12}}, color={255,0,255}));
  connect(pwm1.notFire, inverter.fire_n)
    annotation (Line(points={{42,-21},{42,-12}}, color={255,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-77,4},
          {-68,4},{-68,6},{-60,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-77,
          -4},{-68,-4},{-68,-6},{-60,-6}}, color={0,0,255}));
  connect(inverter.ac, resistor.p)
    annotation (Line(points={{46,0},{58,0}}, color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{78,0},{86,0},{86,-32}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end SingleBranch_Resistive_load;
