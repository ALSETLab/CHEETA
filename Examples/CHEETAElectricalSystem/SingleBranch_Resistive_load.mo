within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_Resistive_load "Resistive load connected to inverter"
  Modelica.Electrical.Analog.Basic.Inductor inductor(i(fixed=true, start=5000),
      L=1e-6)
    annotation (Placement(transformation(extent={{-24,-4},{-4,16}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(i(fixed=true, start=5000),
      L=1e-6)
    annotation (Placement(transformation(extent={{-24,-16},{-4,4}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0, L=
        1e-3) annotation (Placement(transformation(extent={{-90,-6},{-78,6}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{76,-52},{96,-32}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.1)
    annotation (Placement(transformation(extent={{56,-10},{76,10}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
    VkneeTransistor=1,
    VkneeDiode=0.7,
    constantEnable=false)
    annotation (Placement(transformation(extent={{26,-10},{46,10}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
    useConstantDutyCycle=true,
    constantDutyCycle=1,
    f(displayUnit="kHz") = 1000)
    annotation (Placement(transformation(extent={{26,-42},{46,-22}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 1000)
    annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
equation
  connect(resistor.n,ground. p)
    annotation (Line(points={{76,0},{86,0},{86,-32}}, color={0,0,255}));
  connect(pwm1.fire,inverter. fire_p)
    annotation (Line(points={{30,-21},{30,-12}},          color={255,0,255}));
  connect(pwm1.notFire,inverter. fire_n)
    annotation (Line(points={{42,-21},{42,-12}}, color={255,0,255}));
  connect(inverter.ac, resistor.p)
    annotation (Line(points={{46,0},{56,0}}, color={0,0,255}));
  connect(inverter.dc_p, inductor.n)
    annotation (Line(points={{26,6},{12,6},{12,6},{-4,6}}, color={0,0,255}));
  connect(inductor1.n, inverter.dc_n)
    annotation (Line(points={{-4,-6},{26,-6}}, color={0,0,255}));
  connect(dcdc.dc_p1, simplifiedFuelCell.pin_p) annotation (Line(points={{-60,6},
          {-70,6},{-70,4},{-77,4}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-77,
          -4},{-68,-4},{-68,-6},{-60,-6}}, color={0,0,255}));
  connect(dcdc.fire_p, pwm.fire)
    annotation (Line(points={{-56,-12},{-56,-19}}, color={255,0,255}));
  connect(dcdc.dc_p2, inductor.p)
    annotation (Line(points={{-40,6},{-24,6}}, color={0,0,255}));
  connect(inductor1.p, dcdc.dc_n2)
    annotation (Line(points={{-24,-6},{-40,-6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end SingleBranch_Resistive_load;
