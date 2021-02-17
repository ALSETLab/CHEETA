within CHEETA.Aircraft.Electrical.CB;
model CircuitBreakerArc "Circuit breaker with arc during opening"
  extends Modelica.Electrical.Analog.Interfaces.IdealSwitch;
  parameter Modelica.Units.SI.Voltage ArcVoltageLevel=0.5
    "Switch voltage level";


  Modelica.Electrical.Analog.Interfaces.PositivePin control
    "Voltage control for circuit breaker"
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
equation
  off = control.v > ArcVoltageLevel;
  control.i = 0;
  annotation (defaultComponentName="switch",
    Documentation(info="<html>
<p>The switching behavior of the circuit breaker is controlled by the control input voltage. Once the control voltage surpasses the voltage defined by the user, the switch will open.</p><p>For further details, see partial model <a href=\"modelica://Modelica.Electrical.Analog.Interfaces.IdealSwitch\">IdealSwitch</a>. </p>
</html>",
        revisions="<html>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}), graphics={
        Line(points={{40,20},{40,0}}, color={0,0,255}),
        Line(
          visible=useHeatPort,
          points={{0,-100},{0,25}},
          color={127,0,0},
          pattern=LinePattern.Dot)}));
end CircuitBreakerArc;
