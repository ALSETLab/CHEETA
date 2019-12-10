within CHEETA.Aircraft.Electrical.CB;
model SimpleCircuitBreaker "Ideal circuit breaker"
  extends Modelica.Electrical.Analog.Interfaces.IdealSwitch;
  Modelica.Blocks.Interfaces.BooleanInput control
    "true => switch open, false => p--n connected" annotation (Placement(
        transformation(
        origin={0,120},
        extent={{-20,-20},{20,20}},
        rotation=270), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,120})));
equation
  off = control;
  annotation (defaultComponentName="switch",
    Documentation(info="<html>
<p>
The switching behaviour of the ideal opening switch is controlled by the input signal control: off = control.<br>
For further details, see partial model <a href=\"modelica://Modelica.Electrical.Analog.Interfaces.IdealSwitch\">IdealSwitch</a>.
</p>
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
end SimpleCircuitBreaker;
