within CHEETA.Aircraft.Electrical.CB;
model Breaker_simple
  "Simple circuit breaker with time or signal control"
  parameter Boolean enableTrigger=false "=true, if external tigger signal is used"
    annotation (Evaluate=true, choices(checkBox=true));
  parameter Modelica.SIunits.Time t_o=Modelica.Constants.inf "Opening time"
    annotation (Dialog(enable=not enableTrigger));
  parameter Boolean rc_enabled=false "Enable reclosure" annotation (
    Evaluate=true,
    choices(checkBox=true),
    Dialog(enable=not enableTrigger));
  parameter Modelica.SIunits.Time t_rc=Modelica.Constants.inf "Reclosing time"
     annotation (Dialog(enable=not enableTrigger and rc_enabled));
  Modelica.Blocks.Interfaces.BooleanInput Trigger if enableTrigger annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,120})));

  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
protected 
  Modelica.Blocks.Interfaces.BooleanOutput Open "Help variable to indicate open circuit breaker" annotation (Placement(transformation(extent={{-4,-10},{16,10}})));

equation 
  if not enableTrigger then
    if not rc_enabled and time >= t_o then
      Open = true;
    elseif rc_enabled and time >= t_o and time < t_rc then
      Open = true;
    else
      Open = false;
    end if;
  end if;

  if Open then
    pin_p.i = 0;
    pin_n.i = 0;
  else
    pin_p.v = pin_n.v;
    pin_p.i = -pin_p.i;
  end if;

  connect(Trigger, Open) annotation (Line(points={{0,120},{0,60},{0,0},{6,0}}, color={255,0,255}));
  annotation (
    Icon(graphics={Rectangle(
          extent={{-40,40},{40,-40}},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None), Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-90,2},{-44,2}}, color={0,0,255}),
        Ellipse(extent={{-44,6},{-36,-2}}, lineColor={0,0,255}),
        Line(points={{-37,4},{40,42}}, color={0,0,255}),
        Line(points={{40,2},{90,2}}, color={0,0,255}),
        Line(points={{40,22},{40,2}}, color={0,0,255}),
        Text(
          extent={{-150,90},{150,50}},
          textString="%name",
          lineColor={0,0,255})}),
    Documentation(info="<html>
<p>This is an <b>opening</b> circuit breaker which can either be parametrised with an opening and closing time or controlled via an external trigger. If the external trigger is active (i.e.,  <span style=\"font-family: monospace;\">Trigger=true)</span> then the circuit breaker is open.</p>
</html>"));
end Breaker_simple;
