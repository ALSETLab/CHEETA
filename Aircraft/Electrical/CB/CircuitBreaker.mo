within CHEETA.Aircraft.Electrical.CB;
model CircuitBreaker "Ideal circuit breaker"
  SimpleCircuitBreaker switch
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  parameter Boolean useHeatPort=false "=true, if heatPort is enabled";
  parameter Modelica.SIunits.Conductance Goff=1e-5 "Opened switch conductance";
  parameter Modelica.SIunits.Resistance Ron=1e-5 "Closed switch resistance";
  CB_Trigger cB_Trigger(k=k)
    annotation (Placement(transformation(extent={{-16,24},{-4,32}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                           "positive pin" annotation (Placement(transformation(
          extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1
                "Negative electrical pin"
    annotation (Placement(transformation(extent={{88,-10},{108,10}})));
  parameter Real k "Reference overcurrent value";
equation
  connect(cB_Trigger.y1, switch.control)
    annotation (Line(points={{-3,28},{0,28},{0,12}}, color={255,0,255}));
  connect(cB_Trigger.n1, switch.p) annotation (Line(points={{-16,26},{-18,26},{
          -18,0},{-10,0}}, color={0,0,255}));
  connect(cB_Trigger.p1, p1) annotation (Line(points={{-16,30},{-40,30},{-40,0},
          {-100,0}}, color={0,0,255}));
  connect(switch.n, n1)
    annotation (Line(points={{10,0},{98,0}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-20},{100,60}})), Icon(
        coordinateSystem(extent={{-100,-20},{100,60}}), graphics={
        Ellipse(extent={{-44,4},{-36,-4}}, lineColor={0,0,255}),
        Line(points={{-90,0},{-44,0}}, color={0,0,255}),
        Line(points={{-37,2},{40,40}}, color={0,0,255}),
        Line(points={{40,0},{90,0}}, color={0,0,255}),
        Line(points={{40,20},{40,0}}, color={0,0,255})}));
end CircuitBreaker;
