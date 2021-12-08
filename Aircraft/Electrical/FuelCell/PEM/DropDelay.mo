within CHEETA.Aircraft.Electrical.FuelCell.PEM;
model DropDelay
  Modelica.Blocks.Continuous.TransferFunction transferFunction(b={1}, a={80,1})
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-60,-22},{-40,-2}})));
  Modelica.Blocks.Interfaces.RealInput I "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput Vdyn_d
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Math.Gain gain(k=0.16)
    annotation (Placement(transformation(extent={{10,-22},{30,-2}})));
  Modelica.Blocks.Nonlinear.DeadZone deadZone(uMax=0.05)
    annotation (Placement(transformation(extent={{48,-22},{68,-2}})));
equation
  connect(transferFunction.u, I) annotation (Line(points={{-62,20},{-90,20},{
          -90,0},{-120,0}}, color={0,0,127}));
  connect(feedback.u1, I) annotation (Line(points={{-58,-12},{-90,-12},{-90,0},
          {-120,0}}, color={0,0,127}));
  connect(feedback.u2, transferFunction.y) annotation (Line(points={{-50,-20},{
          -50,-30},{-30,-30},{-30,20},{-39,20}}, color={0,0,127}));
  connect(feedback.y, gain.u)
    annotation (Line(points={{-41,-12},{8,-12}}, color={0,0,127}));
  connect(gain.y, deadZone.u)
    annotation (Line(points={{31,-12},{46,-12}}, color={0,0,127}));
  connect(deadZone.y, Vdyn_d) annotation (Line(points={{69,-12},{94,-12},{94,0},
          {110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
end DropDelay;
