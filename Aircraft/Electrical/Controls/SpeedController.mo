within CHEETA.Aircraft.Electrical.Controls;
model SpeedController "Speed controller for Boeing 747"
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
  Modelica.Blocks.Math.Gain gain(k=I)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  parameter Real I "Integral gain" annotation (Dialog(group="PI Gains"));
  Modelica.Blocks.Math.Gain gain1(k=P)
    annotation (Placement(transformation(extent={{20,-50},{40,-30}})));
  parameter Real P "Proportional gain" annotation (Dialog(group="PI Gains"));
  Modelica.Blocks.Continuous.LimIntegrator limIntegrator(outMax=IMax, outMin=
        IMin) annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  parameter Real IMax "Upper limit of integral gain";
  parameter Real IMin=-limIntegrator.outMax "Lower limit of integral gain";
  Modelica.Blocks.Math.Feedback feedback1
    annotation (Placement(transformation(extent={{96,-10},{116,10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=TMax, uMin=TMin)
    annotation (Placement(transformation(extent={{128,-10},{148,10}})));
  parameter Real TMax "Upper limits of torque limiter"
    annotation (Dialog(group="Torque limiter"));
  parameter Real TMin=-limiter.uMax "Lower limits of torque limiter"
    annotation (Dialog(group="Torque limiter"));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=SpeedMax, uMin=SpeedMin)
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  parameter Real SpeedMax "Maximum reference speed";
  parameter Real SpeedMin=-limiter1.uMax "Minimum reference speed";
  Modelica.Blocks.Interfaces.RealInput N "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-142,0},{-102,40}})));
  Modelica.Blocks.Continuous.TransferFunction transferFunction(
    b={1},
    a=a,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{-70,-50},{-50,-30}})));
  parameter Real a[:]={1000,1}
    "Denominator coefficients of first order low pass transfer function";
  Modelica.Blocks.Interfaces.RealInput Np "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealOutput y1
               "Connector of Real output signal"
    annotation (Placement(transformation(extent={{160,-20},{180,0}})));
equation
  connect(feedback.y, gain.u)
    annotation (Line(points={{-15,0},{18,0}},   color={0,0,127}));
  connect(gain1.u, gain.u)
    annotation (Line(points={{18,-40},{12,-40},{12,0},{18,0}},
                                                             color={0,0,127}));
  connect(gain.y, limIntegrator.u)
    annotation (Line(points={{41,0},{58,0}},   color={0,0,127}));
  connect(limIntegrator.y, feedback1.u1)
    annotation (Line(points={{81,0},{98,0}},   color={0,0,127}));
  connect(gain1.y, feedback1.u2)
    annotation (Line(points={{41,-40},{106,-40},{106,-8}},
                                                       color={0,0,127}));
  connect(feedback1.y, limiter.u)
    annotation (Line(points={{115,0},{126,0}},   color={0,0,127}));
  connect(limiter1.y, feedback.u1)
    annotation (Line(points={{-49,0},{-32,0}}, color={0,0,127}));
  connect(limiter1.u, N) annotation (Line(points={{-72,0},{-96,0},{-96,20},{-122,
          20}}, color={0,0,127}));
  connect(transferFunction.y, feedback.u2)
    annotation (Line(points={{-49,-40},{-24,-40},{-24,-8}}, color={0,0,127}));
  connect(transferFunction.u, Np)
    annotation (Line(points={{-72,-40},{-120,-40}}, color={0,0,127}));
  connect(limiter.y, y1) annotation (Line(points={{149,0},{158,0},{158,-10},{170,
          -10}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},
            {160,40}})),                                         Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},{160,40}})));
end SpeedController;
