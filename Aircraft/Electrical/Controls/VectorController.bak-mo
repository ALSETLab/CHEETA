within CHEETA.Aircraft.Electrical.Controls;
model VectorController
  Modelica.Blocks.Interfaces.RealInput Te
    "Electrical torque from speed controller"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput theta_r "rotor angle"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Functions.TorqueToCurrent torqueToCurrent(k=kT)
    annotation (Placement(transformation(extent={{-54,50},{-34,70}})));
  parameter Real kT "Torque constant (from motor)"
    annotation (Dialog(group="From machine"));
  Functions.RotorToElectricalAngle rotorToElectricalAngle
    annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  Functions.InversePark inversePark
    annotation (Placement(transformation(extent={{-10,70},{10,50}})));
  Modelica.Blocks.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{-50,76},{-36,90}})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug Iabc(m=3)
    annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
equation
  connect(Te, torqueToCurrent.T)
    annotation (Line(points={{-120,60},{-56,60}}, color={0,0,127}));
  connect(theta_r, rotorToElectricalAngle.theta_r)
    annotation (Line(points={{-120,0},{-56,0}}, color={0,0,127}));
  connect(rotorToElectricalAngle.theta_e, inversePark.theta)
    annotation (Line(points={{-33,0},{0,0},{0,49}}, color={0,0,127}));
  connect(torqueToCurrent.I, inversePark.iq)
    annotation (Line(points={{-33,60},{-11,60}}, color={0,0,127}));
  connect(const.y, inversePark.i0) annotation (Line(points={{-35.3,83},{-26,83},
          {-26,66},{-11,66}}, color={0,0,127}));
  connect(inversePark.id, inversePark.i0) annotation (Line(points={{-11,54},{
          -26,54},{-26,66},{-11,66}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end VectorController;
