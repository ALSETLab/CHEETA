within CHEETA.Aircraft.Controls.SpeedDrives;
model VariableSpeedDrive_ThreePhase
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-52,0})));
  Modelica.Blocks.Continuous.PI
            currentController(
    k=k,
    T=T,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{36,-10},{56,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange1
    "Flange of shaft from which sensor information shall be measured"
    annotation (Placement(transformation(extent={{-112,-10},{-92,10}})));
  Modelica.Blocks.Interfaces.RealOutput y1[3] "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-10,10},{10,-10}})));
  parameter Modelica.Units.SI.Time T "Time Constant (T>0 required)";
  parameter Real k=1 "Gain";
  Blocks.Routing.RealExtend realExtend
    annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  Modelica.Blocks.Interfaces.RealInput wref
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
equation
  connect(speedSensor.flange, flange1) annotation (Line(points={{-62,1.77636e-15},
          {-80,1.77636e-15},{-80,0},{-102,0}}, color={0,0,0}));
  connect(currentController.u, feedback.y)
    annotation (Line(points={{34,0},{9,0}}, color={0,0,127}));
  connect(feedback.u1, speedSensor.w) annotation (Line(points={{-8,0},{-26,0},{-26,
          -8.88178e-16},{-41,-8.88178e-16}}, color={0,0,127}));
  connect(currentController.y, realExtend.u)
    annotation (Line(points={{57,0},{66,0}}, color={0,0,127}));
  connect(y1, realExtend.y)
    annotation (Line(points={{110,0},{89,0}}, color={0,0,127}));
  connect(feedback.u2, wref)
    annotation (Line(points={{0,8},{0,60},{-120,60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
end VariableSpeedDrive_ThreePhase;
