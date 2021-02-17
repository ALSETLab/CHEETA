within CHEETA.Aircraft.Controls.SpeedDrives;
model SpeedDriveController
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
  Modelica.Blocks.Interfaces.RealOutput y1 "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Sources.Constant const(k=wref)
    annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
  parameter Real wref=100 "Constant output value";
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-10,10},{10,-10}})));
  parameter Modelica.Units.SI.Time T "Time Constant (T>0 required)";
  parameter Real k=1 "Gain";
equation
  connect(speedSensor.flange, flange1) annotation (Line(points={{-62,1.77636e-15},
          {-80,1.77636e-15},{-80,0},{-102,0}}, color={0,0,0}));
  connect(currentController.y, y1)
    annotation (Line(points={{57,0},{110,0}}, color={0,0,127}));
  connect(currentController.u, feedback.y)
    annotation (Line(points={{34,0},{9,0}}, color={0,0,127}));
  connect(feedback.u1, speedSensor.w) annotation (Line(points={{-8,0},{-26,0},{-26,
          -8.88178e-16},{-41,-8.88178e-16}}, color={0,0,127}));
  connect(const.y, feedback.u2)
    annotation (Line(points={{-39,30},{0,30},{0,8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
end SpeedDriveController;
