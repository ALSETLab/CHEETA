within CHEETA.Aircraft.Mechanical.Pumps;
model Pump_Load
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-40,0})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-18})));
  Modelica.Blocks.Math.Gain gain(k=k)      annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=180,
        origin={-41,-37})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
      Placement(transformation(rotation=0, extent={{110,-10},{130,10}})));
  parameter Real k=28/500 "Torque gain";
equation
  connect(speedSensor.w, gain.u) annotation (Line(points={{-1.9984e-15,-29},{0,
          -29},{0,-37},{-30.2,-37}},
                               color={0,0,127}));
  connect(gain.y, torque.tau) annotation (Line(points={{-50.9,-37},{-76,-37},{
          -76,0},{-52,0}}, color={0,0,127}));
  connect(flange, speedSensor.flange)
    annotation (Line(points={{120,0},{1.77636e-15,0},{1.77636e-15,-8}},
                                                         color={0,0,0}));
  connect(flange, torque.flange) annotation (Line(points={{120,0},{-30,0}},
                           color={0,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-80,-60},{120,20}})),  Icon(
        coordinateSystem(extent={{-80,-60},{120,20}})),
    Documentation(info="<html>
<p>Pump model used in Boeing 747 electrical model.</p>
<p>This model is consists of a mechanical multibody reference point driving a torque amplified by a specified gain.</p>
</html>"));
end Pump_Load;
