within CHEETA.Aircraft.Mechanical.Pumps;
model Boeing747_Pump_Load "Pump load for 747 Boeing electrical system model"
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-40,0})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={8,-20})));
  Modelica.Blocks.Math.Gain gain(k=k)      annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=180,
        origin={-41,-37})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
      Placement(transformation(rotation=0, extent={{120,-30},{140,-10}}),
        iconTransformation(extent={{120,-30},{140,-10}})));
  parameter Real k=28/500 "Torque gain";
equation
  connect(speedSensor.w, gain.u) annotation (Line(points={{-3,-20},{-11,-20},{
          -11,-37},{-30.2,-37}},
                               color={0,0,127}));
  connect(gain.y, torque.tau) annotation (Line(points={{-50.9,-37},{-76,-37},{
          -76,0},{-52,0}}, color={0,0,127}));
  connect(flange, speedSensor.flange)
    annotation (Line(points={{130,-20},{18,-20}},        color={0,0,0}));
  connect(flange, torque.flange) annotation (Line(points={{130,-20},{46,-20},{
          46,0},{-30,0}},  color={0,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-80,-60},{120,20}})),  Icon(
        coordinateSystem(extent={{-80,-60},{120,20}}), graphics={Rectangle(
            extent={{-80,20},{120,-60}}, lineColor={28,108,200}), Text(
          extent={{-42,-6},{80,-32}},
          lineColor={28,108,200},
          textString="747 Pump Load")}),
    Documentation(info="<html>
<p>Pump model used in Boeing 747 electrical model.</p>
<p>This model is consists of a mechanical multibody reference point driving a torque amplified by a specified gain.</p>
</html>"));
end Boeing747_Pump_Load;
