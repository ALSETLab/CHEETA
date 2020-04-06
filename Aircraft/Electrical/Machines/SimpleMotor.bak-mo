within CHEETA.Aircraft.Electrical.Machines;
model SimpleMotor "Simplified AC motor model"


  Modelica.Electrical.Analog.Basic.Inductor inductor(L=X_s)
                                                           "Reactance"
    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
  parameter Modelica.SIunits.Resistance R=149
    "Effective resistance for hysteresis";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_trs)
    "Effective resistance for transport  ac loss"
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  parameter Modelica.SIunits.Resistance R_trs=1e-6
    "Effective resistance for transport  ac loss";
  parameter Modelica.SIunits.Inductance X_s=0.041 "Reactance";
  Modelica.Electrical.Analog.Basic.Resistor hysteresis(R=R_hyst)
    "Effective resistance for hysteresis ac loss" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,0})));
  parameter Modelica.SIunits.Resistance R_hyst=149
    "Effective resistance for hysteresis ac loss";
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
  Modelica.Electrical.Analog.Basic.EMF emf(k=k) "back EMF"
    annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  parameter Modelica.SIunits.ElectricalTorqueConstant k=0.021
    "Transformation coefficient of back EMF";
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange1
                                                           "Flange"
    annotation (Placement(transformation(extent={{94,-10},{114,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{46,-42},{66,-22}})));
equation
  connect(inductor.p, resistor.n)
    annotation (Line(points={{-12,0},{-40,0}},   color={0,0,255}));
  connect(inductor.n, hysteresis.p)
    annotation (Line(points={{8,0},{18,0},{18,18},{40,18},{40,10}},
                                                       color={0,0,255}));
  connect(resistor.p, p1)
    annotation (Line(points={{-60,0},{-104,0}},   color={0,0,255}));
  connect(emf.p, hysteresis.p) annotation (Line(points={{72,10},{72,18},{40,18},
          {40,10}}, color={0,0,255}));
  connect(emf.flange, flange1)
    annotation (Line(points={{82,0},{104,0}}, color={0,0,0}));
  connect(hysteresis.n, ground.p) annotation (Line(points={{40,-10},{40,-16},{
          56,-16},{56,-22}}, color={0,0,255}));
  connect(emf.n, ground.p) annotation (Line(points={{72,-10},{72,-16},{56,-16},
          {56,-22}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
        Text(
          extent={{-62,28},{74,-20}},
          lineColor={28,108,200},
          textString="motor")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SimpleMotor;
