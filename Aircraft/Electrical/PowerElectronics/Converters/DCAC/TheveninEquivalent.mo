within CHEETA.Aircraft.Electrical.PowerElectronics.Converters.DCAC;
model TheveninEquivalent "Single phase DC to AC converter"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.Resistance R = 1e-3 "Resistance of diode";
  parameter Modelica.SIunits.Resistance R_cm = 1e-2 "Common mode resistance";
  // parameter Boolean useEnable "Enables enable signal connector";
  Modelica.SIunits.Voltage V_th "Thevenin voltage";
  Modelica.SIunits.Resistance Z_th = R/3 + R_cm "Thevenin resistance";

  extends Modelica.Electrical.PowerConverters.Interfaces.DCAC.DCtwoPin;
  extends Modelica.Electrical.PowerConverters.Interfaces.DCAC.ACpin;


  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage   signalVoltage
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,-22})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=Z_th)
    annotation (Placement(transformation(extent={{-12,34},{-32,54}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=V_th)
    annotation (Placement(transformation(extent={{30,-32},{10,-12}})));
equation
  V_th = (dc_p.v - dc_n.v)/2;
  connect(dc_p, resistor.p) annotation (Line(points={{-100,60},{-80,60},{-80,20},
          {-60,20}}, color={0,0,255}));
  connect(signalVoltage.p, resistor.n)
    annotation (Line(points={{-20,-12},{-20,20},{-40,20}}, color={0,0,255}));
  connect(ac, resistor.n) annotation (Line(points={{100,0},{-20,0},{-20,20},{-40,
          20}}, color={0,0,255}));
  connect(resistor.R, realExpression.y)
    annotation (Line(points={{-50,32},{-50,44},{-33,44}}, color={0,0,127}));
  connect(dc_n, signalVoltage.n)
    annotation (Line(points={{-100,-60},{-20,-60},{-20,-32}}, color={0,0,255}));
  connect(signalVoltage.v, realExpression1.y)
    annotation (Line(points={{-8,-22},{9,-22}}, color={0,0,127}));
  annotation (defaultComponentName="inverter",
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}}), graphics={
        Line(
          points={{-100,-100},{100,100}},
          color={0,0,127}),
        Rectangle(
          extent={{-40,40},{40,-40}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-20,20},{-20,-20}},
          color={0,0,255}),
        Line(
          points={{-28,20},{-28,-20}},
          color={0,0,255}),
        Line(
          points={{-40,0},{-28,0}},
          color={0,0,255}),
        Line(
          points={{-20,4},{0,24},{0,40}},
          color={0,0,255}),
        Line(
          points={{-20,-4},{0,-24},{0,-40}},
          color={0,0,255}),
        Line(
          points={{-4,-20},{-10,-8},{-16,-14},{-4,-20}},
          color={0,0,255}),
        Line(
          points={{0,-24},{10,-24},{10,24},{0,24}},
          color={0,0,255}),
        Line(
          points={{0,8},{20,8}},
          color={0,0,255}),
        Line(
          points={{10,8},{0,-8},{20,-8},{10,8}},
          color={0,0,255}),
        Text(
          extent={{-100,70},{0,50}},
          lineColor={0,0,127},
          textString="DC"),
        Text(
          extent={{0,-50},{100,-70}},
          lineColor={0,0,127},
          textString="AC")}),
    Documentation(info="<html>
<p>
This is a single phase two level inverter. The boolean signals <code>fire_p</code> and <code>fire_n</code> shall not be <code>true</code> at the same time to avoid DC bus short circuits. The inverter consists of two transistors and two anti parallel free wheeling diodes.
</p>

<p>
An example of a single phase inverter with PWM voltage control is included in
<a href=\"modelica://Modelica.Electrical.PowerConverters.Examples.DCAC.SinglePhaseTwoLevel\">Examples.DCAC.SinglePhaseTwoLevel</a>.
</p>
</html>"));
end TheveninEquivalent;
