within CHEETA.Aircraft.Electrical;
package HTS "Models for the high temperature superconductor"

 model HTS

    Modelica.Blocks.Interfaces.RealInput temperature
     annotation (Placement(transformation(extent={{-58,-94},{-18,-54}}),
         iconTransformation(
         extent={{-20,-20},{20,20}},
         rotation=90,
         origin={0,-60})));
   parameter Real l "Length of wire";
   Real p_loss "Power loss";
   Real p_out "Output power";
   Real v_loss "Loss voltage";

   Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (Placement(
         transformation(extent={{-100,-10},{-80,10}}),iconTransformation(extent={{-100,
             -10},{-80,10}})));
   Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (Placement(
         transformation(extent={{80,-10},{100,10}}),iconTransformation(extent={{80,-10},
             {100,10}})));

 equation
   p_loss = 0.001 + (0.0009*(temperature - 67)) * l * (pin_p.i * pin_p.i);
   p_out = (pin_p.v * pin_p.i) - p_loss;
   v_loss = p_loss/pin_p.i;
   pin_n.i = pin_p.i;
   pin_n.v = pin_p.v;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
            -40},{80,40}}), graphics={
                   Rectangle(
           extent={{-80,40},{80,-40}},
           lineColor={0,0,255},
           fillColor={255,255,255},
           fillPattern=FillPattern.Solid),Rectangle(
           extent={{-60,20},{60,-20}},
           lineColor={0,0,255},
           fillColor={95,95,95},
           fillPattern=FillPattern.Solid),Text(
           extent={{-60,20},{60,-20}},
           lineColor={255,255,0},
           textString="%name")}),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-80,-40},{80,40}})));
 end HTS;
end HTS;
