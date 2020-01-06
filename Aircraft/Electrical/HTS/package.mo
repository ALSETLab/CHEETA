within CHEETA.Aircraft.Electrical;
package HTS "Models for the high temperature superconductor"

 replaceable model HTS

    Modelica.Electrical.Analog.Interfaces.Pin
                                         p_in
     annotation (Placement(transformation(extent={{-122,-18},{-82,22}}),
        iconTransformation(extent={{-108,-16},{-80,12}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin
                                          p_out
     annotation (Placement(transformation(extent={{90,-10},{110,10}}),
        iconTransformation(extent={{80,-16},{108,12}})));

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a
                                         temperature
     annotation (Placement(transformation(extent={{-22,-116},{18,-76}}),
        iconTransformation(extent={{-16,-70},{14,-40}})));
   parameter Real l= 1 "Length of wire";
   Real p_loss = 0 "Power loss";

 equation
   p_loss = 0.001 + (0.0009*(temperature.T - 67)) * l * (p_in.i *p_in.i);
   p_out.v = ((p_in.v *p_in.i)  - p_loss)/p_in.i;
   p_out.i = p_in.i;

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
