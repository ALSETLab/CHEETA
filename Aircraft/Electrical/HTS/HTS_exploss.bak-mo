within CHEETA.Aircraft.Electrical.HTS;
model HTS_exploss
  parameter Real temperature;
  parameter Modelica.SIunits.Length l "Length of wire";
  parameter Modelica.SIunits.ElectricFieldStrength E_0 = 1e-4 "Reference electric field";
  parameter Real n = 1 "Intrinstic value of the superconductor";
  parameter Modelica.SIunits.Current I_c = 1 "corner current";
  Modelica.SIunits.ElectricFieldStrength E "Electric field";


  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p             annotation (Placement(
        transformation(extent={{-100,-10},{-80,10}}),iconTransformation(extent={{-100,
            -10},{-80,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (Placement(
        transformation(extent={{80,-10},{100,10}}),iconTransformation(extent={{80,-10},
            {100,10}})));
  parameter Modelica.SIunits.Resistance R = 1000 "Resistance at temperature T_ref";
equation

  E = E_0 *(pin_p.i/I_c)^n;
  pin_n.i = pin_p.i;
  pin_n.v = pin_p.v - E*l;

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
end HTS_exploss;
