within CHEETA.Aircraft.Electrical.HTS;
package Examples "Examples for the high temperature superconductor"


 replaceable model HTS

    Modelica.Blocks.Interfaces.RealInput i_in
     annotation (Placement(transformation(extent={{-100,60},{-60,100}})));
    Modelica.Blocks.Interfaces.RealOutput p_out
     annotation (Placement(transformation(extent={{96,-10},{116,10}})));
    Modelica.Blocks.Interfaces.RealInput v_in
     annotation (Placement(transformation(extent={{-100,-100},{-60,-60}})));

    Modelica.Blocks.Interfaces.RealInput temperature
     annotation (Placement(transformation(extent={{-100,-20},{-60,20}})));
   parameter Real l "Length of wire";
   Real p_loss "Power loss";

 equation
   p_loss = 0.001 + (0.0009*(temperature - 67)) * l * (i_in * i_in);
   p_out = (v_in * i_in) - p_loss;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
 end HTS;
end Examples;
