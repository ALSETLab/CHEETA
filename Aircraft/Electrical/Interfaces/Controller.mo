within CHEETA.Aircraft.Electrical.Interfaces;
partial model Controller
  Modelica.Blocks.Interfaces.RealInput Measurement
    annotation (Placement(transformation(extent={{-300,-60},{-260,-20}})));
  Modelica.Blocks.Interfaces.RealOutput Actuation
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput Reference
    annotation (Placement(transformation(extent={{-300,20},{-260,60}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -280,-100},{100,100}})),                             Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-280,-100},{
            100,100}})));
end Controller;
