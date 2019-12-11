within CHEETA.Blocks;
model ControlledSine
  import Modelica.Constants.pi;
  parameter Real freqHz(start=1) "Frequency of sine wave";
  parameter Real phase=0 "Phase of sine wave";
  extends Modelica.Blocks.Interfaces.SignalSource;
  Modelica.Blocks.Interfaces.RealInput A
    annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
equation
  y = (if time < startTime then 0 else A*Modelica.Math.sin(2
    *pi*freqHz*(time - startTime) + phase));

end ControlledSine;
