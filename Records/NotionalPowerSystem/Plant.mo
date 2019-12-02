within CHEETA.Records.NotionalPowerSystem;
record Plant "Parameters of the converter (First order model 1/(L*s+(R+ron)))"
  extends Modelica.Icons.Record;
  parameter Real R = 5e-3 "Resistance of external network";
  parameter Real L = 690e-6 "Inductance of external network";
  parameter Real ron = 0.88e-3 "Resistance of the switch ON state";
  parameter Real lambda = 5e-3 "Closed-loop time constant";
  parameter Real Vd = 1200 "DC side voltage of the converter";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Plant;
