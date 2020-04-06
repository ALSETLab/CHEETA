within CHEETA.Atmospheres.CoolingMedium;
record Air_30degC "Medium: properties of air at 115K and 1 bar"
extends Modelica.Thermal.FluidHeatFlow.Media.Medium(
  rho=1.149,
  cp=1007,
  cv= 720,
  lamda=0.0264,
  nue=16.3E-6);
  annotation (defaultComponentPrefixes="parameter", Documentation(info="<html>
  Medium: properties of air at 30&deg;C and 1 bar
</html>"));
end Air_30degC;
