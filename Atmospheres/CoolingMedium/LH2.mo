within CHEETA.Atmospheres.CoolingMedium;
record LH2 "Medium: properties of air at 115K and 1 bar"
  extends Modelica.Thermal.FluidHeatFlow.Media.Medium(
    rho=0.07099,
    cp=13.12,
    cv=2.43,
    lambda=0.0264,
    nu=16.3E-6);
  annotation (defaultComponentPrefixes="parameter", Documentation(info="<html>
  Medium: properties of air at 30&deg;C and 1 bar
</html>"));
end LH2;
