within CHEETA.Aircraft.Airframes.Tails.Examples;
model CHEETA "CHEETA tail"
  extends AircraftDynamics.Aircraft.Airframes.Tails.Shevell(
    svOverSref=0.161,
    arv=1.1,
    sweepv=Modelica.SIunits.Conversions.from_deg(43.5),
    tovercv=0.11,
    taperv=0.8016,
    shOverSref=0.26,
    arh=4.928,
    sweeph=Modelica.SIunits.Conversions.from_deg(31.6),
    toverch=0.087,
    taperh=0.352,
    hasTeeTail=0.98,
    dihedralh=Modelica.SIunits.Conversions.from_deg(-3.0),
    clhmax=1.2,
    I_sc=Modelon.Units.Conversions.from_lbmPerFt2(3.5),
    rightSection(color={1,24,72}),
    leftSection(color={1,24,72}));

  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end CHEETA;
