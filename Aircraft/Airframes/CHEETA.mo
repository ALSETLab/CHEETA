within CHEETA.Aircraft.Airframes;
model CHEETA "Bombardier CHEETA commercial aircraft"
  extends AircraftDynamics.Aircraft.Shevell(
    m_mtow=Modelon.Units.Conversions.from_lbm(72750),
    payloadmargin=Modelon.Units.Conversions.from_lbm(2850),
    redeclare Templates.AirframeCHEETA airframe(
      ar=12.5,
      sweep=Modelica.Units.Conversions.from_deg(33.5),
      winglethOverB=0.1,
      span=Modelon.Units.Conversions.from_ft(115.0)),
    redeclare Templates.PowerCHEETA power,
    redeclare Templates.DomesticCPS systems,
    aggregateMass(resolve=Modelon.Mechanics.MultiBody.Internal.AggregateMass.Types.ResolveEnumeration.RESOLVE),
    airframeData(l=5.58, mass=80000));

equation
  connect(aggregateMass.frame_resolve, aeroFrames.wnb) annotation (Line(
      points={{-90,-100},{-90,-106},{-196,-106},{-196,78},{-182,78}},
      color={95,95,95},
      pattern=LinePattern.Dot));
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"),
       __Dymola_Commands(file="SimulateFlightManeuver.mos"));
end CHEETA;
