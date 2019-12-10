within CHEETA.Aircraft.Airframes.Fuselages.Examples;
model FixedFuselage "Fixed Fuseage sizing functionality test"
  // Test fuselage of DC-9-30 assuming wing weight of 10521 lbm and wing sizing of Douglas DC-9-30
  extends
    AircraftDynamics.Aircraft.Airframes.Fuselages.Experiments.Templates.Minimal(
    redeclare CHEETA fuselage(prescribeWallthickness=false),
    m_c=cat(
        1,
        {0,Modelon.Units.Conversions.from_lbm(10521),0,0,2*
          Modelon.Units.Conversions.from_lbm(3320.5 + 1992.3)},
        zeros(AircraftDynamics.Utilities.Mass.nCategories - 5)),
    wingSizing=AircraftDynamics.Aircraft.Airframes.Wings.Records.DouglasDC9(),
    wtStmt=AircraftDynamics.Aircraft.Records.DouglasDC9());
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end FixedFuselage;
