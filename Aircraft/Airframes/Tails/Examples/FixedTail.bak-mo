within CHEETA.Aircraft.Airframes.Tails.Examples;
model FixedTail "Fixed Tail sizing functionality test"
  import AircraftDynamics;
  // Test tail of DC-9-30 assuming wing sizing, fuselage sizing, weight statement of Douglas DC-9-30
  extends
    AircraftDynamics.Aircraft.Airframes.Tails.Experiments.Templates.Minimal(
    redeclare CHEETA tail,
    fuselageSizing=
        AircraftDynamics.Aircraft.Airframes.Fuselages.Records.DouglasDC9(),
    wingSizing=AircraftDynamics.Aircraft.Airframes.Wings.Records.DouglasDC9(),
    wtStmt=AircraftDynamics.Aircraft.Records.DouglasDC9());

  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end FixedTail;
