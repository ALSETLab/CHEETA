within CHEETA.Aircraft.Airframes.Templates;
model DomesticCPS
  "Domestic rubberized consumer systems and cabin/furbishing (Douglas DC9 and similar)"
  extends AircraftDynamics.Aircraft.Systems.Shevell(
    redeclare AircraftDynamics.Aircraft.Systems.Avionics.Examples.Domestic avionics,
    redeclare AircraftDynamics.Aircraft.Systems.AirConditioning.Examples.Generic airconditioning,
    redeclare AircraftDynamics.Aircraft.Systems.Cabins.Examples.Domestic cabin,
    redeclare AircraftDynamics.Aircraft.Systems.FlightDecks.Examples.Generic flightdeck);
equation

  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end DomesticCPS;
