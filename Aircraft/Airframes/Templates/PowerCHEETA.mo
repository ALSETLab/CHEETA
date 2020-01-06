within CHEETA.Aircraft.Airframes.Templates;
model PowerCHEETA "CHEETA primary and secondary power system"
  import AircraftDynamics;
  extends AircraftDynamics.Aircraft.Power.Shevell(
    redeclare AircraftDynamics.Aircraft.Power.Engines.Examples.DouglasDC9_HBR
                                                                          engine6(slsthrust=Modelon.Units.Conversions.from_lbf(14500)),
    redeclare AircraftDynamics.Aircraft.Power.Engines.Examples.DouglasDC9_HBR
                                                                          engine5(slsthrust=Modelon.Units.Conversions.from_lbf(14500)),
    redeclare AircraftDynamics.Aircraft.Power.APUs.Examples.Generic apu,
    redeclare AircraftDynamics.Aircraft.Power.Electric.Examples.Generic electric,
    redeclare AircraftDynamics.Aircraft.Power.Hydraulic.Examples.Generic hydraulic,
    redeclare AircraftDynamics.Aircraft.Power.Pneumatic.Examples.Generic pneumatic,
    redeclare AircraftDynamics.Aircraft.Power.Fuel.Examples.FixedMass fuel);
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end PowerCHEETA;
