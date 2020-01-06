within CHEETA.Aircraft.Airframes.Fuselages.Examples;
model CHEETA "CHEETA fuselage"
  extends AircraftDynamics.Aircraft.Airframes.Fuselages.Shevell(
    seatlayout1=23,
    seatwidth=Modelon.Units.Conversions.from_in(20.54),
    seatlayout2=0,
    aislewidth=Modelon.Units.Conversions.from_in(19.0),
    seatpitch=Modelon.Units.Conversions.from_in(32.0),
    fusehoverw=1.0,
    nosefineness=1.5,
    tailfineness=2.5,
    windshieldht=Modelon.Units.Conversions.from_ft(2.5),
    pilotlength=Modelon.Units.Conversions.from_ft(6.0),
    fwdspace=Modelon.Units.Conversions.from_ft(0.7),
    aftspace=Modelon.Units.Conversions.from_ft(0.0),
    n_coachseats=115,
    wingheight=0.0,
    wingxposition=0.417,
    xnosegearOverNoselength=0.8,
    cabinloadfactor=94/115,
    n_crew=2,
    n_att=3);
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end CHEETA;
