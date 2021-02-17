within CHEETA.Aircraft.Airframes.Templates;
model AirframeCRJ700 "CRJ700 airframe"
  extends AircraftDynamics.Aircraft.Airframes.Shevell(
    redeclare AircraftDynamics.Aircraft.Airframes.Wings.Examples.DouglasDC9 wing(
      prescribeSref=false,
      ar=ar,
      prescribeAr=true,
      sweep=sweep,
      toverc=toverc,
      taper=taper,
      wingdihedral=wingdihedral,
      lex=lex,
      tex=tex,
      chordextspan=chordextspan,
      winglethOverB=winglethOverB,
      xmaingearOverFuselength=xmaingearOverFuselength,
      geartrackOverFusewidth=geartrackOverFusewidth,
      flapspanOverB=flapspanOverB,
      flapchordOverC=flapchordOverC,
      span=span,
      prescribeSpan=true,
      prescribeArwgross=false),
    redeclare AircraftDynamics.Aircraft.Airframes.Fuselages.Examples.DouglasDC9 fuselage(n_coachseats=n_coachseats, wingxposition=0.487),
    redeclare AircraftDynamics.Aircraft.Airframes.Tails.Examples.DouglasDC9 tail,
    redeclare AircraftDynamics.Aircraft.Airframes.LandingGears.Examples.RigidS landingGear,
    redeclare AircraftDynamics.Aircraft.Airframes.Brakes.BehavioralS brakes);

  parameter Real ar=7.76 "Aspect ratio" annotation (Dialog(tab="Airframe"));
  parameter Modelica.Units.SI.Angle sweep=Modelica.Units.Conversions.from_deg(
      24.5) "Quarter chord sweep" annotation (Dialog(tab="Airframe"));
  parameter Real toverc=0.1123 "Thickness over chord" annotation (Dialog(tab="Airframe"));
  parameter Real taper=0.2036 "Tip chord over root chord" annotation (Dialog(tab="Airframe"));
  parameter Modelica.Units.SI.Angle wingdihedral=
      Modelica.Units.Conversions.from_deg(4.0) "Dihedral"
    annotation (Dialog(tab="Airframe"));
  parameter Real lex=0.0 "Leading edge extension, fraction of root chord" annotation (Dialog(tab="Airframe"));
  parameter Real tex=0.0 "Trailing edge extension, fraction of root chord" annotation (Dialog(tab="Airframe"));
  parameter Real chordextspan=0.3 "Size of edge extensions, fraction of semi-span" annotation (Dialog(tab="Airframe"));
  parameter Real winglethOverB=0 "Winglet length over semi-span" annotation (Dialog(tab="Airframe"));
  parameter Real xmaingearOverFuselength=Modelon.Units.Conversions.from_ft(59.7308)/Modelon.Units.Conversions.from_ft(107.825) "x_mainGear over fuselage length"
    annotation (Dialog(tab="Airframe"));
  parameter Real geartrackOverFusewidth=1.6 "Gear track over fuselage width" annotation (Dialog(tab="Airframe"));
  parameter Real flapspanOverB=0.627 "Flap span over semi-span" annotation (Dialog(tab="Airframe"));
  parameter Real flapchordOverC=0.36 "Flap chord over chord" annotation (Dialog(tab="Airframe"));
  parameter Modelica.Units.SI.Distance span=Modelon.Units.Conversions.from_ft(
      150.0) "Span" annotation (Dialog(tab="Airframe"));
  parameter Real n_coachseats=126 "Number of seats in all-coach layout" annotation (Dialog(tab="Airframe"));
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"),
     __Dymola_Commands(file="SimulateFlightManeuver.mos" "Simulate Flight Maneuver"));
end AirframeCRJ700;
