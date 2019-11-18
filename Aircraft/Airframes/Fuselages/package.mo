within CHEETA.Aircraft.Airframes;
package Fuselages

  model CHEETA "CHEETA hybrid aircraft fuselage"
    extends AircraftDynamics.Aircraft.Airframes.Fuselages.Shevell(
      sref=Modelon.Units.Conversions.from_ft2(1000.7),
      ar=8.701,
      sweep=Modelica.SIunits.Conversions.from_deg(24.5),
      toverc=0.1123,
      taper=0.2036,
      isSupercritical=0,
      wingdihedral=Modelica.SIunits.Conversions.from_deg(3.0),
      lex=0.0,
      tex=0.0,
      chordextspan=0.3,
      winglethOverB=0,
      xmaingearOverFuselength=Modelon.Units.Conversions.from_ft(59.7308)/
          Modelon.Units.Conversions.from_ft(107.825),
      geartrackOverFusewidth=1.6,
      I_sc=Modelon.Units.Conversions.from_lbmPerFt2(3.5),
      flapspanOverB=0.627,
      flapchordOverC=0.36);
    annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
  end CHEETA;
end Fuselages;
