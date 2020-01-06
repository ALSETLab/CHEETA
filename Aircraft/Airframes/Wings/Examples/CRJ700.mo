within CHEETA.Aircraft.Airframes.Wings.Examples;
model CRJ700 "CRJ700 wing"
  extends AircraftDynamics.Aircraft.Airframes.Wings.Shevell(
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
    xmaingearOverFuselength=Modelon.Units.Conversions.from_ft(59.7308)/Modelon.Units.Conversions.from_ft(107.825),
    geartrackOverFusewidth=1.6,
    I_sc=Modelon.Units.Conversions.from_lbmPerFt2(3.5),
    flapspanOverB=0.627,
    flapchordOverC=0.36);
                          /*,
    lngth=28.438534281966582,
    arwgross=8.701,
    arlength=0.8081845029545204,
    croot=17.820307023421112,
    crootwexp=16.079143661421146,
    cbreak=13.562679269385338,
    ctip=3.6282145099685383,
    mac=12.289368565203969,
    macwexp=11.164742998864359,
    ymac=18.182723359438302,
    swexpOverSref=0.8060952775856529,
    swgrossOverSref=1.0,
    xac=57.70457725660941,
    xwingrootle=44.963164000000006,
    ywingrootle=0,
    zwingrootle=-5.724,
    xwingbreakle=52.406259931599415,
    ywingbreakle=13.996768939651751,
    zwingbreakle=-4.9914657140680685,
    xwingtiple=69.77348377199804,
    ywingtiple=46.591956181051756,
    zwingtiple=-3.2822190468935597,
    xmaingear=59.7308
   */
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end CRJ700;
