within CHEETA.Aircraft.Airframes.Wings;
model CHEETA
  extends AircraftDynamics.Aircraft.Airframes.Wings.Shevell(
    lex=0.0,
    winglethOverB=0,
    I_sc=Modelon.Units.Conversions.from_lbmPerFt2(3.5),
    sref=242.136,
    ar=44.84/5.4,
    sweep=Modelica.SIunits.Conversions.from_deg(28),
    toverc=0.1050,
    taper=3/7.8,
    isSupercritical=1,
    wingdihedral=Modelica.SIunits.Conversions.from_deg(5.0),
    chordextspan=9/22.4,
    xmaingearOverFuselength=25.8/54.08,
    geartrackOverFusewidth=8.1/5.64,
    flapspanOverB=0.8,
    flapchordOverC=0.2 "This is an approximation",
    tex=11.1/7.8 - 1,
    prescribeTex=false,
    texangle=1.5707963267949,
    prescribeTexangle=true);
end CHEETA;
