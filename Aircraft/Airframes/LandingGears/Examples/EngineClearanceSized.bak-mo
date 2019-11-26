within CHEETA.Aircraft.Airframes.LandingGears.Examples;
model EngineClearanceSized
  "Model of nose landing gear whose length is defined to satisfy a minimum engine ground clearance"
  import AircraftDynamics;
  extends
    AircraftDynamics.Aircraft.Airframes.LandingGears.Experiments.Templates.Minimal(
      redeclare
      AircraftDynamics.Aircraft.Airframes.LandingGears.Examples.ResizeableRigidS
      landingGear);
  extends AircraftDynamics.Utilities.Mass.MixIn.Masses;
  extends AircraftDynamics.Utilities.Mass.MixIn.WingSizing(wingSizing=
        AircraftDynamics.Aircraft.Airframes.Wings.Records.DouglasDC9());
  extends AircraftDynamics.Utilities.Mass.MixIn.FuselageSizing(fuselageSizing=
        AircraftDynamics.Aircraft.Airframes.Fuselages.Records.DouglasDC9());

  inner output AircraftDynamics.Aircraft.Records.BasicWtStmt wtStmt=
      AircraftDynamics.Aircraft.Records.BasicWtStmt(
      m_mtow=Modelon.Units.Conversions.from_lbm(100000),
      mzfwovermtow=0,
      m_mzfw=0,
      m_mew=0) "Weight statement"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

  // Here, engine2Sizing and engine3Sizing have the highest requirements concerning nominal gear length
  //  - Engine cg is down at zcg = -1
  //  - Need to go half maximum engine diameter down (3.5/2 = 1.75)
  //  - Need ground clearance of 2
  //  - Given the gear position at the origin, this results in a nominal gear length of 4.75
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing engine1Sizing=
      AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=1,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     3),
      corelength=Modelon.Units.Conversions.from_ft(
                             1),
      corediameter=Modelon.Units.Conversions.from_ft(
                               3),
      fanlength=Modelon.Units.Conversions.from_ft(
                            1),
      fandiameter=Modelon.Units.Conversions.from_ft(
                              1),
      turbinelength=Modelon.Units.Conversions.from_ft(
                                1),
      turbinediameter=Modelon.Units.Conversions.from_ft(
                                  1),
      fuseoffset=Modelon.Units.Conversions.from_ft(
                             0.5),
      wingoffset=Modelon.Units.Conversions.from_ft(
                             0.75),
      xcg=0,
      ycg=0,
      zcg=0) "Wing-mounted" annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing engine2Sizing=
      AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=1,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     2),
      corelength=Modelon.Units.Conversions.from_ft(
                             1),
      corediameter=Modelon.Units.Conversions.from_ft(
                               3.5),
      fanlength=Modelon.Units.Conversions.from_ft(
                            1),
      fandiameter=Modelon.Units.Conversions.from_ft(
                              1),
      turbinelength=Modelon.Units.Conversions.from_ft(
                                1),
      turbinediameter=Modelon.Units.Conversions.from_ft(
                                  1),
      fuseoffset=Modelon.Units.Conversions.from_ft(
                             0.5),
      wingoffset=Modelon.Units.Conversions.from_ft(
                             0.75),
      xcg=0,
      ycg=0,
      zcg=Modelon.Units.Conversions.from_ft(
                      -1)) "Wing-mounted" annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing engine3Sizing=
      AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=1,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     2),
      corelength=Modelon.Units.Conversions.from_ft(
                             1),
      corediameter=Modelon.Units.Conversions.from_ft(
                               3.5),
      fanlength=Modelon.Units.Conversions.from_ft(
                            1),
      fandiameter=Modelon.Units.Conversions.from_ft(
                              1),
      turbinelength=Modelon.Units.Conversions.from_ft(
                                1),
      turbinediameter=Modelon.Units.Conversions.from_ft(
                                  1),
      fuseoffset=Modelon.Units.Conversions.from_ft(
                             0.5),
      wingoffset=Modelon.Units.Conversions.from_ft(
                             0.75),
      xcg=0,
      ycg=0,
      zcg=Modelon.Units.Conversions.from_ft(
                      -1)) "Wing-mounted" annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing engine4Sizing=
      AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=1,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     3),
      corelength=Modelon.Units.Conversions.from_ft(
                             1),
      corediameter=Modelon.Units.Conversions.from_ft(
                               3),
      fanlength=Modelon.Units.Conversions.from_ft(
                            1),
      fandiameter=Modelon.Units.Conversions.from_ft(
                              1),
      turbinelength=Modelon.Units.Conversions.from_ft(
                                1),
      turbinediameter=Modelon.Units.Conversions.from_ft(
                                  1),
      fuseoffset=Modelon.Units.Conversions.from_ft(
                             0.5),
      wingoffset=Modelon.Units.Conversions.from_ft(
                             0.75),
      xcg=0,
      ycg=0,
      zcg=0) "Wing-mounted" annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing engine5Sizing=
      AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=0,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     1),
      corelength=Modelon.Units.Conversions.from_ft(
                             1),
      corediameter=0,
      fanlength=Modelon.Units.Conversions.from_ft(
                            1),
      fandiameter=0,
      turbinelength=Modelon.Units.Conversions.from_ft(
                                1),
      turbinediameter=0,
      fuseoffset=Modelon.Units.Conversions.from_ft(
                             0.5),
      wingoffset=Modelon.Units.Conversions.from_ft(
                             0.75),
      xcg=0,
      ycg=0,
      zcg=0) "Fuselage-mounted" annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing engine6Sizing=
      AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=0,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     1),
      corelength=Modelon.Units.Conversions.from_ft(
                             1),
      corediameter=0,
      fanlength=Modelon.Units.Conversions.from_ft(
                            1),
      fandiameter=0,
      turbinelength=Modelon.Units.Conversions.from_ft(
                                1),
      turbinediameter=0,
      fuseoffset=Modelon.Units.Conversions.from_ft(
                             0.5),
      wingoffset=Modelon.Units.Conversions.from_ft(
                             0.75),
      xcg=0,
      ycg=0,
      zcg=0) "Fuselage-mounted" annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  AircraftDynamics.Utilities.MultiBody.Parts.VariableTranslation
                                                toRightMlgFrame(r={wingSizing.xmaingear,(wingSizing.ymaingear),
        0})
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  AircraftDynamics.Utilities.MultiBody.Parts.VariableTranslation
                                                toLeftMlgFrame(r={wingSizing.xmaingear,-(wingSizing.ymaingear),
        0})
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  AircraftDynamics.Utilities.MultiBody.Parts.VariableTranslation toNlgFrame(r={
        fuselageSizing.xnosegear,0,0})
    annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  inner replaceable AircraftDynamics.Grounds.Flat ground constrainedby
    AircraftDynamics.Grounds.Interfaces.Base annotation (choicesAllMatching,
      Placement(transformation(extent={{60,-100},{100,-80}})));
  inner AircraftDynamics.Utilities.CoordinateSystems.AeroFrames aeroFrames(
    alpha=0,
    beta=0,
    toCoG={10,0,0}) annotation (Placement(transformation(extent={{-30,-80},{-10,-60}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation nedToWnb(
    angle=180,
    animation=false,
    sequence={1,2,3},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis,
    n={1,0,0},
    angles={180,0,0}) "North-east-down to weight-and-balance"
    annotation (Placement(transformation(extent={{-60,-80},{-40,-60}})));
equation
  connect(landingGear.mainGearMount[1], toLeftMlgFrame.frame_b) annotation (Line(
      points={{-10,6},{-16,6},{-16,0},{-20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(toRightMlgFrame.frame_b, landingGear.mainGearMount[2]) annotation (Line(
      points={{-20,20},{-16,20},{-16,6},{-10,6}},
      color={95,95,95},
      thickness=0.5));
  connect(landingGear.noseGearMount, toNlgFrame.frame_b) annotation (Line(
      points={{-10,-6},{-16,-6},{-16,-20},{-20,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(world.frame_b, nedToWnb.frame_a) annotation (Line(
      points={{-60,-70},{-60,-70},{-60,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(nedToWnb.frame_b, aeroFrames.wnb) annotation (Line(
      points={{-40,-70},{-40,-70},{-30,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(nedToWnb.frame_b, toLeftMlgFrame.frame_a) annotation (Line(
      points={{-40,-70},{-34,-70},{-34,-50},{-50,-50},{-50,0},{-40,0}},
      color={95,95,95},
      thickness=0.5));
  connect(toLeftMlgFrame.frame_a, toRightMlgFrame.frame_a) annotation (Line(
      points={{-40,0},{-50,0},{-50,20},{-40,20}},
      color={95,95,95},
      thickness=0.5));
  connect(toRightMlgFrame.frame_a, toNlgFrame.frame_a) annotation (Line(
      points={{-40,20},{-50,20},{-50,-20},{-40,-20}},
      color={95,95,95},
      thickness=0.5));
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end EngineClearanceSized;
