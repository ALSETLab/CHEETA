within CHEETA.Aircraft.Airframes.LandingGears.Examples;
model Flotation "Model of a tricycle landing gear in contact with the ground"
  import AircraftDynamics;
  extends
    AircraftDynamics.Aircraft.Airframes.LandingGears.Experiments.Templates.Minimal(
      redeclare
      AircraftDynamics.Aircraft.Airframes.LandingGears.Examples.Mechanical3D
      landingGear(
      nosegearLength=2,
      prescribeNosegearLength=true,
      maingearLength=2,
      prescribeMaingearLength=true,
      frontGear(bogie(wheel1(enable_signal_bus=true), wheel2(enable_signal_bus=
                true))),
      rightGear(bogie(
          wheel1(enable_signal_bus=true),
          wheel2(enable_signal_bus=true),
          wheel3(enable_signal_bus=true),
          wheel4(enable_signal_bus=true),
          wheel5(enable_signal_bus=true),
          wheel6(enable_signal_bus=true))),
      leftGear(bogie(
          wheel1(enable_signal_bus=true),
          wheel2(enable_signal_bus=true),
          wheel3(enable_signal_bus=true),
          wheel4(enable_signal_bus=true),
          wheel5(enable_signal_bus=true),
          wheel6(enable_signal_bus=true)))), world(n={0,0,1}));

  extends AircraftDynamics.Utilities.Mass.MixIn.Masses;
  extends AircraftDynamics.Utilities.Mass.MixIn.WingSizing(wingSizing=
        AircraftDynamics.Aircraft.Airframes.Wings.Records.DouglasDC9());
  extends AircraftDynamics.Utilities.Mass.MixIn.FuselageSizing(fuselageSizing=
        AircraftDynamics.Aircraft.Airframes.Fuselages.Records.DouglasDC9());

  Modelica.SIunits.Force summary_fW_z_max=max(contact_force_z);

  AircraftDynamics.Utilities.MultiBody.Parts.VariableTranslation
    toRightMlgFrame(r={18,3,0})
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  AircraftDynamics.Utilities.MultiBody.Parts.VariableTranslation toLeftMlgFrame(
      r={18,-3,0})
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  AircraftDynamics.Utilities.MultiBody.Parts.VariableTranslation toNlgFrame(r={0,
        0,0})
    annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  inner replaceable AircraftDynamics.Grounds.Flat ground(visualize=true)
    constrainedby AircraftDynamics.Grounds.Interfaces.Base annotation (
      choicesAllMatching, Placement(transformation(extent={{60,-100},{100,-80}})));
  inner AircraftDynamics.Utilities.CoordinateSystems.AeroFrames aeroFrames(
    alpha=0,
    beta=0,
    toCoG={17,0,0})
    annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));

  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing
    engine1Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
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
      zcg=0) "Wing-mounted"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing
    engine2Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=1,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     6),
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
                      -1)) "Wing-mounted"
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing
    engine3Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
      slsthrust=1,
      wdryengineOverSlst=1,
      sfcOverSfcref=1,
      minGroundClearance=Modelon.Units.Conversions.from_ft(
                                     6),
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
                      -1)) "Wing-mounted"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing
    engine4Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
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
      zcg=0) "Wing-mounted"
    annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing
    engine5Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
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
      zcg=0) "Fuselage-mounted"
    annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
  inner output AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing
    engine6Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.BasicSizing(
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
      zcg=0) "Fuselage-mounted"
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  inner output AircraftDynamics.Aircraft.Records.BasicWtStmt wtStmt=
      AircraftDynamics.Aircraft.Records.BasicWtStmt(
      m_mtow=Modelon.Units.Conversions.from_lbm(
                          100000),
      mzfwovermtow=0,
      m_mzfw=0,
      m_mew=0) "Weight statement"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Modelica.Mechanics.MultiBody.Joints.FreeMotion freeMotion(
    animation=false,
    r_rel_a(start={0,0,-4}),
    angles_start={0,0.017453292519943,0},
    useQuaternions=false)
    annotation (Placement(transformation(extent={{-60,-80},{-40,-60}})));
  Modelon.Mechanics.MultiBody.Parts.BodyNoStates partialAircraftMass(m=45000)
    annotation (Placement(transformation(extent={{30,-50},{50,-30}})));
  AircraftDynamics.Utilities.MultiBody.Parts.VariableTranslation toCoG(r={-17,0,
        0}, visualize=false)
    annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));
protected
  Modelica.Blocks.Interfaces.RealOutput contact_force_z[n_wheels]
    annotation (Placement(transformation(extent={{30,20},{50,40}})));
protected
  inner AircraftDynamics.Utilities.Busses.SignalBus signalBus
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
equation
  connect(landingGear.mainGearMount[1], toLeftMlgFrame.frame_b) annotation (
      Line(
      points={{-10,6},{-16,6},{-16,0},{-20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(toRightMlgFrame.frame_b, landingGear.mainGearMount[2]) annotation (
      Line(
      points={{-20,20},{-16,20},{-16,6},{-10,6}},
      color={95,95,95},
      thickness=0.5));
  connect(landingGear.noseGearMount, toNlgFrame.frame_b) annotation (Line(
      points={{-10,-6},{-16,-6},{-16,-20},{-20,-20}},
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
  connect(world.frame_b, freeMotion.frame_a) annotation (Line(
      points={{-60,-70},{-60,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(aeroFrames.body, partialAircraftMass.frame_a) annotation (Line(
      points={{10,-42},{20,-42},{20,-40},{30,-40}},
      color={95,95,95},
      thickness=0.5));
  connect(toNlgFrame.frame_a, aeroFrames.wnb) annotation (Line(
      points={{-40,-20},{-50,-20},{-50,-50},{-10,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(freeMotion.frame_b, toCoG.frame_a) annotation (Line(
      points={{-40,-70},{-20,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(toCoG.frame_b, partialAircraftMass.frame_a) annotation (Line(
      points={{0,-70},{20,-70},{20,-40},{30,-40}},
      color={95,95,95},
      thickness=0.5));
  connect(contact_force_z, signalBus.ADL_whl_frc_z)
    annotation (Line(points={{40,30},{10,30}}, color={0,0,127}));
  annotation (experiment(StopTime=5), Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end Flotation;
