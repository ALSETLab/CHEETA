within CHEETA.Aircraft.Airframes.Wings.Examples;
model FixedWing "Fixed Wing sizing functionality test"
  import AircraftDynamics;
  extends
    AircraftDynamics.Aircraft.Airframes.Wings.Experiments.Templates.Minimal(
    redeclare AircraftDynamics.Aircraft.Airframes.Wings.Examples.DouglasDC9
      wing,
    wtStmt=AircraftDynamics.Aircraft.Records.DouglasDC9(),
    fuselageSizing=
        AircraftDynamics.Aircraft.Airframes.Fuselages.Records.DouglasDC9(),
    tailSizing=AircraftDynamics.Aircraft.Airframes.Tails.Records.DouglasDC9(),
    engine1Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.None(),
    engine2Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.DouglasDC9(),
    engine3Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.DouglasDC9(),
    engine4Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.None(),
    engine5Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.None(),
    engine6Sizing=AircraftDynamics.Aircraft.Power.Engines.Records.None());

      inner AircraftDynamics.Utilities.CoordinateSystems.AeroFrames aeroFrames(
    toCoG={10,0,0},
    alpha=0,
    beta=0) annotation (Placement(transformation(extent={{60,-80},{80,-60}})));
protected
  inner AircraftDynamics.Aircraft.Airframes.Aerodynamics.Interfaces.DynamicState
                                                       dynamicStateBus
    annotation (Placement(transformation(extent={{60,-100},{80,-80}}),
        iconTransformation(extent={{-176,52},{-156,72}})));
public
  Modelica.Blocks.Sources.Constant machnumber(k=0.6)
    annotation (Placement(transformation(extent={{20,-100},{40,-80}})));
equation
  connect(aeroFrames.wnb, world.frame_b) annotation (Line(
      points={{60,-70},{-56,-70},{-56,-26},{-64,-26}},
      color={95,95,95},
      thickness=0.5));
  connect(machnumber.y, dynamicStateBus.mach) annotation (Line(points={{41,-90},{56,-90},{56,-89.95},
          {70.05,-89.95}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  annotation (Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>"));
end FixedWing;
