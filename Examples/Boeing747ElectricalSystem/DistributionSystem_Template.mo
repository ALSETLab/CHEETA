within CHEETA.Examples.Boeing747ElectricalSystem;
model DistributionSystem_Template "Template for Boeing 747 electrical system"
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.0001,11900;
        0.5,12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,
        0.0], timeScale=3600)
                      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={94,68})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={62,68})));
  replaceable Aircraft.Electrical.Interfaces.Generation            generation
    constrainedby AircraftPowerSystem.Components.Interfaces.Generation
    annotation (Placement(transformation(
        extent={{-16,-10},{16,10}},
        rotation=0,
        origin={12,68})));
  replaceable Aircraft.Electrical.Interfaces.Loads            DC_Load_1
    constrainedby AircraftPowerSystem.Components.Interfaces.Loads
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  replaceable Aircraft.Electrical.Interfaces.Loads            DC_Load_2
    constrainedby AircraftPowerSystem.Components.Interfaces.Loads
    annotation (Placement(transformation(extent={{0,-20},{20,0}})));
  replaceable Aircraft.Electrical.Interfaces.Loads            AC_Load
    constrainedby AircraftPowerSystem.Components.Interfaces.Loads
    annotation (Placement(transformation(
        extent={{-9,-8},{9,8}},
        rotation=180,
        origin={-73,68})));
equation
  connect(timeTable.y,RPMtoRPS. u)
    annotation (Line(points={{83,68},{74,68}},     color={0,0,127}));
  connect(generation.w_ref, RPMtoRPS.y)
    annotation (Line(points={{29,68},{51,68}},   color={0,0,127}));
  connect(AC_Load.AC_in, generation.AC_out) annotation (Line(points={{-64,68},{
          -46,68},{-46,68.2},{-4.2,68.2}},     color={0,0,255}));
  connect(DC_Load_1.AC_in, generation.AC_out) annotation (Line(points={{0,30},{
          -46,30},{-46,68.2},{-4.2,68.2}},       color={0,0,255}));
  connect(DC_Load_2.AC_in, generation.AC_out) annotation (Line(points={{0,-10},
          {-46,-10},{-46,68.2},{-4.2,68.2}},         color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -40},{120,100}})),       Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-40},{120,100}})));
end DistributionSystem_Template;
