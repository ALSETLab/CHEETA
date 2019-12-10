within CHEETA.Examples.Boeing747ElectricalSystem;
model DistributionSystem
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.1,11900;
        0.5,12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,
        0.0], timeScale=3600)
                      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={78,60})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,60})));
  replaceable Aircraft.Electrical.Machines.Examples.Boeing747.SynGenwAVR generation(
      useDamperCage=false) constrainedby
    AircraftPowerSystem.Components.Interfaces.Generation annotation (Placement(
        transformation(
        extent={{-11,-7},{11,7}},
        rotation=0,
        origin={-9,53})));
  replaceable Aircraft.Electrical.Loads.Boeing747.Fuel_Pump PUMP(
    N=200/(sqrt(2)*28),
    V_rated=28,
    L=-1.5/200,
    P_fixed=0.0001) constrainedby
    AircraftPowerSystem.Components.Interfaces.Loads
    annotation (Placement(transformation(extent={{-18,-2},{2,18}})));
  replaceable Aircraft.Electrical.Loads.Boeing747.PMSM PMSM(
    N=1,
    P_fixed=200,
    V_rated=1) constrainedby AircraftPowerSystem.Components.Interfaces.Loads
    annotation (Placement(transformation(extent={{40,-68},{60,-48}})));
  replaceable Aircraft.Electrical.Loads.Boeing747.Induction_Motor IM(
    R_l=100,
    L_l=0,
    startTime=10) constrainedby AircraftPowerSystem.Components.Interfaces.Loads
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-82,40})));
  Modelica.Blocks.Sources.TimeTable BallScrewSpeedCommands1(table=[0.0,0;
        2,0; 3,40; 5,60; 6,0; 7,50; 8,-50; 9,0; 10,0.0; 11,0.0],
      timeScale=10)  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-10,-30})));
  Modelica.Electrical.MultiPhase.Basic.Inductor inductor(L=fill(1e-3, 3))
    annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
equation
  connect(timeTable.y,RPMtoRPS. u)
    annotation (Line(points={{67,60},{52,60}},     color={0,0,127}));
  connect(generation.w_ref, RPMtoRPS.y)
    annotation (Line(points={{2,53},{18,53},{18,60},{29,60}},
                                                 color={0,0,127}));
  connect(IM.AC_in, generation.AC_out) annotation (Line(points={{-72,41.1111},{
          -42,41.1111},{-42,53.14},{-20.1375,53.14}},    color={0,0,255}));
  connect(PUMP.AC_in, generation.AC_out) annotation (Line(points={{-18,8},{-42,
          8},{-42,53.14},{-20.1375,53.14}},
                                          color={0,0,255}));
  connect(BallScrewSpeedCommands1.y, PMSM.Ref) annotation (Line(points={{1,-30},
          {28,-30},{28,-52.8333},{40.8333,-52.8333}},           color={0,
          0,127}));
  connect(inductor.plug_n, PMSM.AC_in) annotation (Line(points={{0,-60},{30,-60},
          {30,-59.6667},{40,-59.6667}},             color={0,0,255}));
  connect(inductor.plug_p, generation.AC_out) annotation (Line(points={{-20,-60},
          {-42,-60},{-42,53.14},{-20.1375,53.14}},   color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,80}})),        Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-80},{100,80}})),
    experiment(
      StopTime=10,
      Interval=0.0001,
      Tolerance=1e-08,
      __Dymola_Algorithm="Dassl"));
end DistributionSystem;
