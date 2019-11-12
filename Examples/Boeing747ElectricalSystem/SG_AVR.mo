within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_AVR
  parameter AircraftPowerSystem.Records.SynchronousMachine.SM100kVA Data(
    SNominal=100000,
    VsNominal=115,
    fsNominal=400,
    IeOpenCircuit=10,
    x0=0.15,
    xd=2,
    xq=1.9,
    xdTransient=0.245,
    xdSubtransient=0.2,
    xqSubtransient=0.2,
    Ta=0.001,
    Td0Transient=5,
    Td0Subtransient=0.031,
    Tq0Subtransient=0.061,
    TsSpecification=333.15,
    TsRef=298.15,
    alpha20s=0,
    TrSpecification=331.15,
    TrRef=298.15,
    alpha20r=0,
    TeSpecification=333.15,
    TeRef=298.15,
    alpha20e=0)
    annotation (Placement(transformation(extent={{50,36},{70,56}})));
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 10,1; 10,1],
      timeScale=3600) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={58,10})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=Modelica.Constants.pi/30)
                                                annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={38,32})));
  Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R={100,100,100})
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={2,24})));
  Modelica.Electrical.MultiPhase.Basic.Star star annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={2,4})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={2,-16},
        extent={{-6,-6},{6,6}},
        rotation=0)));
  Aircraft.Electrical.Machines.Boeing747_SG sG_747_1
    annotation (Placement(transformation(extent={{-18,58},{22,38}})));
equation
  connect(RPMtoRPS.u, timeTable.y)
    annotation (Line(points={{38,27.2},{38,10},{47,10}}, color={0,0,127}));
  connect(ground.p, star.pin_n)
    annotation (Line(points={{2,-10},{2,-6}},    color={0,0,255}));
  connect(resistor.plug_n, star.plug_p)
    annotation (Line(points={{2,18},{2,14}},     color={0,0,255}));
  connect(RPMtoRPS.y, sG_747_1.w_ref)
    annotation (Line(points={{38,36.4},{38,52.8},{24,52.8}}, color={0,0,127}));
  connect(resistor.plug_p, sG_747_1.plugSupply)
    annotation (Line(points={{2,30},{2,37.2}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-20},{80,60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-40,-20},{80,
            60}})),
    experiment(
      StopTime=36000,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-06));
end SG_AVR;
