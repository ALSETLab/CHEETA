within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_IG
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={96,-16})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={46,-16})));
  Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
    AC_Hydraulic_Pump(
    p=8,
    fsNominal=400,
    TsOperational=298.15,
    Rs=0.2761,
    TsRef=298.15,
    alpha20s(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    Lssigma=0.0002191,
    Jr=0.01,
    frictionParameters(
      PRef=0.000001,
      wRef=1.0471975511966e-7,
      power_w=0.000001),
    phiMechanical(fixed=true, start=0),
    wMechanical(start=0, fixed=true),
    statorCoreParameters(VRef=115),
    strayLoadParameters(power_w=0.00001),
    Lm=0.07614,
    Lrsigma=0.0002191,
    Rr=0.16,
    TrRef=298.15,
    alpha20r(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    TrOperational=298.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-2,30})));

  AircraftPowerSystem.Systems.SG SGandAVR annotation (Placement(transformation(
          rotation=0, extent={{-2,-24},{18,-4}})));
  AircraftPowerSystem.Systems.Pump_Load pump_Load annotation (Placement(
        transformation(rotation=0, extent={{-60,-6},{-40,14}})));
  Modelica.Electrical.MultiPhase.Basic.Star star(final m=3) annotation (
      Placement(transformation(extent={{-24,88},{-44,108}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        origin={-64,98},
        extent={{-10,-10},{10,10}},
        rotation=270)));
equation
  connect(RPMtoRPS.u,timeTable. y)
    annotation (Line(points={{58,-16},{85,-16}},   color={0,0,127}));
  connect(RPMtoRPS.y, SGandAVR.w_ref) annotation (Line(points={{35,-16},{30,-16},
          {30,-17.4},{18,-17.4}},      color={0,0,127}));
  connect(SGandAVR.plugSupply, AC_Hydraulic_Pump.plug_sp)
    annotation (Line(points={{8,-4},{8,24}}, color={0,0,255}));
  connect(pump_Load.flange, AC_Hydraulic_Pump.flange)
    annotation (Line(points={{-40,10},{-2,10},{-2,20}}, color={0,0,0}));
  connect(star.pin_n, ground1.p)
    annotation (Line(points={{-44,98},{-54,98}}, color={0,0,255}));
  connect(AC_Hydraulic_Pump.plug_sn, star.plug_p)
    annotation (Line(points={{8,36},{8,98},{-24,98}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{240,
            140}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            240,140}})),
    experiment(
      StopTime=36000,
      __Dymola_NumberOfIntervals=50000,
      Tolerance=1e-05,
      __Dymola_fixedstepsize=1e-05,
      __Dymola_Algorithm="Dassl"));
end SG_IG;
