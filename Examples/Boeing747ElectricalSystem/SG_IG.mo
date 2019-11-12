within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_IG "Synchronous generator and induction generator"
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={104,20})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=Modelica.Constants.pi/30)
                                                annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={62,20})));
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
        rotation=0,
        origin={-20,28})));

  Aircraft.Electrical.Machines.Boeing747_SG SGandAVR
    annotation (Placement(transformation(rotation=0, extent={{0,16},{32,32}})));
  Aircraft.Mechanical.Pumps.Boeing747_Pump_Load pump_Load annotation (Placement(
        transformation(
        rotation=180,
        extent={{-10,-4},{10,4}},
        origin={12,8})));
  Modelica.Electrical.MultiPhase.Basic.Star star(final m=3) annotation (
      Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-40,28})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        origin={-40,2},
        extent={{-10,-10},{10,10}},
        rotation=0)));
equation
  connect(RPMtoRPS.u, timeTable.y)
    annotation (Line(points={{74,20},{93,20}}, color={0,0,127}));
  connect(RPMtoRPS.y, SGandAVR.w_ref) annotation (Line(points={{51,20},{34,20},
          {34,20.16},{33.6,20.16}},    color={0,0,127}));
  connect(SGandAVR.plugSupply, AC_Hydraulic_Pump.plug_sp)
    annotation (Line(points={{16,32.64},{16,38},{-14,38}},
                                             color={0,0,255}));
  connect(star.pin_n, ground1.p)
    annotation (Line(points={{-40,18},{-40,12}}, color={0,0,255}));
  connect(AC_Hydraulic_Pump.plug_sn, star.plug_p)
    annotation (Line(points={{-26,38},{-40,38}},      color={0,0,255}));
  connect(pump_Load.flange, AC_Hydraulic_Pump.flange)
    annotation (Line(points={{1,8},{-4,8},{-4,28},{-10,28}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-20},{120,60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-20},{120,
            60}})),
    experiment(
      StopTime=36000,
      __Dymola_NumberOfIntervals=50000,
      Tolerance=1e-05,
      __Dymola_fixedstepsize=1e-05,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>This model contains the system consisting of the induction motor generator and the synchronous generator for the Boeing 747 electrical system. </p>
<p><br>This is used as a basis for the rest of the electrical system model to connect to the rest of the aircraft.</p>
</html>"));
end SG_IG;
