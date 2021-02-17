within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_IG
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={212,106})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={168,106})));
  Modelica.Electrical.Machines.BasicMachines.InductionMachines.IM_SquirrelCage
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
        origin={18,96})));

  Aircraft.Electrical.Machines.Examples.Boeing747.Boeing747_SG SG_with_AVR
    annotation (Placement(transformation(
        rotation=0,
        extent={{-17,-17},{17,17}},
        origin={115,103})));
  Modelica.Electrical.Polyphase.Basic.Star star(final m=3)
    annotation (Placement(transformation(extent={{0,98},{-20,118}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        origin={-38,108},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    HydraulicPumpLoad(
    useSupport=false,
    tau_nominal=-28/500,
    w_nominal=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={20,58})));

  Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch switch
    annotation (Placement(transformation(extent={{48,96},{68,116}})));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=20, startValue=
        false)
    annotation (Placement(transformation(extent={{0,116},{20,136}})));
  Modelica.Blocks.Routing.BooleanReplicator booleanReplicator(nout=3)
    annotation (Placement(transformation(extent={{30,116},{50,136}})));
equation
  connect(RPMtoRPS.u,timeTable. y)
    annotation (Line(points={{180,106},{201,106}}, color={0,0,127}));
  connect(RPMtoRPS.y, SG_with_AVR.w_ref) annotation (Line(points={{157,106},{
          157,106.4},{133.7,106.4}},  color={0,0,127}));
  connect(star.pin_n, ground1.p)
    annotation (Line(points={{-20,108},{-28,108}},
                                                 color={0,0,255}));
  connect(AC_Hydraulic_Pump.plug_sn, star.plug_p)
    annotation (Line(points={{12,106},{12,108},{0,108}},
                                                      color={0,0,255}));
  connect(HydraulicPumpLoad.flange, AC_Hydraulic_Pump.flange) annotation (
      Line(points={{30,58},{38,58},{38,96},{28,96}}, color={0,0,0}));
  connect(switch.plug_p, AC_Hydraulic_Pump.plug_sp)
    annotation (Line(points={{48,106},{24,106}}, color={0,0,255}));
  connect(switch.plug_n, SG_with_AVR.plugSupply) annotation (Line(points={{68,106},
          {84,106},{84,106.06},{98,106.06}},            color={0,0,255}));
  connect(booleanReplicator.u, booleanStep.y)
    annotation (Line(points={{28,126},{21,126}}, color={255,0,255}));
  connect(booleanReplicator.y, switch.control) annotation (Line(points={{51,126},
          {54,126},{54,118},{58,118}},      color={255,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,40},{240,
            140}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,40},{
            240,140}})),
    experiment(
      StopTime=100,
      __Dymola_NumberOfIntervals=50000,
      Tolerance=1e-05,
      __Dymola_fixedstepsize=0.001,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>Synchronous generator with induction generator</p>
</html>"));
end SG_IG;
