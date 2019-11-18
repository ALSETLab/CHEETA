within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_to_secondary_distribution
  "Synchronous generator and induction generator"
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-80,72})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=Modelica.Constants.pi/30)
                                                annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-48,72})));
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
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={44,30})));

  Aircraft.Electrical.Machines.Boeing747_SG SGandAVR annotation (Placement(
        transformation(rotation=0, extent={{8,76},{-24,60}})));
  Aircraft.Mechanical.Pumps.Boeing747_Pump_Load Pumped_Load
    "Pumped load for 12kVA induction motor of an AC hydraulic pump" annotation (
     Placement(transformation(
        rotation=180,
        extent={{-10,-4},{10,4}},
        origin={44,8})));
  Modelica.Electrical.MultiPhase.Basic.Star star(final m=3) annotation (
      Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=90,
        origin={74,30})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        origin={74,2},
        extent={{10,-10},{-10,10}},
        rotation=0)));
  Modelica.Electrical.MultiPhase.Basic.Resistor Lamp(m=3, R={1000/(200^2),1000/
        (200^2),1000/(200^2)})
    "AC 1kW load to power a resistive lamp"
    annotation (Placement(transformation(extent={{28,-24},{48,-4}})));
  Modelica.Electrical.MultiPhase.Basic.Star star1
    annotation (Placement(transformation(extent={{62,-24},{82,-4}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={100,-14})));
equation
  connect(SGandAVR.plugSupply, AC_Hydraulic_Pump.plug_sp)
    annotation (Line(points={{-8,59.36},{-8,40},{38,40}},
                                             color={0,0,255}));
  connect(star.pin_n, ground1.p)
    annotation (Line(points={{74,20},{74,12}},   color={0,0,255}));
  connect(AC_Hydraulic_Pump.plug_sn, star.plug_p)
    annotation (Line(points={{50,40},{74,40}},        color={0,0,255}));
  connect(Pumped_Load.flange, AC_Hydraulic_Pump.flange)
    annotation (Line(points={{33,8},{26,8},{26,30},{34,30}}, color={0,0,0}));
  connect(SGandAVR.w_ref, RPMtoRPS.y) annotation (Line(points={{-25.6,71.84},{
          -34,71.84},{-34,72},{-37,72}}, color={0,0,127}));
  connect(RPMtoRPS.u, timeTable.y)
    annotation (Line(points={{-60,72},{-69,72}}, color={0,0,127}));
  connect(Lamp.plug_p, AC_Hydraulic_Pump.plug_sp) annotation (Line(points={{28,
          -14},{-8,-14},{-8,40},{38,40}}, color={0,0,255}));
  connect(Lamp.plug_n, star1.plug_p)
    annotation (Line(points={{48,-14},{62,-14}}, color={0,0,255}));
  connect(star1.pin_n, ground.p)
    annotation (Line(points={{82,-14},{90,-14}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{120,
            100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            120,100}})),
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
end SG_to_secondary_distribution;
