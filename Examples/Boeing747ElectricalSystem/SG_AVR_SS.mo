within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_AVR_SS
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
    annotation (Placement(transformation(extent={{-8,-78},{12,-58}})));
  Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited
    smee(
    fsNominal=Data.fsNominal,
    Rs=Data.Rs,
    TsRef=Data.TsRef,
    Lssigma=Data.Lssigma,
    Lmd=Data.Lmd,
    Lmq=Data.Lmq,
    Lrsigmad=Data.Lrsigmad,
    Lrsigmaq=Data.Lrsigmaq,
    Rrd=Data.Rrd,
    Rrq=Data.Rrq,
    TrRef=Data.TrRef,
    VsNominal=Data.VsNominal,
    IeOpenCircuit=Data.IeOpenCircuit,
    Re=Data.Re,
    TeRef=Data.TeRef,
    sigmae=Data.sigmae,
    p=4,
    Jr=0.29,
    Js=0.29,
    useDamperCage=true,
    statorCoreParameters(VRef=115),
    strayLoadParameters(IRef=100),
    brushParameters(ILinear=0.01),
    TsOperational=293.15,
    alpha20s=Data.alpha20s,
    ir(each fixed=true),
    TrOperational=293.15,
    alpha20r=Data.alpha20r,
    alpha20e=Data.alpha20e,
    TeOperational=293.15)
    annotation (Placement(transformation(extent={{-16,-24},{4,-4}})));
  Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle
                                          rotorDisplacementAngle(p=4)
    annotation (Placement(transformation(
        origin={24,-14},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
      Placement(transformation(
        origin={-86,-28},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor
                                         mechanicalPowerSensor
    annotation (Placement(transformation(extent={{44,-24},{64,-4}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{-16,-8},{4,12}})));
  Modelica.Mechanics.Rotational.Sources.Speed speed annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={80,-14})));
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={146,-14})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={114,-14})));
  Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R={100,100,100})
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-20,82})));
  Modelica.Electrical.MultiPhase.Basic.Star star annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-46,82})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={-74,82},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={18,72})));
  Modelica.Electrical.MultiPhase.Blocks.QuasiRMS rms annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-22,52})));
  AircraftPowerSystem.Controls.IEEEtype1AVR iEEEtype1AVR(
    T_R=2e-3,
    T_C=0.001,
    T_B=0.001,
    K_A=300,
    T_A=0.001,
    K_E=1,
    T_E=0.001,
    K_F=0.001,
    T_F=0.1,
    Vmax=7,
    Vmin=-2,
    Vref=1) annotation (Placement(transformation(
        extent={{-15,-10},{15,10}},
        rotation=270,
        origin={-75,4})));
  Modelica.Blocks.Math.Gain PerUnitConversion(k=1/Data.VsNominal) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-56,52})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-42,-14})));
  Modelica.Blocks.Interfaces.RealInput u
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Math.Add add annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-74,32})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{160,-10},{180,10}})));
equation
  connect(rotorDisplacementAngle.plug_n,smee. plug_sn) annotation (Line(
        points={{30,-4},{30,6},{-12,6},{-12,-4}},       color={0,0,255}));
  connect(rotorDisplacementAngle.plug_p,smee. plug_sp)
    annotation (Line(points={{18,-4},{0,-4}},    color={0,0,255}));
  connect(terminalBox.plug_sn,smee. plug_sn) annotation (Line(
      points={{-12,-4},{-12,-4}},
      color={0,0,255}));
  connect(terminalBox.plug_sp,smee. plug_sp) annotation (Line(
      points={{0,-4},{0,-4}},
      color={0,0,255}));
  connect(smee.flange,rotorDisplacementAngle. flange) annotation (Line(
      points={{4,-14},{14,-14}}));
  connect(smee.flange,mechanicalPowerSensor. flange_a) annotation (Line(
      points={{4,-14},{44,-14}}));
  connect(speed.flange, mechanicalPowerSensor.flange_b)
    annotation (Line(points={{70,-14},{64,-14}}, color={0,0,0}));
  connect(RPMtoRPS.u, timeTable.y)
    annotation (Line(points={{126,-14},{135,-14}}, color={0,0,127}));
  connect(RPMtoRPS.y, speed.w_ref)
    annotation (Line(points={{103,-14},{92,-14}}, color={0,0,127}));
  connect(ground.p, star.pin_n)
    annotation (Line(points={{-64,82},{-56,82}}, color={0,0,255}));
  connect(resistor.plug_n, star.plug_p)
    annotation (Line(points={{-30,82},{-36,82}}, color={0,0,255}));
  connect(resistor.plug_p, terminalBox.plugSupply)
    annotation (Line(points={{-10,82},{-10,-2},{-6,-2}},
                                                       color={0,0,255}));
  connect(voltageSensor.plug_p, rotorDisplacementAngle.plug_p)
    annotation (Line(points={{8,72},{8,-4},{18,-4}},   color={0,0,255}));
  connect(voltageSensor.plug_n, rotorDisplacementAngle.plug_n)
    annotation (Line(points={{28,72},{28,26},{30,26},{30,-4}},
                                                       color={0,0,255}));
  connect(rms.u, voltageSensor.v)
    annotation (Line(points={{-10,52},{18,52},{18,61}}, color={0,0,127}));
  connect(PerUnitConversion.u, rms.y)
    annotation (Line(points={{-44,52},{-33,52}}, color={0,0,127}));
  connect(signalVoltage.p, smee.pin_ep) annotation (Line(points={{-42,-4},{-30,-4},
          {-30,-8},{-16,-8}}, color={0,0,255}));
  connect(signalVoltage.n, smee.pin_en) annotation (Line(points={{-42,-24},{-30,
          -24},{-30,-20},{-16,-20}}, color={0,0,255}));
  connect(signalVoltage.v, iEEEtype1AVR.Ifd) annotation (Line(points={{-30,-14},
          {-54,-14},{-54,-12.6},{-75,-12.6}},   color={0,0,127}));
  connect(groundExcitation.p, signalVoltage.n) annotation (Line(points={{-76,-28},
          {-60,-28},{-60,-24},{-42,-24}}, color={0,0,255}));
  connect(add.y, iEEEtype1AVR.Vterm) annotation (Line(points={{-74,21},{-78,21},
          {-78,21},{-75,21}},           color={0,0,127}));
  connect(PerUnitConversion.y, add.u1)
    annotation (Line(points={{-67,52},{-68,52},{-68,44}}, color={0,0,127}));
  connect(u, add.u2) annotation (Line(points={{-120,0},{-80,0},{-80,44}},
                    color={0,0,127}));
  connect(y, voltageSensor.v[1]) annotation (Line(points={{170,0},{64,0},{64,50},
          {18,50},{18,61}},     color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{160,
            100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            160,100}})),
    experiment(
      StopTime=36000,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-06),
    Documentation(info="<html>
<p>synchronous generator, avr, synchronous </p>
</html>"));
end SG_AVR_SS;
