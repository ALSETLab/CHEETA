within CHEETA.Examples.Boeing747ElectricalSystem;
model TestSG
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
    p=2,
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
    annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
  Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle
                                          rotorDisplacementAngle(p=2)
    annotation (Placement(transformation(
        origin={20,-40},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
      Placement(transformation(
        origin={-90,-54},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(final
      w_fixed=1256)          annotation (Placement(transformation(extent=
            {{90,-50},{70,-30}})));
  Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor
                                         mechanicalPowerSensor
    annotation (Placement(transformation(extent={{40,-50},{60,-30}})));
  Modelica.Electrical.Machines.Sensors.ElectricalPowerSensor
                                         electricalPowerSensor
    annotation (Placement(transformation(
        origin={0,60},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor
                                         currentQuasiRMSSensor
    annotation (Placement(transformation(
        origin={0,30},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
    final m=3,
    final V=fill(Data.VsNominal*sqrt(2), 3),
    final freqHz=fill(Data.fsNominal, 3))
                                    annotation (Placement(transformation(
          extent={{-20,80},{-40,100}})));
  Modelica.Electrical.MultiPhase.Basic.Star star(final m=3) annotation (
      Placement(transformation(extent={{-50,80},{-70,100}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={-90,90},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    duration=0.1,
    I=0,
    offset=2)   annotation (Placement(transformation(
        origin={-50,-44},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{-20,-34},{0,-14}})));
  parameter AircraftPowerSystem.Records.SynchronousMachine.SM100kVA Data(
    SNominal=100000,
    VsNominal=115,
    fsNominal=60,
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
    annotation (Placement(transformation(extent={{-18,-86},{2,-66}})));
equation
  connect(rotorDisplacementAngle.plug_n,smee. plug_sn) annotation (Line(
        points={{26,-30},{26,-20},{-16,-20},{-16,-30}}, color={0,0,255}));
  connect(rotorDisplacementAngle.plug_p,smee. plug_sp)
    annotation (Line(points={{14,-30},{-4,-30}}, color={0,0,255}));
  connect(star.pin_n,ground. p)
    annotation (Line(points={{-70,90},{-80,90}}, color={0,0,255}));
  connect(star.plug_p,sineVoltage. plug_n)
    annotation (Line(points={{-50,90},{-40,90}}, color={0,0,255}));
  connect(electricalPowerSensor.plug_ni,currentQuasiRMSSensor. plug_p)
    annotation (Line(points={{0,50},{0,46},{0,40}}, color={0,0,255}));
  connect(mechanicalPowerSensor.flange_b,constantSpeed. flange)
    annotation (Line(points={{60,-40},{70,-40}}));
  connect(sineVoltage.plug_p,electricalPowerSensor. plug_p)
    annotation (Line(points={{-20,90},{0,90},{0,70}}, color={0,0,255}));
  connect(rampCurrent.p,groundExcitation. p)
    annotation (Line(points={{-50,-54},{-80,-54}}, color={0,0,255}));
  connect(rampCurrent.p,smee. pin_en) annotation (Line(points={{-50,-54},{-40,
          -54},{-40,-46},{-20,-46}},      color={0,0,255}));
  connect(rampCurrent.n,smee. pin_ep) annotation (Line(points={{-50,-34},{-20,
          -34}},                          color={0,0,255}));
  connect(electricalPowerSensor.plug_nv,smee. plug_sn) annotation (Line(
        points={{-10,60},{-16,60},{-16,-30}}, color={0,0,255}));
  connect(terminalBox.plugSupply,currentQuasiRMSSensor. plug_n)
    annotation (Line(
      points={{-10,-28},{-10,0},{0,0},{0,20}},
      color={0,0,255}));
  connect(terminalBox.plug_sn,smee. plug_sn) annotation (Line(
      points={{-16,-30},{-16,-30}},
      color={0,0,255}));
  connect(terminalBox.plug_sp,smee. plug_sp) annotation (Line(
      points={{-4,-30},{-4,-30}},
      color={0,0,255}));
  connect(smee.flange,rotorDisplacementAngle. flange) annotation (Line(
      points={{0,-40},{10,-40}}));
  connect(smee.flange,mechanicalPowerSensor. flange_a) annotation (Line(
      points={{0,-40},{40,-40}}));
  annotation (experiment(__Dymola_NumberOfIntervals=50000));
end TestSG;
