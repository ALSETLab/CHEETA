within CHEETA.Aircraft.Electrical.Machines;
model SG_747 "Synchronous generator used in Boeing 747 electrical system"
  parameter Records.Boeing747electricalModel.SM100kVA               Data(
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
    annotation (Placement(transformation(extent={{60,30},{80,50}})));
  Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited
    smee(
    fsNominal=Data.fsNominal,
    Rs=Data.Rs,
    TsRef=Data.TsRef,
    Lssigma=Data.Lssigma,
    phiMechanical(start=0),
    wMechanical(start=0),
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
    TrOperational=293.15,
    alpha20r=Data.alpha20r,
    alpha20e=Data.alpha20e,
    TeOperational=293.15)
    annotation (Placement(transformation(extent={{-10,-24},{10,-4}})));
  Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle
    rotorDisplacementAngle(p=2) annotation (Placement(transformation(
        origin={28,6},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
      Placement(transformation(
        origin={-25,-29},
        extent={{-5,-5},{5,5}},
        rotation=0)));
  Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor
    mechanicalPowerSensor
    annotation (Placement(transformation(extent={{40,-24},{60,-4}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-10,-2},{10,18}})));
  Modelica.Mechanics.Rotational.Sources.Speed speed annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={78,-14})));
  Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=0,
        origin={28,24})));
  Modelica.Electrical.MultiPhase.Blocks.QuasiRMS RMS(m=3)
    "RMS calculation for AVR input"                  annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-24,36})));
  Controls.IEEEtype1AVR                     iEEEtype1AVR(
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
        extent={{-9,-6},{9,6}},
        rotation=0,
        origin={-57,-14})));
  Modelica.Blocks.Math.Gain PerUnitConversion(k=1/Data.VsNominal) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-56,36})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=270,
        origin={-32,-14})));
  Modelica.Blocks.Interfaces.RealInput w_ref(unit="rad/s") annotation (
      Placement(transformation(rotation=180,
                                           extent={{-10,-10},{10,10}},
        origin={110,-14})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plugSupply
    annotation (Placement(transformation(rotation=0, extent={{-10,60},{10,80}}),
        iconTransformation(extent={{-10,60},{10,80}})));
equation
  connect(terminalBox.plug_sn,smee. plug_sn) annotation (Line(
      points={{-6,2},{-6,-4}},
      color={0,0,255}));
  connect(terminalBox.plug_sp,smee. plug_sp) annotation (Line(
      points={{6,2},{6,-4}},
      color={0,0,255}));
  connect(smee.flange,rotorDisplacementAngle. flange) annotation (Line(
      points={{10,-14},{14,-14},{14,6},{18,6}}));
  connect(smee.flange,mechanicalPowerSensor. flange_a) annotation (Line(
      points={{10,-14},{40,-14}}));
  connect(speed.flange,mechanicalPowerSensor. flange_b)
    annotation (Line(points={{68,-14},{60,-14}}, color={0,0,0}));
  connect(voltageSensor.plug_p,rotorDisplacementAngle. plug_p)
    annotation (Line(points={{22,24},{22,16}},         color={0,0,255}));
  connect(voltageSensor.plug_n,rotorDisplacementAngle. plug_n)
    annotation (Line(points={{34,24},{34,16}},         color={0,0,255}));
  connect(RMS.u,voltageSensor. v)
    annotation (Line(points={{-12,36},{28,36},{28,30.6}},
                                                        color={0,0,127}));
  connect(PerUnitConversion.u,RMS. y)
    annotation (Line(points={{-44,36},{-35,36}}, color={0,0,127}));
  connect(PerUnitConversion.y,iEEEtype1AVR. Vterm) annotation (Line(points={{-67,36},
          {-74,36},{-74,-13.8},{-67.2,-13.8}},       color={0,0,127}));
  connect(signalVoltage.p,smee. pin_ep) annotation (Line(points={{-32,-8},{-10,
          -8}},               color={0,0,255}));
  connect(signalVoltage.n,smee. pin_en) annotation (Line(points={{-32,-20},{-10,
          -20}},                     color={0,0,255}));
  connect(groundExcitation.p,signalVoltage. n) annotation (Line(points={{-25,-24},
          {-25,-20},{-32,-20}},           color={0,0,255}));
  connect(w_ref, speed.w_ref)
    annotation (Line(points={{110,-14},{90,-14}}, color={0,0,127}));
  connect(plugSupply, terminalBox.plugSupply) annotation (Line(points={{0,70},{
          0,4}},                   color={0,0,255}));
  connect(signalVoltage.v, iEEEtype1AVR.Ifd) annotation (Line(points={{-39.2,
          -14},{-46,-14},{-46,-13.7},{-46.86,-13.7}},
                                                  color={0,0,127}));
  connect(plugSupply, plugSupply)
    annotation (Line(points={{0,70},{0,70}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-40},{100,60}})),  Icon(
        coordinateSystem(extent={{-100,-40},{100,60}}), graphics={Rectangle(
            extent={{-100,60},{100,-40}}, lineColor={28,108,200}), Text(
          extent={{-68,48},{74,-26}},
          lineColor={28,108,200},
          textString="AC Synchronous Generator")}));
end SG_747;
