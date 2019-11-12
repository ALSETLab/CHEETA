within CHEETA.Aircraft.Electrical.Machines;
model SG_747 "Synchronous generator used in Boeing 747 electrical system"
  parameter Records.Boeing747electricalModel.SM100kVA
                                                Data(
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
    annotation (Placement(transformation(extent={{60,22},{80,42}})));
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
        origin={26,-14},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
      Placement(transformation(
        origin={-26,-32},
        extent={{-6,-6},{6,6}},
        rotation=0)));
  Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor
    mechanicalPowerSensor
    annotation (Placement(transformation(extent={{44,-24},{64,-4}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-10,-8},{10,12}})));
  Modelica.Mechanics.Rotational.Sources.Speed speed annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={80,-14})));
  Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=0,
        origin={26,18})));
  Modelica.Electrical.MultiPhase.Blocks.QuasiRMS rms annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-24,34})));
  Controls.IEEEtype1AVR iEEEtype1AVR(
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
        extent={{-9,-4},{9,4}},
        rotation=0,
        origin={-53,-14})));
  Modelica.Blocks.Math.Gain PerUnitConversion(k=1/Data.VsNominal) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-58,34})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=270,
        origin={-26,-14})));
  Modelica.Blocks.Interfaces.RealInput w_ref(unit="rad/s") annotation (
      Placement(transformation(rotation=0, extent={{120,-24},{100,-4}})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plugSupply
    annotation (Placement(transformation(rotation=0, extent={{-10,54},{10,74}})));
equation
  connect(rotorDisplacementAngle.plug_n,smee. plug_sn) annotation (Line(
        points={{32,-4},{32,6},{-6,6},{-6,-4}},         color={0,0,255}));
  connect(rotorDisplacementAngle.plug_p,smee. plug_sp)
    annotation (Line(points={{20,-4},{6,-4}},    color={0,0,255}));
  connect(terminalBox.plug_sn,smee. plug_sn) annotation (Line(
      points={{-6,-4},{-6,-4}},
      color={0,0,255}));
  connect(terminalBox.plug_sp,smee. plug_sp) annotation (Line(
      points={{6,-4},{6,-4}},
      color={0,0,255}));
  connect(smee.flange,rotorDisplacementAngle. flange) annotation (Line(
      points={{10,-14},{16,-14}}));
  connect(smee.flange,mechanicalPowerSensor. flange_a) annotation (Line(
      points={{10,-14},{44,-14}}));
  connect(speed.flange,mechanicalPowerSensor. flange_b)
    annotation (Line(points={{70,-14},{64,-14}}, color={0,0,0}));
  connect(voltageSensor.plug_p,rotorDisplacementAngle. plug_p)
    annotation (Line(points={{20,18},{20,-4}},         color={0,0,255}));
  connect(voltageSensor.plug_n,rotorDisplacementAngle. plug_n)
    annotation (Line(points={{32,18},{32,-4}},         color={0,0,255}));
  connect(rms.u,voltageSensor. v)
    annotation (Line(points={{-12,34},{26,34},{26,24.6}},
                                                        color={0,0,127}));
  connect(PerUnitConversion.u,rms. y)
    annotation (Line(points={{-46,34},{-35,34}}, color={0,0,127}));
  connect(PerUnitConversion.y,iEEEtype1AVR. Vterm) annotation (Line(points={{-69,34},
          {-74,34},{-74,-13.8667},{-63.2,-13.8667}}, color={0,0,127}));
  connect(signalVoltage.p,smee. pin_ep) annotation (Line(points={{-26,-8},{-10,
          -8}},               color={0,0,255}));
  connect(signalVoltage.n,smee. pin_en) annotation (Line(points={{-26,-20},{-10,
          -20}},                     color={0,0,255}));
  connect(signalVoltage.v,iEEEtype1AVR. Ifd) annotation (Line(points={{-33.2,
          -14},{-33.2,-13.8},{-42.86,-13.8}},   color={0,0,127}));
  connect(groundExcitation.p,signalVoltage. n) annotation (Line(points={{-26,-26},
          {-26,-20}},                     color={0,0,255}));
  connect(w_ref,speed. w_ref)
    annotation (Line(points={{110,-14},{92,-14}}, color={0,0,127}));
  connect(plugSupply,terminalBox. plugSupply) annotation (Line(points={{0,64},{
          0,-2}},                  color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-40},{100,60}})),  Icon(
        coordinateSystem(extent={{-100,-40},{100,60}}), graphics={Rectangle(
            extent={{-100,60},{100,-40}}, lineColor={28,108,200}), Text(
          extent={{-68,48},{74,-26}},
          lineColor={28,108,200},
          textString="AC Synchronous Generator")}));
end SG_747;
