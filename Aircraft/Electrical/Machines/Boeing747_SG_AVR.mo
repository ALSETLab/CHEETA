within CHEETA.Aircraft.Electrical.Machines;
model Boeing747_SG_AVR
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
        origin={-21,-33},
        extent={{-7,-7},{7,7}},
        rotation=0)));
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
  Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=0,
        origin={24,26})));
  Modelica.Electrical.MultiPhase.Blocks.QuasiRMS rms annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-24,36})));
  Controls.AVR.IEEEtype1AVR
                        iEEEtype1AVR(
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
        extent={{-12,-4},{12,4}},
        rotation=0,
        origin={-58,-14})));
  Modelica.Blocks.Math.Gain PerUnitConversion(k=1/Data.VsNominal) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-56,36})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=270,
        origin={-26,-14})));
  Modelica.Blocks.Interfaces.RealInput w_ref1
    "Reference angular velocity of flange with respect to support as input signal"
    annotation (Placement(transformation(extent={{140,-40},{100,0}}),
        iconTransformation(extent={{140,-40},{100,0}})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plugSupply1
          "To grid" annotation (Placement(transformation(extent={{-10,54},{10,
            74}}), iconTransformation(extent={{-10,54},{10,74}})));
  parameter Records.Boeing747electricalModel.SynchronousMachine.SM100kVA Data
    annotation (Placement(transformation(extent={{52,22},{72,44}})));
equation
  connect(rotorDisplacementAngle.plug_n,smee. plug_sn) annotation (Line(
        points={{30,-4},{30,8},{-12,8},{-12,-4}},       color={0,0,255}));
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
  connect(voltageSensor.plug_p, rotorDisplacementAngle.plug_p)
    annotation (Line(points={{18,26},{18,-4}},         color={0,0,255}));
  connect(voltageSensor.plug_n, rotorDisplacementAngle.plug_n)
    annotation (Line(points={{30,26},{30,-4}},         color={0,0,255}));
  connect(rms.u, voltageSensor.v)
    annotation (Line(points={{-12,36},{24,36},{24,32.6}},
                                                        color={0,0,127}));
  connect(PerUnitConversion.u, rms.y)
    annotation (Line(points={{-44,36},{-35,36}}, color={0,0,127}));
  connect(PerUnitConversion.y, iEEEtype1AVR.Vterm) annotation (Line(points={{-67,36},
          {-78,36},{-78,-13.92},{-70.48,-13.92}},    color={0,0,127}));
  connect(signalVoltage.p, smee.pin_ep) annotation (Line(points={{-26,-8},{-16,
          -8}},               color={0,0,255}));
  connect(signalVoltage.n, smee.pin_en) annotation (Line(points={{-26,-20},{-16,
          -20}},                     color={0,0,255}));
  connect(signalVoltage.v, iEEEtype1AVR.Ifd) annotation (Line(points={{-33.2,
          -14},{-38,-14},{-38,-13.88},{-44.48,-13.88}},
                                                color={0,0,127}));
  connect(groundExcitation.p, smee.pin_en)
    annotation (Line(points={{-21,-26},{-21,-20},{-16,-20}}, color={0,0,255}));
  connect(speed.w_ref, w_ref1) annotation (Line(points={{92,-14},{100,-14},{100,
          -20},{120,-20}}, color={0,0,127}));
  connect(terminalBox.plugSupply, plugSupply1)
    annotation (Line(points={{-6,-2},{0,-2},{0,64}}, color={0,0,255}));
  connect(plugSupply1, plugSupply1)
    annotation (Line(points={{0,64},{0,64}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,60}}),
        graphics={                                                Rectangle(
            extent={{-100,60},{100,-40}}, lineColor={28,108,200}), Text(
          extent={{-68,48},{74,-26}},
          lineColor={28,108,200},
          textString="AC Synchronous Generator")}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,
            60}})),
    experiment(
      StopTime=36000,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-06));
end Boeing747_SG_AVR;
