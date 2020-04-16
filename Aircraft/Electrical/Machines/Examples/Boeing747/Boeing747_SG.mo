within CHEETA.Aircraft.Electrical.Machines.Examples.Boeing747;
model Boeing747_SG "Synchronous generator used in Boeing 747 electrical system"
  import CHEETA;
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
  CHEETA.Aircraft.Controls.AVR.IEEEtype1AVR iEEEtype1AVR(
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
        extent={{-7,-4},{7,4}},
        rotation=0,
        origin={-61,-14})));
  Modelica.Blocks.Math.Gain PerUnitConversion(k=1/Data.VsNominal) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-58,34})));
  Modelica.Blocks.Interfaces.RealInput w_ref(unit="rad/s") annotation (
      Placement(transformation(rotation=0, extent={{120,30},{100,50}}),
        iconTransformation(extent={{120,30},{100,50}})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plugSupply
    annotation (Placement(transformation(rotation=0, extent={{-110,28},{-90,48}}),
        iconTransformation(extent={{-110,28},{-90,48}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=270,
        origin={-26,-14})));
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
    annotation (Placement(transformation(extent={{-10,-24},{10,-4}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{-10,-8},{10,12}})));
  Modelica.Blocks.Math.Gain PerUnitConversion1(k=Data.VsNominal)  annotation (
     Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-44,-14})));
  parameter CHEETA.Records.Boeing747electricalModel.SynchronousMachine.SM300kVA
                                                                         Data
    annotation (Placement(transformation(extent={{60,70},{80,90}})));
equation
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
          {-76,34},{-76,-14},{-68.9333,-14}},        color={0,0,127}));
  connect(w_ref,speed. w_ref)
    annotation (Line(points={{110,40},{102,40},{102,-14},{92,-14}},
                                                  color={0,0,127}));
  connect(smee.flange, rotorDisplacementAngle.flange)
    annotation (Line(points={{10,-14},{16,-14}}, color={0,0,0}));
  connect(terminalBox.plugSupply, plugSupply)
    annotation (Line(points={{0,-2},{0,38},{-100,38}},
                                             color={0,0,255}));
  connect(smee.pin_en, signalVoltage.n) annotation (Line(points={{-10,-20},{-18,
          -20},{-18,-20},{-26,-20}}, color={0,0,255}));
  connect(smee.pin_ep, signalVoltage.p)
    annotation (Line(points={{-10,-8},{-26,-8}}, color={0,0,255}));
  connect(groundExcitation.p, signalVoltage.n)
    annotation (Line(points={{-26,-26},{-26,-20}}, color={0,0,255}));
  connect(rotorDisplacementAngle.plug_p, terminalBox.plug_sp)
    annotation (Line(points={{20,-4},{6,-4}}, color={0,0,255}));
  connect(terminalBox.plug_sn, rotorDisplacementAngle.plug_n)
    annotation (Line(points={{-6,-4},{-6,6},{32,6},{32,-4}}, color={0,0,255}));
  connect(terminalBox.plug_sn, smee.plug_sn)
    annotation (Line(points={{-6,-4},{-6,-4}}, color={0,0,255}));
  connect(terminalBox.plug_sp, smee.plug_sp)
    annotation (Line(points={{6,-4},{6,-4}}, color={0,0,255}));
  connect(mechanicalPowerSensor.flange_a, rotorDisplacementAngle.flange)
    annotation (Line(points={{44,-14},{30,-14},{30,-14},{16,-14}}, color={0,0,0}));
  connect(signalVoltage.v, PerUnitConversion1.y)
    annotation (Line(points={{-33.2,-14},{-39.6,-14}}, color={0,0,127}));
  connect(PerUnitConversion1.u, iEEEtype1AVR.Ifd)
    annotation (Line(points={{-48.8,-14},{-50,-14},{-50,-14.04},{-53.1133,
          -14.04}},                                       color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-100,-80},{100,120}})), Icon(
        coordinateSystem(extent={{-100,-80},{100,120}}), graphics={
        Rectangle(
          extent={{-52,102},{68,-18}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,128,255}),
        Rectangle(
          extent={{-52,102},{-72,-18}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={128,128,128}),
        Rectangle(
          extent={{-52,112},{28,92}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-62,-48},{-52,-48},{-22,22},{28,22},{58,-48},{68,-48},
              {68,-58},{-62,-58},{-62,-48}},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-162,-78},{138,-118}},
          lineColor={0,0,255},
          textString="%name"),
        Text(
          extent={{-102,158},{100,122}},
          lineColor={0,0,0},
          lineThickness=1,
          textString="%name
",        fontSize=16)}),
    experiment(
      StopTime=50,
      Interval=0.0001,
      Tolerance=1e-08,
      __Dymola_fixedstepsize=1e-07,
      __Dymola_Algorithm="Dassl"));
end Boeing747_SG;
