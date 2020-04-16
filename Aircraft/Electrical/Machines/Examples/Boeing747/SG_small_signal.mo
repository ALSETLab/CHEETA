within CHEETA.Aircraft.Electrical.Machines.Examples.Boeing747;
model SG_small_signal
  import CHEETA;
  Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited
    smee(
    fsNominal=Data.fsNominal,
    Rs=Data.Rs,
    TsRef=Data.TsRef,
    Lssigma=Data.Lssigma,
    phiMechanical(fixed=false),
    wMechanical(fixed=false),
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
    p=Data.Poles,
    Jr=0.02,
    Js=0.29,
    useDamperCage=true,
    statorCoreParameters(VRef=115),
    strayLoadParameters(IRef=100),
    brushParameters(ILinear=0.01),
    TsOperational=293.15,
    alpha20s=Data.alpha20s,
    TrOperational=293.15,
    alpha20r=Data.alpha20r,
    alpha20e(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    TeOperational=293.15)
    annotation (Placement(transformation(extent={{-14,-24},{6,-4}})));

  Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle
    rotorDisplacementAngle(p=Data.Poles)
                                annotation (Placement(transformation(
        origin={30,-14},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
      Placement(transformation(
        origin={-86,-50},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor
    mechanicalPowerSensor
    annotation (Placement(transformation(extent={{44,-24},{64,-4}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-14,-8},{6,12}})));
  Modelica.Mechanics.Rotational.Sources.Speed speed(exact=false)
                                                    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={80,-14})));
  Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={26,46})));
  Modelica.Electrical.MultiPhase.Blocks.QuasiRMS rms annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-24,36})));
  CHEETA.Aircraft.Controls.AVR.IEEEtype1AVR iEEEtype1AVR(
    T_R=2e-3,
    T_C=0.001,
    T_B=0.001,
    K_A=50,
    T_A=0.001,
    K_E=1,
    T_E=0.001,
    K_F=0.001,
    T_F=0.1,
    Vmax=2,
    Vmin=-2,
    Vref=1) annotation (Placement(transformation(
        extent={{-9.5,-6.5},{9.5,6.5}},
        rotation=270,
        origin={-83.5,8.5})));
  Modelica.Blocks.Math.Gain PerUnitConversion(k=1/Data.VsNominal) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-56,36})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=270,
        origin={-38,-14})));
  Modelica.Blocks.Interfaces.RealInput w_ref(unit="rad/s") annotation (
      Placement(transformation(rotation=0, extent={{100,30},{120,50}}),
        iconTransformation(extent={{100,30},{120,50}})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plugSupply
    annotation (Placement(transformation(rotation=0, extent={{-112,32},{
            -92,52}})));
  Modelica.Blocks.Math.Gain PerUnitConversion1(k=Data.VsNominal)  annotation (
     Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={-60,-14})));

  parameter CHEETA.Records.Boeing747electricalModel.SynchronousMachine.SM300kVA
                                                                         Data
    annotation (Placement(transformation(extent={{26,-64},{46,-44}})));
  Modelica.Blocks.Interfaces.RealOutput dV annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-84,130}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-80,130})));
  Modelica.Blocks.Interfaces.RealOutput ddelta annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-12,130}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-24,130})));
  Modelica.Blocks.Interfaces.RealOutput dEfd annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={94,130}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={80,130})));
  Modelica.Blocks.Interfaces.RealOutput dpm annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={44,130}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={26,130})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=iEEEtype1AVR.Ifd)
    annotation (Placement(transformation(extent={{72,100},{92,120}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=
        mechanicalPowerSensor.P)
    annotation (Placement(transformation(extent={{20,100},{40,120}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=
        rotorDisplacementAngle.rotorDisplacementAngle)
    annotation (Placement(transformation(extent={{-38,98},{-18,118}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=PerUnitConversion.y)
    annotation (Placement(transformation(extent={{-54,98},{-74,118}})));
equation
  connect(rotorDisplacementAngle.plug_n,smee. plug_sn) annotation (Line(
        points={{36,-4},{36,6},{-10,6},{-10,-4}},       color={0,0,255}));
  connect(rotorDisplacementAngle.plug_p,smee. plug_sp)
    annotation (Line(points={{24,-4},{2,-4}},    color={0,0,255}));
  connect(terminalBox.plug_sn,smee. plug_sn) annotation (Line(
      points={{-10,-4},{-10,-4}},
      color={0,0,255}));
  connect(terminalBox.plug_sp,smee. plug_sp) annotation (Line(
      points={{2,-4},{2,-4}},
      color={0,0,255}));
  connect(smee.flange,rotorDisplacementAngle. flange) annotation (Line(
      points={{6,-14},{20,-14}}));
  connect(smee.flange,mechanicalPowerSensor. flange_a) annotation (Line(
      points={{6,-14},{44,-14}}));
  connect(speed.flange,mechanicalPowerSensor. flange_b)
    annotation (Line(points={{70,-14},{64,-14}}, color={0,0,0}));
  connect(voltageSensor.plug_p,rotorDisplacementAngle. plug_p)
    annotation (Line(points={{16,46},{16,-4},{24,-4}}, color={0,0,255}));
  connect(voltageSensor.plug_n,rotorDisplacementAngle. plug_n)
    annotation (Line(points={{36,46},{36,-4}},         color={0,0,255}));
  connect(rms.u,voltageSensor. v)
    annotation (Line(points={{-12,36},{26,36},{26,35}}, color={0,0,127}));
  connect(PerUnitConversion.u,rms. y)
    annotation (Line(points={{-44,36},{-35,36}}, color={0,0,127}));
  connect(PerUnitConversion.y,iEEEtype1AVR. Vterm) annotation (Line(points={{-67,36},
          {-84,36},{-84,19.2667},{-83.5,19.2667}},   color={0,0,127}));
  connect(signalVoltage.p,smee. pin_ep) annotation (Line(points={{-38,-8},{-30,
          -8},{-30,-8},{-14,-8}},
                              color={0,0,255}));
  connect(signalVoltage.n,smee. pin_en) annotation (Line(points={{-38,-20},{-24,
          -20},{-24,-20},{-14,-20}}, color={0,0,255}));
  connect(groundExcitation.p,signalVoltage. n) annotation (Line(points={{-76,-50},
          {-38,-50},{-38,-20}},           color={0,0,255}));
  connect(w_ref, speed.w_ref)
    annotation (Line(points={{110,40},{96,40},{96,-14},{92,-14}},
                                                  color={0,0,127}));
  connect(plugSupply, terminalBox.plugSupply) annotation (Line(points={{-102,42},
          {-102,54},{-4,54},{-4,-2}},
                                   color={0,0,255}));
  connect(PerUnitConversion1.u, iEEEtype1AVR.Ifd) annotation (Line(points={{-69.6,
          -14},{-84,-14},{-84,-2.20333},{-83.565,-2.20333}},
                                                          color={0,0,127}));
  connect(PerUnitConversion1.y, signalVoltage.v) annotation (Line(points={{-51.2,
          -14},{-45.2,-14}},                         color={0,0,127}));
  connect(realExpression.y, dEfd)
    annotation (Line(points={{93,110},{94,110},{94,130}}, color={0,0,127}));
  connect(realExpression1.y, dpm) annotation (Line(points={{41,110},{44,110},{
          44,130},{44,130}}, color={0,0,127}));
  connect(realExpression2.y, ddelta)
    annotation (Line(points={{-17,108},{-12,108},{-12,130}}, color={0,0,127}));
  connect(dV, realExpression3.y)
    annotation (Line(points={{-84,130},{-84,108},{-75,108}}, color={0,0,127}));
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
end SG_small_signal;
