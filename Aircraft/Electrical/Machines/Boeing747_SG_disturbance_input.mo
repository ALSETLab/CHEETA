within CHEETA.Aircraft.Electrical.Machines;
model Boeing747_SG_disturbance_input
  "Synchronous generator used in Boeing 747 electrical system"
  Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_ElectricalExcited
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
  Modelica.Electrical.Polyphase.Sensors.VoltageSensor voltageSensor annotation (
     Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=0,
        origin={26,18})));
  Modelica.Electrical.Polyphase.Blocks.QuasiRMS rms annotation (Placement(
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
      Placement(transformation(rotation=0, extent={{-124,-32},{-100,-8}})));
  Modelica.Electrical.Polyphase.Interfaces.PositivePlug plugSupply annotation (
      Placement(transformation(rotation=0, extent={{-10,54},{10,74}})));
  Modelica.Blocks.Math.Add add annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-81,7})));
  Modelica.Blocks.Interfaces.RealInput Vd "Disturbance input"
    annotation (Placement(transformation(extent={{-124,28},{-100,52}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,0},{120,20}})));
  parameter CHEETA.Records.Boeing747electricalModel.SynchronousMachine.SM100kVA
                                                                         Data
    annotation (Placement(transformation(extent={{60,28},{80,50}})));
equation
  connect(rotorDisplacementAngle.plug_n,smee. plug_sn) annotation (Line(
        points={{32,-4},{32,4},{-6,4},{-6,-4}},         color={0,0,255}));
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
  connect(signalVoltage.p,smee. pin_ep) annotation (Line(points={{-26,-8},{-10,
          -8}},               color={0,0,255}));
  connect(signalVoltage.n,smee. pin_en) annotation (Line(points={{-26,-20},{-10,
          -20}},                     color={0,0,255}));
  connect(signalVoltage.v,iEEEtype1AVR. Ifd) annotation (Line(points={{-33.2,
          -14},{-33.2,-14.04},{-42.86,-14.04}}, color={0,0,127}));
  connect(groundExcitation.p,signalVoltage. n) annotation (Line(points={{-26,-26},
          {-26,-20}},                     color={0,0,255}));
  connect(w_ref,speed. w_ref)
    annotation (Line(points={{-112,-20},{-88,-20},{-88,-36},{98,-36},{98,-14},{
          92,-14}},                               color={0,0,127}));
  connect(plugSupply,terminalBox. plugSupply) annotation (Line(points={{0,64},{
          0,-2}},                  color={0,0,255}));
  connect(add.y, iEEEtype1AVR.Vterm) annotation (Line(points={{-81,-0.7},{-81,
          -14},{-63.2,-14}}, color={0,0,127}));
  connect(PerUnitConversion.y, add.u1) annotation (Line(points={{-69,34},{-76.8,
          34},{-76.8,15.4}}, color={0,0,127}));
  connect(Vd, add.u2) annotation (Line(points={{-112,40},{-92,40},{-92,20},{
          -85.2,20},{-85.2,15.4}}, color={0,0,127}));
  connect(y, voltageSensor.v[1]) annotation (Line(points={{110,10},{52,10},{52,
          34},{26,34},{26,24.6}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-100,-40},{100,60}})),  Icon(
        coordinateSystem(extent={{-100,-40},{100,60}}), graphics={Rectangle(
            extent={{-100,60},{100,-40}}, lineColor={28,108,200}), Text(
          extent={{-68,48},{74,-26}},
          lineColor={28,108,200},
          textString="AC Synchronous Generator")}));
end Boeing747_SG_disturbance_input;
