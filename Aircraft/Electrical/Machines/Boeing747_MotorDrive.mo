within CHEETA.Aircraft.Electrical.Machines;
model Boeing747_MotorDrive
  "Permanent magnet synchronous motor drive for Boeing 747"
  Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
    smpm annotation (Placement(transformation(extent={{70,-14},{90,6}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{70,2},{90,22}})));
  PowerElectronics.Converters.ThreePhaseRectifier threePhaseRectifier(m=3)
    annotation (Placement(transformation(extent={{-88,-8},{-72,8}})));
  PowerElectronics.Switches.BrakeChopper brakeChopper(
    R=R,
    C=C_DCBus,
    period=period)
    annotation (Placement(transformation(extent={{-34,-6},{-20,6}})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_p1
                                 "Positive polyphase electrical plug with m pins"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-54,0})));
  parameter Modelica.SIunits.Capacitance C "Smoothing capacitor capacitance";
  Modelica.Electrical.PowerConverters.DCAC.MultiPhase2Level inverter1
    annotation (Placement(transformation(extent={{30,12},{50,32}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM[
    m](each useConstantDutyCycle=false, each f=f) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={40,-8})));
  Modelica.Blocks.Sources.Sine sine[m](
    phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
    startTime=zeros(m),
    amplitude=fill(0.5, m),
    offset=fill(0.5, m),
    freqHz=fill(f1, m)) annotation (Placement(transformation(extent={{50,-52},{
            30,-32}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
    annotation (Placement(transformation(extent={{102,-14},{122,6}})));
  Modelica.Blocks.Continuous.Derivative derivative
    annotation (Placement(transformation(extent={{140,-14},{160,6}})));
  Modelica.Blocks.Interfaces.RealInput w_ref "reference speed"
    annotation (Placement(transformation(extent={{-140,-92},{-100,-52}})));
  parameter Modelica.SIunits.Resistance R=10 "Braking chopper resistance"
    annotation (Dialog(group="Braking Chopper"));
  parameter Modelica.SIunits.Time period=1e-6 "Chopping period (1/f)"
    annotation (Dialog(group="Braking Chopper"));
  parameter Modelica.SIunits.Capacitance C_DCBus=1e-5 "DC bus capacitance"
    annotation (Dialog(group="Braking Chopper"));
  Controls.SpeedController speedController
    annotation (Placement(transformation(extent={{138,-46},{112,-36}})));
  Controls.VectorController vectorController
    annotation (Placement(transformation(extent={{88,-34},{68,-54}})));
equation
  connect(terminalBox.plug_sn, smpm.plug_sn)
    annotation (Line(points={{74,6},{74,6}}, color={0,0,255}));
  connect(terminalBox.plug_sp, smpm.plug_sp)
    annotation (Line(points={{86,6},{86,6}},
                                           color={0,0,255}));
  connect(threePhaseRectifier.plug_p1, plug_p1)
    annotation (Line(points={{-88,0},{-100,0}}, color={0,0,255}));
  connect(capacitor.p, threePhaseRectifier.load_p) annotation (Line(points={{
          -54,10},{-54,20},{-80,20},{-80,8.4}}, color={0,0,255}));
  connect(threePhaseRectifier.load_n, capacitor.n) annotation (Line(points={{
          -80,-8.4},{-80,-20},{-54,-20},{-54,-10}}, color={0,0,255}));
  connect(brakeChopper.in_p, threePhaseRectifier.load_p) annotation (Line(
        points={{-34.4,3.4},{-34.4,4},{-40,4},{-40,20},{-80,20},{-80,8.4}},
        color={0,0,255}));
  connect(brakeChopper.in_n, capacitor.n) annotation (Line(points={{-34.4,-4},{
          -40,-4},{-40,-20},{-54,-20},{-54,-10}}, color={0,0,255}));
  connect(brakeChopper.p_out, inverter1.dc_p) annotation (Line(points={{-19.6,
          3.4},{-9.8,3.4},{-9.8,28},{30,28}}, color={0,0,255}));
  connect(brakeChopper.n_out, inverter1.dc_n) annotation (Line(points={{-19.6,
          -4},{4,-4},{4,16},{30,16}}, color={0,0,255}));
  connect(inverter1.ac, terminalBox.plugSupply)
    annotation (Line(points={{50,22},{80,22},{80,8}}, color={0,0,255}));
  connect(signalPWM.fire, inverter1.fire_p)
    annotation (Line(points={{34,3},{34,10}}, color={255,0,255}));
  connect(signalPWM.notFire, inverter1.fire_n)
    annotation (Line(points={{46,3},{46,10}}, color={255,0,255}));
  connect(signalPWM.dutyCycle, sine.y) annotation (Line(points={{28,-8},{22,-8},
          {22,-42},{29,-42}}, color={0,0,127}));
  connect(smpm.flange, angleSensor.flange)
    annotation (Line(points={{90,-4},{102,-4}}, color={0,0,0}));
  connect(angleSensor.phi, derivative.u)
    annotation (Line(points={{123,-4},{138,-4}}, color={0,0,127}));
  connect(speedController.N, derivative.y) annotation (Line(points={{140.2,-38},
          {170,-38},{170,-4},{161,-4}}, color={0,0,127}));
  connect(w_ref, speedController.Np) annotation (Line(points={{-120,-72},{170,
          -72},{170,-44},{140,-44}}, color={0,0,127}));
  connect(vectorController.Te, speedController.y1) annotation (Line(points={{90,
          -50},{106,-50},{106,-41},{111,-41}}, color={0,0,127}));
  connect(vectorController.theta_r, derivative.u) annotation (Line(points={{90,
          -44},{98,-44},{98,-18},{130,-18},{130,-4},{138,-4}}, color={0,0,127}));
  connect(vectorController.Iabc, terminalBox.plugSupply) annotation (Line(
        points={{88,-38},{96,-38},{96,22},{80,22},{80,8}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{180,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{180,
            100}})));
end Boeing747_MotorDrive;
