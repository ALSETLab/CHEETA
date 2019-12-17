within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_InductiveMachine
  parameter Real pi = Modelica.Constants.pi;
  parameter Integer m = 3 "Number of phases";
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-4},{-4,16}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-16},{-4,4}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=
        0.001) annotation (Placement(transformation(extent={{-86,-6},{-74,6}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM[3](each
      useConstantDutyCycle=false, each f(displayUnit="kHz") = 100000)
                                                  annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={22,-36})));
  Modelica.Electrical.PowerConverters.DCAC.MultiPhase2Level inverter1(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{12,-10},{32,10}})));
  Modelica.Blocks.Sources.Sine sine[3](
    phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(3),
    startTime=zeros(3),
    amplitude=fill(0.5, 3),
    offset=fill(0.5, 3),
    freqHz=fill(100e3, 3))
                        annotation (Placement(transformation(extent={{32,-80},{12,
            -60}})));
  parameter Records.NotionalPowerSystem.SM_PermanentMagnetData smpmData
    annotation (Placement(transformation(extent={{42,20},{62,40}})));
equation
  connect(dcdc.dc_p2, inductor.p)
    annotation (Line(points={{-40,6},{-24,6}}, color={0,0,255}));
  connect(dcdc.fire_p, pwm.fire)
    annotation (Line(points={{-56,-12},{-56,-19}}, color={255,0,255}));
  connect(inductor1.p, dcdc.dc_n2)
    annotation (Line(points={{-24,-6},{-40,-6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-73,4},
          {-66,4},{-66,6},{-60,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-73,
          -4},{-68,-4},{-68,-6},{-60,-6}}, color={0,0,255}));
  connect(signalPWM.fire, inverter1.fire_p)
    annotation (Line(points={{16,-25},{16,-12}}, color={255,0,255}));
  connect(signalPWM.notFire, inverter1.fire_n)
    annotation (Line(points={{28,-25},{28,-12}}, color={255,0,255}));
  connect(sine.y,signalPWM. dutyCycle) annotation (Line(
      points={{11,-70},{2,-70},{2,-36},{10,-36}},       color={0,0,127}));
  connect(inverter1.dc_p, inductor.n)
    annotation (Line(points={{12,6},{-4,6}}, color={0,0,255}));
  connect(inductor1.n, inverter1.dc_n)
    annotation (Line(points={{-4,-6},{12,-6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=0.5));
end SingleBranch_InductiveMachine;
