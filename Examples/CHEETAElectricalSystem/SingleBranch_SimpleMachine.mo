within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_SimpleMachine
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0, L=
        0.0001)
              annotation (Placement(transformation(extent={{-68,-6},{-56,6}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter2(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{8,-10},{28,10}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM1(
    useConstantDutyCycle=true,
    constantDutyCycle=0.5,
    f=1000)                            annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={18,-34})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(i(start=5000), L=1e-6)
    annotation (Placement(transformation(extent={{-22,-4},{-2,16}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(i(start=5000), L=1e-6)
    annotation (Placement(transformation(extent={{-22,-16},{-2,4}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-38,0})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM2(
      constantDutyCycle=0.6, f=100)
                                  annotation (Placement(transformation(
          extent={{-48,-44},{-28,-24}})));
  Aircraft.Electrical.Machines.SimpleMotor simpleMotor1
    annotation (Placement(transformation(extent={{38,-10},{58,10}})));
equation
  connect(signalPWM1.notFire, inverter2.fire_n)
    annotation (Line(points={{24,-23},{24,-12}}, color={255,0,255}));
  connect(inverter2.fire_p, signalPWM1.fire)
    annotation (Line(points={{12,-12},{12,-23}}, color={255,0,255}));
  connect(inductor.n, inverter2.dc_p)
    annotation (Line(points={{-2,6},{8,6}}, color={0,0,255}));
  connect(inductor1.n, inverter2.dc_n)
    annotation (Line(points={{-2,-6},{8,-6}}, color={0,0,255}));
  connect(dcdc.dc_p2, inductor.p)
    annotation (Line(points={{-28,6},{-22,6}}, color={0,0,255}));
  connect(dcdc.dc_n2, inductor1.p)
    annotation (Line(points={{-28,-6},{-22,-6}}, color={0,0,255}));
  connect(dcdc.fire_p, signalPWM2.fire)
    annotation (Line(points={{-44,-12},{-44,-23}}, color={255,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-55,4},
          {-52,4},{-52,6},{-48,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-55,
          -4},{-52,-4},{-52,-6},{-48,-6}}, color={0,0,255}));
  connect(inverter2.ac, simpleMotor1.p1)
    annotation (Line(points={{28,0},{37.6,0}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=0.5));
end SingleBranch_SimpleMachine;
