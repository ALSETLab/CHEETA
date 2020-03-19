within CHEETA.Examples.NotionalElectricalSystem;
model SingleBranch_SimpleMachine
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0, L=0)
              annotation (Placement(transformation(extent={{-68,-6},{-56,6}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter2(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{8,-10},{28,10}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM1(
    useConstantDutyCycle=false,
    constantDutyCycle=1,
    f=10,
    startTime=80)                      annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        origin={18,-34})));
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
  Aircraft.Mechanical.Loads.Pinwheel
                                pinwheel
    annotation (Placement(transformation(extent={{80,-4},{88,4}})));
  Aircraft.Electrical.Controls.VariableSpeedDrive variableSpeedDrive(wref=4000,
      T=10) annotation (Placement(transformation(extent={{64,-44},{44,-24}})));
equation
  connect(dcdc.fire_p, signalPWM2.fire)
    annotation (Line(points={{-44,-12},{-44,-23}}, color={255,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-55,4},
          {-52,4},{-52,6},{-48,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-55,
          -4},{-52,-4},{-52,-6},{-48,-6}}, color={0,0,255}));
  connect(inverter2.ac, simpleMotor1.p1)
    annotation (Line(points={{28,0},{37.6,0}}, color={0,0,255}));
  connect(dcdc.dc_p2, inverter2.dc_p)
    annotation (Line(points={{-28,6},{-10,6},{-10,6},{8,6}}, color={0,0,255}));
  connect(inverter2.dc_n, dcdc.dc_n2)
    annotation (Line(points={{8,-6},{-28,-6}}, color={0,0,255}));
  connect(pinwheel.flange_a1, simpleMotor1.flange1)
    annotation (Line(points={{80,0},{70,0},{70,0},{58.4,0}}, color={0,0,0}));
  connect(variableSpeedDrive.y1, signalPWM1.dutyCycle) annotation (Line(points={{43,-34},
          {30,-34}},                           color={0,0,127}));
  connect(variableSpeedDrive.flange1, simpleMotor1.flange1) annotation (Line(
        points={{64.2,-34},{72,-34},{72,0},{58.4,0}}, color={0,0,0}));
  connect(signalPWM1.notFire, inverter2.fire_p)
    annotation (Line(points={{12,-23},{12,-12}}, color={255,0,255}));
  connect(signalPWM1.fire, inverter2.fire_n)
    annotation (Line(points={{24,-23},{24,-12}}, color={255,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=500, __Dymola_NumberOfIntervals=5000));
end SingleBranch_SimpleMachine;
