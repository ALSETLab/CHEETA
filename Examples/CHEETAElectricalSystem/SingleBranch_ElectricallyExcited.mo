within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_ElectricallyExcited
  parameter Real pi = Modelica.Constants.pi;
  parameter Integer m = 3 "Number of phases";
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=
        0.001) annotation (Placement(transformation(extent={{-86,-6},{-74,6}})));
  parameter Records.NotionalPowerSystem.SM_PermanentMagnetData smpmData
    annotation (Placement(transformation(extent={{42,20},{62,40}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
      constantEnable=false)
    annotation (Placement(transformation(extent={{26,-10},{46,10}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{26,-42},{46,-22}})));
  Aircraft.Electrical.Machines.AIMC_SquirrelCage rectifierDrivenGenerator
    annotation (Placement(transformation(extent={{88,-8},{108,8}})));
  Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor
    annotation (Placement(transformation(extent={{56,-10},{76,10}})));
  Aircraft.Mechanical.Loads.Fan fan
    annotation (Placement(transformation(extent={{120,-4},{128,4}})));
equation
  connect(dcdc.fire_p, pwm.fire)
    annotation (Line(points={{-56,-12},{-56,-19}}, color={255,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-73,4},
          {-66,4},{-66,6},{-60,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-73,
          -4},{-68,-4},{-68,-6},{-60,-6}}, color={0,0,255}));
  connect(inverter.fire_p, pwm1.fire)
    annotation (Line(points={{30,-12},{30,-21}}, color={255,0,255}));
  connect(inverter.fire_n, pwm1.notFire)
    annotation (Line(points={{42,-12},{42,-21}}, color={255,0,255}));
  connect(inverter.ac, potentialSensor.p)
    annotation (Line(points={{46,0},{56,0}}, color={0,0,255}));
  connect(rectifierDrivenGenerator.v1, potentialSensor.phi) annotation (Line(
        points={{91.2727,0.8},{84,0.8},{84,0},{77,0}},
                                                    color={0,0,127}));
  connect(rectifierDrivenGenerator.flange1, fan.flange_a1)
    annotation (Line(points={{107.273,0},{119,0}},
                                               color={0,0,0}));
  connect(dcdc.dc_p2, inverter.dc_p)
    annotation (Line(points={{-40,6},{26,6}}, color={0,0,255}));
  connect(inverter.dc_n, dcdc.dc_n2)
    annotation (Line(points={{26,-6},{-40,-6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{140,
            60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            140,60}})),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=0.5));
end SingleBranch_ElectricallyExcited;
