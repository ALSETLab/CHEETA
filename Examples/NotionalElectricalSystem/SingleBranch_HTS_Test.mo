within CHEETA.Examples.NotionalElectricalSystem;
model SingleBranch_HTS_Test
  parameter Real pi = Modelica.Constants.pi;
  parameter Integer m = 3 "Number of phases";
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=1, L=0)
               annotation (Placement(transformation(extent={{-86,-8},{-74,4}})));
  parameter Records.NotionalPowerSystem.SM_PermanentMagnetData smpmData
    annotation (Placement(transformation(extent={{42,20},{62,40}})));
  Aircraft.Electrical.HTS.HTS hTS(l=1)
    annotation (Placement(transformation(extent={{-10,6},{6,14}})));
  Aircraft.Electrical.HTS.HTS hTS1(l=1)
    annotation (Placement(transformation(extent={{-10,-16},{6,-8}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=100,
    duration=20,
    offset=67)
    annotation (Placement(transformation(extent={{-44,-64},{-24,-44}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter2(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM1(
    useConstantDutyCycle=true,
    constantDutyCycle=1,
    f=100)                             annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={30,-34})));
  Aircraft.Electrical.Machines.Motors.SimpleMotor simpleMotor(R_hyst(
        displayUnit="Ohm") = 149)
    annotation (Placement(transformation(extent={{66,-10},{86,10}})));
equation
  connect(ramp.y, hTS1.temperature) annotation (Line(points={{-23,-54},{-16,-54},
          {-16,-52},{-2,-52},{-2,-16}}, color={0,0,127}));
  connect(hTS.temperature, hTS1.temperature) annotation (Line(points={{-2,6},{
          -16,6},{-16,-52},{-2,-52},{-2,-16}}, color={0,0,127}));
  connect(simplifiedFuelCell.pin_p, hTS.pin_p) annotation (Line(points={{-73,2},
          {-42,2},{-42,10},{-11,10}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, hTS1.pin_p) annotation (Line(points={{-73,-6},
          {-42,-6},{-42,-12},{-11,-12}},     color={0,0,255}));
  connect(signalPWM1.fire,inverter2. fire_p)
    annotation (Line(points={{24,-23},{24,-12}},
                                               color={255,0,255}));
  connect(signalPWM1.notFire,inverter2. fire_n)
    annotation (Line(points={{36,-23},{36,-12}}, color={255,0,255}));
  connect(inverter2.ac,simpleMotor. p1)
    annotation (Line(points={{40,0},{65.6,0}},     color={0,0,255}));
  connect(hTS1.pin_n, inverter2.dc_n) annotation (Line(points={{7,-12},{14,-12},
          {14,-6},{20,-6}}, color={0,0,255}));
  connect(hTS.pin_n, inverter2.dc_p)
    annotation (Line(points={{7,10},{14,10},{14,6},{20,6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{140,
            60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            140,60}})),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20, __Dymola_NumberOfIntervals=5000));
end SingleBranch_HTS_Test;
