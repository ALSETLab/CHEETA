within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_HTS_Test
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
  Aircraft.Electrical.HTS.HTS_Cooling hTS_Cooling(l=1)
    annotation (Placement(transformation(extent={{-12,-4},{8,16}})));
  Aircraft.Electrical.HTS.HTS_Cooling hTS_Cooling1(l=1)
    annotation (Placement(transformation(extent={{-12,-16},{8,4}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=100)
    annotation (Placement(transformation(extent={{32,-4},{52,16}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=100)
    annotation (Placement(transformation(extent={{32,-16},{52,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{54,-34},{74,-14}})));
equation
  connect(dcdc.fire_p, pwm.fire)
    annotation (Line(points={{-56,-12},{-56,-19}}, color={255,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-73,4},
          {-66,4},{-66,6},{-60,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-73,
          -4},{-68,-4},{-68,-6},{-60,-6}}, color={0,0,255}));
  connect(dcdc.dc_p2, hTS_Cooling.p_in1)
    annotation (Line(points={{-40,6},{-12,6}}, color={0,0,255}));
  connect(dcdc.dc_n2, hTS_Cooling1.p_in1)
    annotation (Line(points={{-40,-6},{-12,-6}}, color={0,0,255}));
  connect(hTS_Cooling.p_out1, resistor.p)
    annotation (Line(points={{8,6},{32,6}}, color={0,0,255}));
  connect(hTS_Cooling1.p_out1, resistor1.p)
    annotation (Line(points={{8,-6},{32,-6}}, color={0,0,255}));
  connect(resistor1.n, ground.p)
    annotation (Line(points={{52,-6},{64,-6},{64,-14}}, color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{52,6},{64,6},{64,-14}}, color={0,0,255}));
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
end SingleBranch_HTS_Test;
