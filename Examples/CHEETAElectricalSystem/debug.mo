within CHEETA.Examples.CHEETAElectricalSystem;
model debug "Resistive load connected to inverter"
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter1(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{4,44},{24,64}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM(
    useConstantDutyCycle=true,
    constantDutyCycle=0.5,
    f=100)                             annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={14,4})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_n(V=500)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-58,28})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_p(V=500)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-58,66})));
  Modelica.Electrical.Analog.Basic.Ground ground2
                                                 annotation (Placement(
        transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={-88,24})));
  Aircraft.Electrical.Machines.AIMC_SquirrelCage rectifierDrivenGenerator
    annotation (Placement(transformation(extent={{60,46},{80,62}})));
  Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor
    annotation (Placement(transformation(extent={{30,44},{50,64}})));
  Aircraft.Mechanical.Loads.Fan fan
    annotation (Placement(transformation(extent={{94,50},{102,58}})));
  parameter Records.NotionalPowerSystem.SM_PermanentMagnetData smpmData
    annotation (Placement(transformation(extent={{48,10},{68,30}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter2(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{2,-50},{22,-30}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM1(
    useConstantDutyCycle=true,
    constantDutyCycle=0.5,
    f=100)                             annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={12,-90})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_n1(V=500)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-68})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_p1(V=500)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-28})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={-90,-72})));
  parameter Records.NotionalPowerSystem.SM_PermanentMagnetData smpmData1
    annotation (Placement(transformation(extent={{46,-84},{66,-64}})));
  Aircraft.Electrical.Machines.SimpleMotor simpleMotor(R_hyst(displayUnit="Ohm")=
         149) annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-22,-40})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-32,-80},{-12,-60}})));
  Modelica.Mechanics.Rotational.Components.Inertia
                                J2(J=10, w(start=2400))
                             annotation (Placement(transformation(extent={{62,-48},
            {82,-28}})));
equation
  connect(signalPWM.fire, inverter1.fire_p)
    annotation (Line(points={{8,15},{8,42}}, color={255,0,255}));
  connect(constantVoltage_p.p, inverter1.dc_p) annotation (Line(points={{-58,76},
          {-58,90},{-4,90},{-4,60},{4,60}}, color={0,0,255}));
  connect(inverter1.dc_n, constantVoltage_n.n) annotation (Line(points={{4,48},{
          -28,48},{-28,18},{-58,18}},  color={0,0,255}));
  connect(constantVoltage_p.n, constantVoltage_n.p)
    annotation (Line(points={{-58,56},{-58,38}}, color={0,0,255}));
  connect(ground2.p, constantVoltage_n.p) annotation (Line(points={{-88,32},{
          -88,46},{-58,46},{-58,38}}, color={0,0,255}));
  connect(signalPWM.notFire, inverter1.fire_n)
    annotation (Line(points={{20,15},{20,42}}, color={255,0,255}));
  connect(rectifierDrivenGenerator.v1,potentialSensor. phi) annotation (Line(
        points={{63.2727,54.8},{58,54.8},{58,54},{51,54}},
                                                    color={0,0,127}));
  connect(rectifierDrivenGenerator.flange1,fan. flange_a1)
    annotation (Line(points={{79.2727,54},{93,54}},
                                               color={0,0,0}));
  connect(potentialSensor.p, inverter1.ac)
    annotation (Line(points={{30,54},{24,54}}, color={0,0,255}));
  connect(signalPWM1.fire, inverter2.fire_p)
    annotation (Line(points={{6,-79},{6,-52}}, color={255,0,255}));
  connect(constantVoltage_p1.n, constantVoltage_n1.p)
    annotation (Line(points={{-60,-38},{-60,-58}}, color={0,0,255}));
  connect(ground1.p, constantVoltage_n1.p) annotation (Line(points={{-90,-64},{
          -90,-48},{-60,-48},{-60,-58}}, color={0,0,255}));
  connect(signalPWM1.notFire, inverter2.fire_n)
    annotation (Line(points={{18,-79},{18,-52}}, color={255,0,255}));
  connect(inverter2.ac, simpleMotor.p1)
    annotation (Line(points={{22,-40},{33.6,-40}}, color={0,0,255}));
  connect(pwm.fire, dcdc.fire_p)
    annotation (Line(points={{-28,-59},{-28,-52}}, color={255,0,255}));
  connect(dcdc.dc_p2, inverter2.dc_p)
    annotation (Line(points={{-12,-34},{2,-34}}, color={0,0,255}));
  connect(dcdc.dc_n2, inverter2.dc_n)
    annotation (Line(points={{-12,-46},{2,-46}}, color={0,0,255}));
  connect(constantVoltage_p1.p, dcdc.dc_p1) annotation (Line(points={{-60,-18},
          {-60,-6},{-40,-6},{-40,-34},{-32,-34}}, color={0,0,255}));
  connect(dcdc.dc_n1, constantVoltage_n1.n) annotation (Line(points={{-32,-46},
          {-40,-46},{-40,-50},{-42,-50},{-42,-78},{-60,-78}}, color={0,0,255}));
  connect(simpleMotor.flange1, J2.flange_a) annotation (Line(points={{54.4,-40},
          {58,-40},{58,-38},{62,-38}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=0.001));
end debug;
