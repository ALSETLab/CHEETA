within CHEETA.Examples.NotionalElectricalSystem;
model SingleBranch_Resistive_load "Resistive load connected to inverter"
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0, L=
        0.0001)
              annotation (Placement(transformation(extent={{-74,-6},{-62,6}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=1)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
                                                 annotation (Placement(
        transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={68,-8})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter2(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{2,-10},{22,10}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM1(
    useConstantDutyCycle=true,
    constantDutyCycle=0.5,
    f=100)                             annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={12,-34})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(i(fixed=true, start=5000),
      L=1e-6)
    annotation (Placement(transformation(extent={{-28,-4},{-8,16}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(i(fixed=true, start=5000),
      L=1e-6)
    annotation (Placement(transformation(extent={{-28,-16},{-8,4}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-44,0})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM2(
      constantDutyCycle=0.6, f=100)
                                  annotation (Placement(transformation(
          extent={{-54,-44},{-34,-24}})));
equation
  connect(resistor2.n, ground3.p)
    annotation (Line(points={{60,0},{68,0}}, color={0,0,255}));
  connect(signalPWM1.notFire, inverter2.fire_n)
    annotation (Line(points={{18,-23},{18,-12}}, color={255,0,255}));
  connect(inverter2.fire_p, signalPWM1.fire)
    annotation (Line(points={{6,-12},{6,-23}}, color={255,0,255}));
  connect(inverter2.ac, resistor2.p)
    annotation (Line(points={{22,0},{40,0}}, color={0,0,255}));
  connect(inductor.n, inverter2.dc_p)
    annotation (Line(points={{-8,6},{2,6}}, color={0,0,255}));
  connect(inductor1.n, inverter2.dc_n)
    annotation (Line(points={{-8,-6},{2,-6}}, color={0,0,255}));
  connect(dcdc.dc_p2, inductor.p)
    annotation (Line(points={{-34,6},{-28,6}}, color={0,0,255}));
  connect(dcdc.dc_n2, inductor1.p)
    annotation (Line(points={{-34,-6},{-28,-6}}, color={0,0,255}));
  connect(dcdc.fire_p, signalPWM2.fire)
    annotation (Line(points={{-50,-12},{-50,-23}}, color={255,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-61,4},
          {-58,4},{-58,6},{-54,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-61,
          -4},{-58,-4},{-58,-6},{-54,-6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end SingleBranch_Resistive_load;
