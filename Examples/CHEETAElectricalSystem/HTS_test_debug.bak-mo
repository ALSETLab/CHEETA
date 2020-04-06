within CHEETA.Examples.CHEETAElectricalSystem;
model HTS_test_debug "Test for HTS transmission"
  Aircraft.Electrical.HTS.Stekly.Stekly_CopperLosses stekly_ACLosses(l=1, P(
        displayUnit="mm"))
    annotation (Placement(transformation(extent={{-20,14},{-4,22}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_CopperLosses stekly_ACLosses1(
    l=1,
    P(displayUnit="mm"),
    rho=2.1) annotation (Placement(transformation(extent={{-20,2},{-4,10}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{-26,-60},{-46,-40}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=100)
    annotation (Placement(transformation(extent={{30,8},{50,28}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=100)
    annotation (Placement(transformation(extent={{30,-20},{50,0}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{56,-38},{76,-18}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc(
    VkneeDiode=1,
    L(displayUnit="uH") = 1e-6,
    i=0.0001,
    C=0.1,
    v=1000)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-78,12})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-88,-28},{-68,-8}})));
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(
    R=0,
    L=0,
    V=1000)    annotation (Placement(transformation(extent={{-114,6},{-102,18}})));
equation
  connect(fixedTemperature.port,thermalConductor. port_b)
    annotation (Line(points={{-60,-50},{-46,-50}},
                                                 color={191,0,0}));
  connect(stekly_ACLosses.port_a, stekly_ACLosses1.port_a)
    annotation (Line(points={{-12,14},{-12,2}}, color={191,0,0}));
  connect(stekly_ACLosses1.port_a, thermalConductor.port_a)
    annotation (Line(points={{-12,2},{-12,-50},{-26,-50}}, color={191,0,0}));
  connect(stekly_ACLosses.pin_n, resistor.p)
    annotation (Line(points={{-3,18},{30,18}}, color={0,0,255}));
  connect(stekly_ACLosses1.pin_n, resistor1.p) annotation (Line(points={{-3,6},
          {14,6},{14,-10},{30,-10}}, color={0,0,255}));
  connect(resistor.n, ground.p)
    annotation (Line(points={{50,18},{66,18},{66,-18}}, color={0,0,255}));
  connect(resistor1.n, ground.p)
    annotation (Line(points={{50,-10},{66,-10},{66,-18}}, color={0,0,255}));
  connect(dcdc.fire_p,pwm. fire)
    annotation (Line(points={{-84,0},{-84,-7}},    color={255,0,255}));
  connect(simplifiedFuelCell.pin_p,dcdc. dc_p1) annotation (Line(points={{-101,16},
          {-94,16},{-94,18},{-88,18}},
                                    color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1,dcdc. dc_n1) annotation (Line(points={{-101,8},
          {-96,8},{-96,6},{-88,6}},        color={0,0,255}));
  connect(stekly_ACLosses.pin_p, dcdc.dc_p2)
    annotation (Line(points={{-21,18},{-68,18}}, color={0,0,255}));
  connect(dcdc.dc_n2, stekly_ACLosses1.pin_p)
    annotation (Line(points={{-68,6},{-21,6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end HTS_test_debug;
