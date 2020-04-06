within CHEETA.Examples.CHEETAElectricalSystem;
model HTS_test "Test for HTS transmission"
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0,   L=0)
    annotation (Placement(transformation(extent={{-108,6},{-96,18}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly(                      l=1)
    annotation (Placement(transformation(extent={{-20,14},{-4,22}})));
  Aircraft.Electrical.HTS.Stekly.Stekly
                                      stekly1(                      l=1)
    annotation (Placement(transformation(extent={{-20,2},{-4,10}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{-26,-60},{-46,-40}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker4(k=200000)
    annotation (Placement(transformation(extent={{-54,16},{-34,24}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker5(k=200000)
    annotation (Placement(transformation(extent={{-54,4},{-34,12}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker6(k=200000)
    annotation (Placement(transformation(extent={{8,16},{28,24}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker7(k=200000)
    annotation (Placement(transformation(extent={{8,4},{28,12}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter1(
      constantEnable=false)
    annotation (Placement(transformation(extent={{46,2},{66,22}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm3(
    useConstantDutyCycle=false,
    constantDutyCycle=0.5,
    f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{66,-30},{46,-10}})));
  Aircraft.Electrical.Machines.Motors.SimpleMotor simpleMotor2
    annotation (Placement(transformation(extent={{80,2},{100,22}})));
  Aircraft.Electrical.Controls.VariableSpeedDrive variableSpeedDrive1(wref=
        41000,
      T=10) annotation (Placement(transformation(extent={{100,-30},{80,-10}})));
  Aircraft.Mechanical.Loads.Fan      fan
    annotation (Placement(transformation(extent={{126,2},{146,22}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{-84,-30},{-64,-10}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc(
    L=0.001,
    i=0,
    C=0.001,
    v(start=1000) = 1000)                                     annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-74,12})));
equation
  connect(fixedTemperature.port,thermalConductor. port_b)
    annotation (Line(points={{-60,-50},{-46,-50}},
                                                 color={191,0,0}));
  connect(stekly.port_a, stekly1.port_a)
    annotation (Line(points={{-12,14},{-12,2}},  color={191,0,0}));
  connect(stekly1.port_a, thermalConductor.port_a)
    annotation (Line(points={{-12,2},{-12,-50},{-26,-50}}, color={191,0,0}));
  connect(circuitBreaker4.n1, stekly.pin_p)
    annotation (Line(points={{-34.2,18},{-21,18}}, color={0,0,255}));
  connect(circuitBreaker6.p1, stekly.pin_n)
    annotation (Line(points={{8,18},{-3,18}}, color={0,0,255}));
  connect(circuitBreaker5.n1, stekly1.pin_p)
    annotation (Line(points={{-34.2,6},{-21,6}}, color={0,0,255}));
  connect(circuitBreaker7.p1, stekly1.pin_n)
    annotation (Line(points={{8,6},{-3,6}}, color={0,0,255}));
  connect(variableSpeedDrive1.flange1,simpleMotor2. flange1) annotation (Line(
        points={{100.2,-20},{114,-20},{114,12},{100.4,12}},
                                                          color={0,0,0}));
  connect(simpleMotor2.p1,inverter1. ac)
    annotation (Line(points={{79.6,12},{66,12}},   color={0,0,255}));
  connect(inverter1.fire_p,pwm3. notFire)
    annotation (Line(points={{50,0},{50,-9}},    color={255,0,255}));
  connect(inverter1.fire_n,pwm3. fire)
    annotation (Line(points={{62,0},{62,-9}},    color={255,0,255}));
  connect(fan.flange_a1, simpleMotor2.flange1)
    annotation (Line(points={{123.5,12},{100.4,12}}, color={0,0,0}));
  connect(inverter1.dc_p, circuitBreaker6.n1)
    annotation (Line(points={{46,18},{27.8,18}}, color={0,0,255}));
  connect(inverter1.dc_n, circuitBreaker7.n1)
    annotation (Line(points={{46,6},{27.8,6}}, color={0,0,255}));
  connect(variableSpeedDrive1.y1, pwm3.dutyCycle)
    annotation (Line(points={{79,-20},{68,-20}}, color={0,0,127}));
  connect(dcdc.fire_p,pwm. fire)
    annotation (Line(points={{-80,0},{-80,-9}},    color={255,0,255}));
  connect(dcdc.dc_p2, circuitBreaker4.p1)
    annotation (Line(points={{-64,18},{-54,18}}, color={0,0,255}));
  connect(circuitBreaker5.p1, dcdc.dc_n2)
    annotation (Line(points={{-54,6},{-64,6}}, color={0,0,255}));
  connect(dcdc.dc_p1, simplifiedFuelCell.pin_p) annotation (Line(points={{-84,
          18},{-90,18},{-90,16},{-95,16}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-95,
          8},{-90,8},{-90,6},{-84,6}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=20));
end HTS_test;
