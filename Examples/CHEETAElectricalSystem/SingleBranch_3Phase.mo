within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_3Phase
  parameter Real pi = Modelica.Constants.pi;
  parameter Integer m = 3 "Number of phases";
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=0, L=0)
              annotation (Placement(transformation(extent={{-80,24},{-68,36}})));
  Modelica.Electrical.PowerConverters.DCAC.MultiPhase2Level  inverter2(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{66,20},{86,40}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,30})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM2(
      constantDutyCycle=0.6, f=100)
                                  annotation (Placement(transformation(
          extent={{-60,-10},{-40,10}})));
  Aircraft.Electrical.Machines.ThreePhaseMotor threePhaseMotor
    annotation (Placement(transformation(extent={{98,20},{120,40}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM[
    m](each useConstantDutyCycle=false, each f(displayUnit="kHz") = 100000)
                                                  annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        origin={76,0})));
  Aircraft.Electrical.Controls.VariableSpeedDrive_ThreePhase
    variableSpeedDrive_ThreePhase(wref=41000, T=1)
    annotation (Placement(transformation(extent={{120,-10},{100,10}})));
  Aircraft.Mechanical.Loads.Fan fan
    annotation (Placement(transformation(extent={{130,26},{138,34}})));
  Modelica.Electrical.PowerConverters.DCAC.MultiPhase2Level  inverter1(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{64,-60},{84,-40}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc1
                                                              annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,-50})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM1(
      constantDutyCycle=0.6, f=100)
                                  annotation (Placement(transformation(
          extent={{-60,-90},{-40,-70}})));
  Aircraft.Electrical.Machines.ThreePhaseMotor threePhaseMotor1
    annotation (Placement(transformation(extent={{96,-60},{118,-40}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM3
                                                                      [
    m](each useConstantDutyCycle=false, each f(displayUnit="kHz") = 100000)
                                                  annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        origin={74,-80})));
  Aircraft.Electrical.Controls.VariableSpeedDrive_ThreePhase
    variableSpeedDrive_ThreePhase1(wref=41000, T=1)
    annotation (Placement(transformation(extent={{118,-90},{98,-70}})));
  Aircraft.Mechanical.Loads.Fan fan1
    annotation (Placement(transformation(extent={{130,-54},{138,-46}})));
  Electrification.Electrical.DCSplitter  splitterHVDC
    annotation (Placement(transformation(extent={{-72,-54},{-64,-46}})));
              Electrification.Batteries.Examples.BatteryPack
    batteryPack(internal_ground=true, enable_thermal_port=false)
    annotation (Placement(transformation(extent={{-72,-60},{-92,-40}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-78,-88},{-58,-68}})));
  Aircraft.Electrical.HTS.SimpleLine simpleLine
    annotation (Placement(transformation(extent={{6,32},{26,40}})));
  Aircraft.Electrical.HTS.SimpleLine simpleLine1
    annotation (Placement(transformation(extent={{6,20},{26,28}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker(k=200000)
    annotation (Placement(transformation(extent={{-26,34},{-6,42}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{-26,22},{-6,30}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{36,34},{56,42}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker3(k=200000)
    annotation (Placement(transformation(extent={{36,22},{56,30}})));
  Aircraft.Electrical.HTS.SimpleLine simpleLine2
    annotation (Placement(transformation(extent={{4,-48},{24,-40}})));
  Aircraft.Electrical.HTS.SimpleLine simpleLine3
    annotation (Placement(transformation(extent={{4,-60},{24,-52}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker4(k=200000)
    annotation (Placement(transformation(extent={{-28,-46},{-8,-38}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker5(k=200000)
    annotation (Placement(transformation(extent={{-28,-58},{-8,-50}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker6(k=200000)
    annotation (Placement(transformation(extent={{34,-46},{54,-38}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker7(k=200000)
    annotation (Placement(transformation(extent={{34,-58},{54,-50}})));
equation
  connect(dcdc.fire_p,signalPWM2. fire)
    annotation (Line(points={{-56,18},{-56,11}},   color={255,0,255}));
  connect(simplifiedFuelCell.pin_p,dcdc. dc_p1) annotation (Line(points={{-67,34},
          {-64,34},{-64,36},{-60,36}},
                                    color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1,dcdc. dc_n1) annotation (Line(points={{-67,26},
          {-64,26},{-64,24},{-60,24}},     color={0,0,255}));
  connect(inverter2.ac, threePhaseMotor.plug_p1)
    annotation (Line(points={{86,30},{100,30}}, color={0,0,255}));
  connect(threePhaseMotor.flange1, fan.flange_a1)
    annotation (Line(points={{120,30},{129,30}}, color={0,0,0}));
  connect(variableSpeedDrive_ThreePhase.flange1, fan.flange_a1) annotation (
      Line(points={{120.2,0},{126,0},{126,30},{129,30}}, color={0,0,0}));
  connect(inverter2.fire_p, signalPWM.notFire)
    annotation (Line(points={{70,18},{70,11}}, color={255,0,255}));
  connect(inverter2.fire_n, signalPWM.fire)
    annotation (Line(points={{82,18},{82,11}}, color={255,0,255}));
  connect(variableSpeedDrive_ThreePhase.y1, signalPWM.dutyCycle)
    annotation (Line(points={{99,0},{88,0}}, color={0,0,127}));
  connect(dcdc1.fire_p, signalPWM1.fire)
    annotation (Line(points={{-56,-62},{-56,-69}}, color={255,0,255}));
  connect(inverter1.ac, threePhaseMotor1.plug_p1)
    annotation (Line(points={{84,-50},{98,-50}}, color={0,0,255}));
  connect(threePhaseMotor1.flange1, fan1.flange_a1)
    annotation (Line(points={{118,-50},{129,-50}}, color={0,0,0}));
  connect(variableSpeedDrive_ThreePhase1.flange1, fan1.flange_a1) annotation (
      Line(points={{118.2,-80},{124,-80},{124,-50},{129,-50}}, color={0,0,0}));
  connect(inverter1.fire_p, signalPWM3.notFire)
    annotation (Line(points={{68,-62},{68,-69}}, color={255,0,255}));
  connect(inverter1.fire_n, signalPWM3.fire)
    annotation (Line(points={{80,-62},{80,-69}}, color={255,0,255}));
  connect(variableSpeedDrive_ThreePhase1.y1, signalPWM3.dutyCycle)
    annotation (Line(points={{97,-80},{86,-80}}, color={0,0,127}));
  connect(splitterHVDC.p, dcdc1.dc_p1)
    annotation (Line(points={{-64,-46},{-64,-44},{-60,-44}}, color={0,0,255}));
  connect(dcdc1.dc_n1, splitterHVDC.n)
    annotation (Line(points={{-60,-56},{-64,-56},{-64,-54}}, color={0,0,255}));
  connect(dcdc1.dc_n1, ground.p) annotation (Line(points={{-60,-56},{-62,-56},{
          -62,-68},{-68,-68}}, color={0,0,255}));
  connect(dcdc.dc_p2, circuitBreaker.p1)
    annotation (Line(points={{-40,36},{-26,36}}, color={0,0,255}));
  connect(simpleLine.p1, circuitBreaker.n1)
    annotation (Line(points={{7,36},{-6.2,36}}, color={0,0,255}));
  connect(dcdc.dc_n2, circuitBreaker1.p1)
    annotation (Line(points={{-40,24},{-26,24}}, color={0,0,255}));
  connect(simpleLine1.p1, circuitBreaker1.n1)
    annotation (Line(points={{7,24},{-6.2,24}}, color={0,0,255}));
  connect(simpleLine.n1, circuitBreaker2.p1)
    annotation (Line(points={{25,36},{36,36}}, color={0,0,255}));
  connect(inverter2.dc_p, circuitBreaker2.n1)
    annotation (Line(points={{66,36},{55.8,36}}, color={0,0,255}));
  connect(circuitBreaker3.p1, simpleLine1.n1)
    annotation (Line(points={{36,24},{25,24}}, color={0,0,255}));
  connect(circuitBreaker3.n1, inverter2.dc_n)
    annotation (Line(points={{55.8,24},{66,24}}, color={0,0,255}));
  connect(simpleLine2.p1, circuitBreaker4.n1)
    annotation (Line(points={{5,-44},{-8.2,-44}}, color={0,0,255}));
  connect(simpleLine3.p1, circuitBreaker5.n1)
    annotation (Line(points={{5,-56},{-8.2,-56}}, color={0,0,255}));
  connect(simpleLine2.n1, circuitBreaker6.p1)
    annotation (Line(points={{23,-44},{34,-44}}, color={0,0,255}));
  connect(circuitBreaker7.p1, simpleLine3.n1)
    annotation (Line(points={{34,-56},{23,-56}}, color={0,0,255}));
  connect(circuitBreaker4.p1, dcdc1.dc_p2)
    annotation (Line(points={{-28,-44},{-40,-44}}, color={0,0,255}));
  connect(circuitBreaker5.p1, dcdc1.dc_n2)
    annotation (Line(points={{-28,-56},{-40,-56}}, color={0,0,255}));
  connect(circuitBreaker6.n1, inverter1.dc_p)
    annotation (Line(points={{53.8,-44},{64,-44}}, color={0,0,255}));
  connect(inverter1.dc_n, circuitBreaker7.n1)
    annotation (Line(points={{64,-56},{53.8,-56}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-160},{140,
            60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-160},{
            140,60}})),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=10));
end SingleBranch_3Phase;
