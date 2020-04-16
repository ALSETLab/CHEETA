within CHEETA.Examples.CHEETAElectricalSystem;
model SingleBranch_3Phase
  parameter Real pi = Modelica.Constants.pi;
  parameter Integer m = 3 "Number of phases";
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
  Aircraft.Electrical.Machines.Motors.ThreePhaseMotor threePhaseMotor1
    annotation (Placement(transformation(extent={{96,-60},{118,-40}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM3
                                                                      [
    m](each useConstantDutyCycle=false, each f(displayUnit="kHz") = 100000)
                                                  annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        origin={74,-80})));
  Aircraft.Controls.SpeedDrives.SpeedDriveController
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
  Modelica.Electrical.PowerConverters.DCAC.MultiPhase2Level  inverter3(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{64,-124},{84,-104}})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc2
                                                              annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-52,-114})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM4(
      constantDutyCycle=0.6, f=100)
                                  annotation (Placement(transformation(
          extent={{-62,-154},{-42,-134}})));
  Aircraft.Electrical.Machines.Motors.ThreePhaseMotor threePhaseMotor2
    annotation (Placement(transformation(extent={{96,-124},{118,-104}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM5
                                                                      [
    m](each useConstantDutyCycle=false, each f(displayUnit="kHz") = 100000)
                                                  annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        origin={74,-144})));
  Aircraft.Controls.SpeedDrives.SpeedDriveController
    variableSpeedDrive_ThreePhase2(wref=41000, T=1)
    annotation (Placement(transformation(extent={{118,-154},{98,-134}})));
  Aircraft.Mechanical.Loads.Fan fan2
    annotation (Placement(transformation(extent={{128,-118},{136,-110}})));
  Aircraft.Electrical.HTS.HTS_exploss hTS_exploss(
    temperature=100,
    l=1,
    R=100) annotation (Placement(transformation(extent={{2,-112},{18,-104}})));
  Aircraft.Electrical.HTS.HTS_exploss hTS_exploss1(
    temperature=100,
    l=1,
    R=100) annotation (Placement(transformation(extent={{2,-124},{18,-116}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{-30,-110},{-10,-102}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{-30,-122},{-10,-114}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker3(k=200000)
    annotation (Placement(transformation(extent={{32,-122},{52,-114}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker8(k=200000)
    annotation (Placement(transformation(extent={{32,-110},{52,-102}})));
  Aircraft.Electrical.FuelCell.PEMFC pEMFC annotation (Placement(transformation(
        extent={{12,-8},{-12,8}},
        rotation=270,
        origin={-76,-114})));
equation
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
  connect(dcdc2.fire_p, signalPWM4.fire)
    annotation (Line(points={{-58,-126},{-58,-133}}, color={255,0,255}));
  connect(inverter3.ac, threePhaseMotor2.plug_p1)
    annotation (Line(points={{84,-114},{98,-114}}, color={0,0,255}));
  connect(threePhaseMotor2.flange1, fan2.flange_a1)
    annotation (Line(points={{118,-114},{127,-114}}, color={0,0,0}));
  connect(variableSpeedDrive_ThreePhase2.flange1, fan2.flange_a1) annotation (
      Line(points={{118.2,-144},{124,-144},{124,-114},{127,-114}}, color={0,0,0}));
  connect(inverter3.fire_p, signalPWM5.notFire)
    annotation (Line(points={{68,-126},{68,-133}}, color={255,0,255}));
  connect(inverter3.fire_n, signalPWM5.fire)
    annotation (Line(points={{80,-126},{80,-133}}, color={255,0,255}));
  connect(variableSpeedDrive_ThreePhase2.y1, signalPWM5.dutyCycle)
    annotation (Line(points={{97,-144},{86,-144}}, color={0,0,127}));
  connect(dcdc2.dc_p2, circuitBreaker1.p1)
    annotation (Line(points={{-42,-108},{-30,-108}}, color={0,0,255}));
  connect(hTS_exploss.pin_p, circuitBreaker1.n1)
    annotation (Line(points={{1,-108},{-10.2,-108}}, color={0,0,255}));
  connect(circuitBreaker2.p1, dcdc2.dc_n2)
    annotation (Line(points={{-30,-120},{-42,-120}}, color={0,0,255}));
  connect(hTS_exploss1.pin_p, circuitBreaker2.n1)
    annotation (Line(points={{1,-120},{-10.2,-120}}, color={0,0,255}));
  connect(circuitBreaker8.p1, hTS_exploss.pin_n)
    annotation (Line(points={{32,-108},{19,-108}}, color={0,0,255}));
  connect(inverter3.dc_p, circuitBreaker8.n1)
    annotation (Line(points={{64,-108},{51.8,-108}}, color={0,0,255}));
  connect(hTS_exploss1.pin_n, circuitBreaker3.p1)
    annotation (Line(points={{19,-120},{32,-120}}, color={0,0,255}));
  connect(inverter3.dc_n, circuitBreaker3.n1)
    annotation (Line(points={{64,-120},{51.8,-120}}, color={0,0,255}));
  connect(pEMFC.pin_p1, dcdc2.dc_p1) annotation (Line(points={{-69,-108},{-64,
          -108},{-64,-108},{-62,-108}}, color={0,0,255}));
  connect(pEMFC.pin_n1, dcdc2.dc_n1)
    annotation (Line(points={{-69,-120},{-62,-120}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},{140,
            60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},{
            140,60}})),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=10));
end SingleBranch_3Phase;
