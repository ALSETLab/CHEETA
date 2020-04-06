within CHEETA.Aircraft.Electrical.Machines.Examples.CHEETA;
model CooledMotor
  parameter Real pi = Modelica.Constants.pi;
  parameter Integer m = 3 "Number of phases";
  PowerElectronics.Converters.DCDC.BoostConverter        dcdc2(
    L=0.001,
    i=0,
    C=0.1,
    v=1000)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-60,-2})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM4(
      constantDutyCycle=0.6, f=100)
                                  annotation (Placement(transformation(
          extent={{-70,-42},{-50,-22}})));
  Motors.ThreePhaseMotor_Cooled threePhaseMotor_Cooled
    annotation (Placement(transformation(extent={{88,-12},{110,8}})));
  Aircraft.Electrical.Controls.VariableSpeedDrive_ThreePhase
    variableSpeedDrive_ThreePhase2(wref=41000, T=1)
    annotation (Placement(transformation(extent={{110,-42},{90,-22}})));
  Aircraft.Mechanical.Loads.Fan fan2
    annotation (Placement(transformation(extent={{120,-6},{128,2}})));
  HTS.Stekly.Stekly                   stekly(
    l=1)   annotation (Placement(transformation(extent={{-6,0},{10,8}})));
  HTS.Stekly.Stekly                   stekly1(
    l=1)   annotation (Placement(transformation(extent={{-6,-12},{10,-4}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{-38,2},{-18,10}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{-38,-10},{-18,-2}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker3(k=200000)
    annotation (Placement(transformation(extent={{24,-10},{44,-2}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker8(k=200000)
    annotation (Placement(transformation(extent={{24,2},{44,10}})));
  FuelCell.SimplifiedFuelCell        simplifiedFuelCell(R=100, L=0)
                                           annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=180,
        origin={-90,-2})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-70,-76},{-50,-56}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{-4,-76},{-24,-56}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.EnergyAnalyser machineEfficiencyComputation(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{90,20},{110,40}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.Switched.Ideal
                                                         inverter(redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Switched.Electrical.Records.Data.Ideal.Default
      data)
    annotation (Placement(transformation(extent={{56,-12},{76,8}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.SpaceVectorModulation
                                                             modulationMethod(fs=5e3)
    annotation (Placement(transformation(extent={{76,-42},{56,-22}})));
equation
  connect(dcdc2.fire_p, signalPWM4.fire)
    annotation (Line(points={{-66,-14},{-66,-21}},   color={255,0,255}));
  connect(threePhaseMotor_Cooled.flange1, fan2.flange_a1)
    annotation (Line(points={{110,-2},{119,-2}}, color={0,0,0}));
  connect(variableSpeedDrive_ThreePhase2.flange1, fan2.flange_a1) annotation (
      Line(points={{110.2,-32},{116,-32},{116,-2},{119,-2}},       color={0,0,0}));
  connect(dcdc2.dc_p2, circuitBreaker1.p1)
    annotation (Line(points={{-50,4},{-38,4}},       color={0,0,255}));
  connect(stekly.pin_p, circuitBreaker1.n1)
    annotation (Line(points={{-7,4},{-18.2,4}}, color={0,0,255}));
  connect(circuitBreaker2.p1, dcdc2.dc_n2)
    annotation (Line(points={{-38,-8},{-50,-8}},     color={0,0,255}));
  connect(stekly1.pin_p, circuitBreaker2.n1)
    annotation (Line(points={{-7,-8},{-18.2,-8}}, color={0,0,255}));
  connect(circuitBreaker8.p1, stekly.pin_n)
    annotation (Line(points={{24,4},{11,4}}, color={0,0,255}));
  connect(stekly1.pin_n, circuitBreaker3.p1)
    annotation (Line(points={{11,-8},{24,-8}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p1, dcdc2.dc_p1) annotation (Line(points={{-83,
          2},{-76,2},{-76,4},{-70,4}}, color={0,0,255}));
  connect(simplifiedFuelCell.pin_p, dcdc2.dc_n1) annotation (Line(points={{-83,
          -6},{-76,-6},{-76,-8},{-70,-8}}, color={0,0,255}));
  connect(fixedTemperature.port, thermalConductor.port_b)
    annotation (Line(points={{-50,-66},{-24,-66}}, color={191,0,0}));
  connect(thermalConductor.port_a, stekly1.port_a) annotation (Line(points={{-4,
          -66},{2,-66},{2,-12},{2,-12}}, color={191,0,0}));
  connect(stekly.port_a, stekly1.port_a)
    annotation (Line(points={{2,0},{2,-12}}, color={191,0,0}));
  connect(machineEfficiencyComputation.electricDriveBus, threePhaseMotor_Cooled.electricDriveBus1)
    annotation (Line(
      points={{100,20},{100,8}},
      color={0,86,166},
      thickness=0.5));
  connect(threePhaseMotor_Cooled.plug_p1, inverter.plug)
    annotation (Line(points={{90,-2},{76,-2}}, color={0,0,255}));
  connect(inverter.pin_p, circuitBreaker8.n1)
    annotation (Line(points={{56,4},{43.8,4}}, color={0,0,255}));
  connect(inverter.pin_n, circuitBreaker3.n1)
    annotation (Line(points={{56,-8},{43.8,-8}}, color={0,0,255}));
  connect(modulationMethod.phaseVoltages, variableSpeedDrive_ThreePhase2.y1)
    annotation (Line(points={{78,-32},{89,-32}}, color={0,0,127}));
  connect(modulationMethod.gateSignals, inverter.gateSignals) annotation (Line(
        points={{55,-32},{44,-32},{44,-2},{54,-2}}, color={255,0,255}));
  connect(modulationMethod.electricDriveBus, inverter.electricDriveBus)
    annotation (Line(
      points={{66,-42},{64,-42},{64,-58},{84,-58},{84,-16},{66,-16},{66,-12}},
      color={0,86,166},
      thickness=0.5));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{140,60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{140,
            60}})),
    Documentation(info="<html>
</html>"),
    experiment(StopTime=10));
end CooledMotor;
