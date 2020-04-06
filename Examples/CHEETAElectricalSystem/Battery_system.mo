within CHEETA.Examples.CHEETAElectricalSystem;
model Battery_system
  "AIM with forced cooling modeled in a simple thermal model"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{182,82},{194,94}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{54,32},{74,40}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{54,20},{74,28}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                        stekly_ExtraHeatGeneration(
                                               l=1, G_d=100)
    annotation (Placement(transformation(extent={{22,30},{38,38}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                        stekly_ExtraHeatGeneration1(
                                                l=1, G_d=100)
    annotation (Placement(transformation(extent={{22,18},{38,26}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{2,-46},{-18,-26}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-50,-46},{-30,-26}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc(
    L=0.001,
    i=0.1,
    C=0.001,
    v(start=1000) = 1000)                                     annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={6,28})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="Hz") = 100)
    annotation (Placement(transformation(extent={{-4,-14},{16,6}})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Blocks.EnergyAnalysis energyAnalysis(
      useBusConnector=true)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-42,74})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter(
      constantEnable=false)
    annotation (Placement(transformation(extent={{86,18},{106,38}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
    useConstantDutyCycle=false,
    constantDutyCycle=0.5,
    f(displayUnit="kHz") = 100000)
    annotation (Placement(transformation(extent={{106,-14},{86,6}})));
  Aircraft.Electrical.Machines.Motors.SimpleMotor simpleMotor1
    annotation (Placement(transformation(extent={{120,18},{140,38}})));
  Aircraft.Electrical.Controls.VariableSpeedDrive variableSpeedDrive(wref=41000,
      T=10) annotation (Placement(transformation(extent={{140,-14},{120,6}})));
  Aircraft.Mechanical.Loads.Fan      fan1
    annotation (Placement(transformation(extent={{160,18},{180,38}})));
  Aircraft.Electrical.Battery.DC_Battery dC_Battery
    annotation (Placement(transformation(extent={{-32,22},{-20,34}})));
equation
  connect(circuitBreaker1.p1, stekly_ExtraHeatGeneration.pin_n)
    annotation (Line(points={{54,34},{39,34}},color={0,0,255}));
  connect(circuitBreaker2.p1, stekly_ExtraHeatGeneration1.pin_n)
    annotation (Line(points={{54,22},{39,22}},color={0,0,255}));
  connect(stekly_ExtraHeatGeneration.port_a, thermalConductor.port_a)
    annotation (Line(points={{30,30},{30,-36},{2,-36}},  color={191,0,0}));
  connect(stekly_ExtraHeatGeneration1.port_a, thermalConductor.port_a)
    annotation (Line(points={{30,18},{30,-36},{2,-36}},  color={191,0,0}));
  connect(thermalConductor.port_b, fixedTemperature.port)
    annotation (Line(points={{-18,-36},{-30,-36}}, color={191,0,0}));
  connect(dcdc.fire_p,pwm. fire)
    annotation (Line(points={{0,16},{0,7}},        color={255,0,255}));
  connect(dcdc.dc_p2, stekly_ExtraHeatGeneration.pin_p)
    annotation (Line(points={{16,34},{21,34}},   color={0,0,255}));
  connect(stekly_ExtraHeatGeneration1.pin_p, dcdc.dc_n2)
    annotation (Line(points={{21,22},{16,22}},   color={0,0,255}));
  connect(variableSpeedDrive.flange1,simpleMotor1. flange1) annotation (Line(
        points={{140.2,-4},{154,-4},{154,28},{140.4,28}},
                                                      color={0,0,0}));
  connect(simpleMotor1.p1,inverter. ac)
    annotation (Line(points={{119.6,28},{106,28}},
                                                 color={0,0,255}));
  connect(inverter.fire_n,pwm1. fire)
    annotation (Line(points={{102,16},{102,7}},color={255,0,255}));
  connect(variableSpeedDrive.y1,pwm1. dutyCycle)
    annotation (Line(points={{119,-4},{108,-4}},
                                               color={0,0,127}));
  connect(fan1.flange_a1, simpleMotor1.flange1)
    annotation (Line(points={{157.5,28},{140.4,28}}, color={0,0,0}));
  connect(inverter.dc_p, circuitBreaker1.n1)
    annotation (Line(points={{86,34},{73.8,34}}, color={0,0,255}));
  connect(circuitBreaker2.n1, inverter.dc_n)
    annotation (Line(points={{73.8,22},{86,22}}, color={0,0,255}));
  connect(pwm1.notFire, inverter.fire_p)
    annotation (Line(points={{90,7},{90,16}}, color={255,0,255}));
  connect(dcdc.dc_p1, dC_Battery.p1) annotation (Line(points={{-4,34},{-12,34},
          {-12,32},{-20,32}}, color={0,0,255}));
  connect(dcdc.dc_n1, dC_Battery.n1) annotation (Line(points={{-4,22},{-12,22},
          {-12,24},{-20,24}}, color={0,0,255}));
  connect(energyAnalysis.batteryBus, dC_Battery.batteryBus1) annotation (Line(
      points={{-42,64},{-42,30},{-32,30}},
      color={0,255,0},
      thickness=0.5));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{200,
            100}})),
    Icon(coordinateSystem(extent={{-60,-60},{200,100}},   preserveAspectRatio=false), graphics),
    experiment(Interval=0.1, __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Battery_system;
