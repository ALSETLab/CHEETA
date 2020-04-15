within CHEETA.Examples.CHEETAElectricalSystem;
model AveragedConverters_Battery_EPL
  "AIM with forced cooling modeled in a simple thermal model"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{24,46},{44,66}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{140,16},
            {152,28}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{182,82},{194,94}})));
  Aircraft.Mechanical.Loads.Fan fan
    annotation (Placement(transformation(extent={{188,18},{196,26}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker1(k=200000)
    annotation (Placement(transformation(extent={{18,26},{38,34}})));
  Aircraft.Electrical.CB.CircuitBreaker circuitBreaker2(k=200000)
    annotation (Placement(transformation(extent={{18,14},{38,22}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                        stekly_ExtraHeatGeneration(
                                               l=1, G_d=100)
    annotation (Placement(transformation(extent={{-12,24},{4,32}})));
  Aircraft.Electrical.HTS.Stekly.Stekly_ExtraHeatGeneration
                                        stekly_ExtraHeatGeneration1(
                                                l=1, G_d=100)
    annotation (Placement(transformation(extent={{-12,12},{4,20}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{-8,-60},{-28,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 115)
    annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
  ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.VoltageInputConstantEfficiency
                                                         converterVoltageInput(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.Records.Data.ConstantEfficiency.Constant100percent
      data)                                                   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,22})));
  Modelica.Blocks.Sources.Constant voltage_ce(k=1000)
                                                    annotation (Placement(transformation(extent={{-54,-20},
            {-34,0}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Data.Speed.MSL_18p5kW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{80,14},{100,
            34}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=0,
        origin={-71,-1})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=0,
        origin={-17,7})));
  Aircraft.Electrical.Battery.DC_Battery dC_Battery1
                                                    annotation (Placement(
        transformation(
        extent={{-12,-11},{12,11}},
        rotation=270,
        origin={-75,20})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Blocks.EnergyAnalysis energyAnalysis1(
      useBusConnector=true)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,46})));
equation
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{152,22},{187,22}},
                                              color={0,0,0}));
  connect(circuitBreaker1.p1, stekly_ExtraHeatGeneration.pin_n)
    annotation (Line(points={{18,28},{5,28}}, color={0,0,255}));
  connect(circuitBreaker2.p1, stekly_ExtraHeatGeneration1.pin_n)
    annotation (Line(points={{18,16},{5,16}}, color={0,0,255}));
  connect(stekly_ExtraHeatGeneration.port_a, thermalConductor.port_a)
    annotation (Line(points={{-4,24},{-4,-50},{-8,-50}}, color={191,0,0}));
  connect(stekly_ExtraHeatGeneration1.port_a, thermalConductor.port_a)
    annotation (Line(points={{-4,12},{-4,-50},{-8,-50}}, color={191,0,0}));
  connect(thermalConductor.port_b, fixedTemperature.port)
    annotation (Line(points={{-28,-50},{-40,-50}}, color={191,0,0}));
  connect(converterVoltageInput.p2, stekly_ExtraHeatGeneration.pin_p)
    annotation (Line(points={{-20,28},{-16,28},{-16,28},{-13,28}}, color={0,0,
          255}));
  connect(stekly_ExtraHeatGeneration1.pin_p, converterVoltageInput.n2)
    annotation (Line(points={{-13,16},{-16,16},{-16,16},{-20,16}}, color={0,0,
          255}));
  connect(converterVoltageInput.v, voltage_ce.y)
    annotation (Line(points={{-22,10},{-22,-10},{-33,-10}}, color={0,0,127}));
  connect(electricDrive.pin_p, circuitBreaker1.n1) annotation (Line(points={{80,
          30},{60,30},{60,28},{37.8,28}}, color={0,0,255}));
  connect(circuitBreaker2.n1, electricDrive.pin_n)
    annotation (Line(points={{37.8,16},{80,16},{80,18}}, color={0,0,255}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{45,56},{90,56},{90,36}}, color={0,0,127}));
  connect(multiSensor.flange_a, electricDrive.flange) annotation (Line(points={
          {140,22},{120,22},{120,24},{100,24}}, color={0,0,0}));
  connect(converterVoltageInput.n1, ground.p) annotation (Line(points={{-40,16},
          {-60,16},{-60,8},{-71,8}}, color={0,0,255}));
  connect(stekly_ExtraHeatGeneration1.pin_p, ground1.p)
    annotation (Line(points={{-13,16},{-17,16}}, color={0,0,255}));
  connect(converterVoltageInput.n1, dC_Battery1.n1)
    annotation (Line(points={{-40,16},{-64,16},{-64,14.24}}, color={0,0,255}));
  connect(dC_Battery1.p1, converterVoltageInput.p1) annotation (Line(points={{
          -64,26.24},{-52,26.24},{-52,28},{-40,28}}, color={0,0,255}));
  connect(dC_Battery1.batteryBus1, energyAnalysis1.batteryBus) annotation (Line(
      points={{-86,25.04},{-86,36}},
      color={0,255,0},
      thickness=0.5));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{200,
            100}})),
    Icon(coordinateSystem(extent={{-100,-80},{200,100}},  preserveAspectRatio=false), graphics),
    experiment(Interval=0.1, __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end AveragedConverters_Battery_EPL;
