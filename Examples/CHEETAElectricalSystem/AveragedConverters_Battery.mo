within CHEETA.Examples.CHEETAElectricalSystem;
model AveragedConverters_Battery
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
  Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage2
                                        hTS_Piline3_1(
                                               l=1,
    I_crit=1100,
    R_L=10,
    G_d=0)
    annotation (Placement(transformation(extent={{-14,24},{2,32}})));
  Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage2
                                        hTS_Piline3_2(
                                                l=1,
    I_crit=1100,
    R_L=10,
    G_d=0)
    annotation (Placement(transformation(extent={{-14,12},{2,20}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=1)
                                                        annotation (Placement(
        transformation(extent={{-8,-60},{-28,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 20)
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
  Aircraft.Electrical.Machines.ElectricDrives.SimpleSpeedDrive_Variable
    simpleSpeedDrive_Variable
    annotation (Placement(transformation(extent={{70,10},{90,30}})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Blocks.EnergyAnalysis energyAnalysis1(
      useBusConnector=true)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,48})));
  Aircraft.Electrical.Battery.DC_Battery dC_Battery1
                                                    annotation (Placement(
        transformation(
        extent={{-12,-11},{12,11}},
        rotation=270,
        origin={-65,22})));
equation
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{152,22},{187,22}},
                                              color={0,0,0}));
  connect(circuitBreaker1.p1, hTS_Piline3_1.pin_n)
    annotation (Line(points={{18,28},{3,28}}, color={0,0,255}));
  connect(circuitBreaker2.p1, hTS_Piline3_2.pin_n)
    annotation (Line(points={{18,16},{3,16}}, color={0,0,255}));
  connect(hTS_Piline3_1.port_a, thermalConductor.port_a)
    annotation (Line(points={{-5.8,24},{-5.8,-50},{-8,-50}},
                                                         color={191,0,0}));
  connect(hTS_Piline3_2.port_a, thermalConductor.port_a)
    annotation (Line(points={{-5.8,12},{-5.8,-50},{-8,-50}},
                                                         color={191,0,0}));
  connect(thermalConductor.port_b, fixedTemperature.port)
    annotation (Line(points={{-28,-50},{-40,-50}}, color={191,0,0}));
  connect(converterVoltageInput.p2, hTS_Piline3_1.pin_p)
    annotation (Line(points={{-20,28},{-15,28}}, color={0,0,255}));
  connect(hTS_Piline3_2.pin_p, converterVoltageInput.n2)
    annotation (Line(points={{-15,16},{-20,16}}, color={0,0,255}));
  connect(converterVoltageInput.v, voltage_ce.y)
    annotation (Line(points={{-22,10},{-22,-10},{-33,-10}}, color={0,0,127}));
  connect(circuitBreaker1.n1, simpleSpeedDrive_Variable.dc_p1) annotation (Line(
        points={{37.8,28},{54,28},{54,24.2},{71.6,24.2}}, color={0,0,255}));
  connect(circuitBreaker2.n1, simpleSpeedDrive_Variable.dc_n1)
    annotation (Line(points={{37.8,16},{71.6,16}}, color={0,0,255}));
  connect(multiSensor.flange_a, simpleSpeedDrive_Variable.flange1) annotation (
      Line(points={{140,22},{114,22},{114,20.4},{89,20.4}}, color={0,0,0}));
  connect(tauRef.y, simpleSpeedDrive_Variable.wref)
    annotation (Line(points={{45,56},{80,56},{80,29.4}}, color={0,0,127}));
  connect(energyAnalysis1.batteryBus,dC_Battery1. batteryBus1) annotation (Line(
      points={{-86,38},{-86,27.04},{-76,27.04}},
      color={0,255,0},
      thickness=0.5));
  connect(dC_Battery1.p1, converterVoltageInput.p1) annotation (Line(points={{
          -54,28.24},{-46,28.24},{-46,28},{-40,28}}, color={0,0,255}));
  connect(converterVoltageInput.n1, dC_Battery1.n1) annotation (Line(points={{
          -40,16},{-48,16},{-48,16.24},{-54,16.24}}, color={0,0,255}));
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
end AveragedConverters_Battery;
