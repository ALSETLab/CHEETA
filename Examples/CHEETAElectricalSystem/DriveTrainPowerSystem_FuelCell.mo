within CHEETA.Examples.CHEETAElectricalSystem;
model DriveTrainPowerSystem_FuelCell
  "AIM with forced cooling modeled in a simple thermal model"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Constant
                               tauRef(k=1000)
               annotation (Placement(transformation(extent={{24,46},{44,66}})));
  ElectrifiedPowertrains.ElectricDrives.Common.Blocks.EnergyAnalyser driveEfficiencyComputation(useBusConnector=true)
    annotation (Placement(transformation(extent={{86,-18},{106,2}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.ElectricQuantities machineVariables(terminalConnection=electricDrive.machine.data.terminalConnection)
    annotation (Placement(transformation(extent={{116,-18},{136,2}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{140,16},
            {152,28}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Data.Speed.MSL_18p5kW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage machine(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW data),
    redeclare ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=true)
                    annotation (Placement(transformation(extent={{46,12},{66,32}})));
  replaceable Atmospheres.CoolingMedium.Air_30degC            coolingMedium constrainedby
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
  Aircraft.Electrical.PowerElectronics.Converters.DCDC.BoostConverter
                                                         dcdc(
    L=0.001,
    i=0.1,
    C=0.001,
    v(start=1000) = 1000)                                     annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-28,22})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f(displayUnit="Hz") = 100)
    annotation (Placement(transformation(extent={{-38,-20},{-18,0}})));
  ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.ThreeMassTEFC machineThermal(
      redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.Records.Data.ForcedCoolingThreeMassEstimation2.MSL_18p5kW
      data)             annotation (Placement(transformation(extent={{76,46},{
            96,66}})));
  Modelica.Thermal.FluidHeatFlow.Sources.Ambient coolingMediaSource(
    medium=coolingMedium,
    constantAmbientPressure=100000,
    constantAmbientTemperature=313.15) annotation (Placement(transformation(extent={{52,70},
            {40,82}})));
  Modelica.Thermal.FluidHeatFlow.Sensors.TemperatureSensor inletTemperature(medium=
        coolingMedium)
    annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=270,
        origin={66,86})));
  Modelica.Thermal.FluidHeatFlow.Sensors.VolumeFlowSensor volumeFlowSensor(medium=
        coolingMedium)
    annotation (Placement(transformation(extent={{130,72},{122,80}})));
  ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.FourElements inverterThermal(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Data.FourElements.FF150R17KE4
      data)
    annotation (Placement(transformation(extent={{-4,46},{16,66}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature inverterAmbientTemperature
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={6,78})));
  Modelica.Thermal.FluidHeatFlow.Sources.IdealPump linearFan1(
    medium=coolingMedium,
    dp0(displayUnit="Pa") = 1000,
    wNominal=electricDrive.machine.data.w_nom,
    V_flow0=10/60)            annotation (Placement(transformation(extent={{100,70},
            {112,82}})));
  Modelica.Thermal.FluidHeatFlow.Sources.Ambient coolingMediaSink(
    medium=coolingMedium,
    constantAmbientPressure=100000,
    constantAmbientTemperature=313.15) annotation (Placement(transformation(extent={{158,70},
            {170,82}})));
  Hydrogen.Sources.GasPressureBoundary airSource(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    p_set=200000,
    T_set=343.15,
    X_set={0.01,0.768,0.222},
    N=1)          annotation (Placement(transformation(extent={{-120,-20},{-100,
            0}})));
  Hydrogen.Sources.GasPressureBoundary airSink(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    X_set={0,0.768,0.232},
    p_set=199000,
    T_set=343.15,
    usePressureInput=false,
    N=2)          annotation (Placement(transformation(extent={{-120,60},{-100,
            80}})));
  Hydrogen.Sources.GasPressureBoundary fuelSource(
    redeclare package Medium = Hydrogen.Media.Fuel.MixtureGasH2,
    p_set=200000,
    T_set=343.15,
    N=1)          annotation (Placement(transformation(extent={{-120,20},{-100,
            40}})));
   Hydrogen.CellStacks.PEM.Stack_NominalMembrane stack(
    N=5,
    N_cAn=10,
    N_pAn=3,
    N_cCat=10,
    N_pCat=3,
    m_flowNominalCathode=0.065,
    w_a=0.55,
    h_a=0.35,
    w_cAn=0.001,
    h_cAn=0.001,
    w_cCat=0.001,
    h_cCat=0.001) annotation (Placement(transformation(extent={{-10,-10},{10,10}},

        rotation=90,
        origin={-58,22})));
  inner Hydrogen.Common.SystemSettings hydrogenSettings(initType=Hydrogen.Common.Types.InitType.FixedInitial)
    annotation (Placement(transformation(extent={{180,40},{200,60}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-58,-16},{-38,4}})));
equation
  connect(driveEfficiencyComputation.electricDriveBus, electricDrive.electricDriveBus)
    annotation (Line(
      points={{96,-18},{96,-28},{56,-28},{56,12}},
      color={0,86,166},
      thickness=0.5));
  connect(machineVariables.electricDriveBus, driveEfficiencyComputation.electricDriveBus)
    annotation (Line(
      points={{126,-18},{126,-28},{96,-28},{96,-18}},
      color={0,86,166},
      thickness=0.5));
  connect(electricDrive.flange, multiSensor.flange_a) annotation (Line(points={{66,22},
          {140,22}},                                                                                  color={0,0,0}));
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{152,22},{187,22}},
                                              color={0,0,0}));
  connect(electricDrive.desiredSpeed, tauRef.y)
    annotation (Line(points={{56,34},{56,56},{45,56}},color={0,0,127}));
  connect(circuitBreaker1.n1, electricDrive.pin_p)
    annotation (Line(points={{37.8,28},{46,28}}, color={0,0,255}));
  connect(electricDrive.pin_n, circuitBreaker2.n1)
    annotation (Line(points={{46,16},{37.8,16}}, color={0,0,255}));
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
  connect(dcdc.fire_p,pwm. fire)
    annotation (Line(points={{-34,10},{-34,1}},    color={255,0,255}));
  connect(dcdc.dc_p2, stekly_ExtraHeatGeneration.pin_p)
    annotation (Line(points={{-18,28},{-13,28}}, color={0,0,255}));
  connect(stekly_ExtraHeatGeneration1.pin_p, dcdc.dc_n2)
    annotation (Line(points={{-13,16},{-18,16}}, color={0,0,255}));
  connect(machineThermal.flange, electricDrive.flange)
    annotation (Line(points={{96,56},{106,56},{106,22},{66,22}}, color={0,0,0}));
  connect(inverterAmbientTemperature.port,inverterThermal. heatPort_heatSink)
    annotation (Line(points={{6,72},{6,66}},              color={191,0,0}));
  connect(inletTemperature.flowPort,coolingMediaSource. flowPort)
    annotation (Line(points={{66,82},{66,76},{52,76}}, color={255,0,0}));
  connect(coolingMediaSource.flowPort,machineThermal. flowPort_a)
    annotation (Line(points={{52,76},{78,76},{78,66}}, color={255,0,0}));
  connect(volumeFlowSensor.flowPort_a,coolingMediaSink. flowPort) annotation (Line(points={{130,76},
          {158,76}},                                                                                          color={255,0,0}));
  connect(inverterAmbientTemperature.T,inletTemperature. y)
    annotation (Line(points={{6,85.2},{6,94},{66,94},{66,90.4}},     color={0,0,127}));
  connect(linearFan1.flowPort_b,volumeFlowSensor. flowPort_b)
    annotation (Line(points={{112,76},{122,76}},       color={255,0,0}));
  connect(linearFan1.flange_a,machineThermal. flange)
    annotation (Line(points={{106,70},{106,56},{96,56}},               color={0,0,0}));
  connect(machineThermal.flowPort_b,linearFan1. flowPort_a) annotation (Line(points={{94,66},
          {94,76},{100,76}},                                                                                   color={255,0,0}));
  connect(machineThermal.thermalPort, electricDrive.thermalPortMachine)
    annotation (Line(points={{86,46},{86,40},{62,40},{62,32}}, color={191,0,0}));
  connect(inverterThermal.thermalPortInverter, electricDrive.thermalPortInverter)
    annotation (Line(points={{6,46},{6,36},{50,36},{50,32}}, color={199,0,0}));
  connect(stack.cathodePort_b, airSink.port[1])
    annotation (Line(points={{-54,32},{-54,72},{-100,72}}, color={0,178,169}));
  connect(stack.pin_p, dcdc.dc_p1) annotation (Line(points={{-60,32},{-60,42},{
          -44,42},{-44,26},{-38,26},{-38,28}}, color={0,0,255}));
  connect(dcdc.dc_n1, stack.pin_n) annotation (Line(points={{-38,16},{-46,16},{
          -46,4},{-60,4},{-60,11.8}}, color={0,0,255}));
  connect(stack.cathodePort_a, airSource.port[1]) annotation (Line(points={{-54,
          12},{-54,-10},{-100,-10}}, color={0,178,169}));
  connect(stack.anodePort_a, fuelSource.port[1]) annotation (Line(points={{
          -62.8,12},{-62.8,-2},{-90,-2},{-90,30},{-100,30}}, color={0,178,169}));
  connect(dcdc.dc_n1, ground.p) annotation (Line(points={{-38,16},{-46,16},{-46,
          4},{-48,4}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-80},{200,
            100}})),
    Icon(coordinateSystem(extent={{-140,-80},{200,100}},  preserveAspectRatio=false), graphics),
    experiment(Interval=0.1, __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end DriveTrainPowerSystem_FuelCell;
