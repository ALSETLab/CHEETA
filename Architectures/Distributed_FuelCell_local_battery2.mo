within CHEETA.Architectures;
model Distributed_FuelCell_local_battery2
  "iii. Distributed architecture, with one set of fuel cell stacks routed to one individual propulsor motor (power augmented from battery DC bus)"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{108,4},
            {120,16}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{-96,34},
            {-84,46}})));
  Aircraft.Mechanical.Loads.Fan fan(J=10)
    annotation (Placement(transformation(extent={{130,6},{138,14}})));
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.SpeedFOC  electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed
        data(redeclare CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_1MW
          machineData)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_1MW
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false)
                    annotation (Placement(transformation(extent={{76,0},{96,20}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{46,-26},{66,-6}})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-16,-12},{16,12}},
        rotation=270,
        origin={-68,8})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-58,-24},{-38,-4}})));
  Aircraft.Electrical.BusExt busExt(nn=1, np=3)
    annotation (Placement(transformation(extent={{2,-22},{0,54}})));
  Modelica.Blocks.Sources.BooleanExpression fuelCellOn(y=EMS.FuelCellState)
    annotation (Placement(transformation(extent={{-98,-24},{-78,-4}})));
  Aircraft.Controls.MasterSystemControl masterSystemControl
    annotation (Placement(transformation(extent={{-124,30},{-104,50}})));
  Modelica.Blocks.Sources.RealExpression ReferenceSpeed(y=EMS.Speed)
    annotation (Placement(transformation(extent={{48,20},{68,40}})));
  Hydrogen.Sources.GasPressureBoundary air_Inlet(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    N=1,
    X_set={0.01,0.768,0.222},
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{-124,-82},{-104,
            -62}})));
  Hydrogen.Sources.GasPressureBoundary air_Outlet(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    X_set={0,0.768,0.232},
    N=1,
    usePressureInput=false,
    p_set=190000,
    T_set=343.15) annotation (Placement(transformation(extent={{0,-68},{-20,-48}})));
  Hydrogen.Sources.GasPressureBoundary hydrogen_Inlet(
    redeclare package Medium = Hydrogen.Media.Fuel.MixtureGasH2,
    N=1,
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{-124,-52},{-104,
            -32}})));
  Hydrogen.CellStacks.PEM.StackWithCooling_DetailedMembrane
                                                   stack(
    N=5,
    N_cAn=10,
    N_pAn=3,
    N_cCat=10,
    N_pCat=3,
    N_cCool=10,
    m_flowNominal=0.1,
    m_flowInitCathode=0.01,
    useHeatTransfer=false,
    m_flowNominalCathode=0.065,
    N_pCool=10,
    alpha_set=1600,
    m_flowInitAnode=0.01,
    C_cell=1e4,
    T_init=333.15,
    p_bInitCathode=190000,
    w_a=0.55,
    h_a=0.35,
    w_cAn=0.001,
    h_cAn=0.001,
    w_cCat=0.001,
    h_cCat=0.001,
    w_cCool=0.002,
    h_cCool=0.002) annotation (Placement(transformation(extent={{-66,-64},{-46,
            -44}})));
  DassaultSystemes.Fluid.Sources.LiquidFlowBoundary coolant_Inlet(
    redeclare package Medium =
        DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47,
    m_flowSet=0.025,
    useMassFlowInput=true,
    T_set=293.15) annotation (Placement(transformation(extent={{-86,-88},{-66,
            -68}})));
  DassaultSystemes.Fluid.Sources.LiquidPressureBoundary coolant_Outlet(N=1,
      redeclare package Medium =
        DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47)
    annotation (Placement(transformation(extent={{-22,-88},{-42,-68}})));
  Modelica.Blocks.Sources.Ramp m_flow_Ramp(
    duration=2,
    startTime=500,
    height=-0.01,
    offset=0.04)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-114,-102})));
              ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Speed
                                                                          controller(data(
      kp_FW=1,
      Ti_FW=4e-3,
      redeclare
        ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Records.Data.Linear.Industrial_550W
        machineData,
      i_s_max=1.1))
    annotation (Placement(transformation(extent={{44,-50},{64,-30}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
                                                             modulationMethod
    annotation (Placement(transformation(extent={{78,-50},{98,-30}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.LossyLinearized
                                                         inverter(
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Losses.Records.Data.LossyLinearized.IGBT_ModuleSemikron_600V_6A
      data,
    fs=20e3)
    annotation (Placement(transformation(extent={{118,-50},{138,-30}})));
              ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
                                                                               machine(
      useThermalPort=false)
    annotation (Placement(transformation(extent={{158,-50},{178,-30}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-82,-62},{-72,-52}})));
  Aircraft.Electrical.HTS.HTS_filmboiling2 hTS_filmboiling2_1
    annotation (Placement(transformation(extent={{24,20},{40,12}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=1)
                                                        annotation (Placement(
        transformation(extent={{90,34},{110,54}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{152,34},{132,54}})));
  Modelica.Blocks.Sources.Constant const(k=77)
    annotation (Placement(transformation(extent={{198,34},{178,54}})));
equation
  connect(multiSensor.flange_b, fan.flange_a1)
    annotation (Line(points={{120,10},{129,10}},
                                              color={0,0,0}));
  connect(multiSensor.flange_a, electricDrive.flange) annotation (Line(points={{108,10},
          {96,10}},                             color={0,0,0}));
  connect(electricDrive.pin_n, ground1.p)
    annotation (Line(points={{76,4},{56,4},{56,-6}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, ground.p) annotation (Line(points={{-58.1818,
          4.23529},{-48,4.23529},{-48,-4}}, color={0,0,255}));
  connect(fuelCellOn.y, battery_FC_Charging.u1) annotation (Line(points={{-77,-14},
          {-66.9091,-14},{-66.9091,-4.23529}},
                                       color={255,0,255}));
  connect(ReferenceSpeed.y, electricDrive.desiredSpeed)
    annotation (Line(points={{69,30},{86,30},{86,22}}, color={0,0,127}));
  connect(stack.anodePort_a,hydrogen_Inlet. port[1])
    annotation (Line(points={{-66,-49.2},{-96,-49.2},{-96,-42},{-104,-42}},
                                                                       color={0,178,169}));
  connect(stack.cathodePort_a,air_Inlet. port[1])
    annotation (Line(points={{-66,-58},{-96,-58},{-96,-72},{-104,-72}},
                                                                   color={0,178,169}));
  connect(stack.cathodePort_b,air_Outlet. port[1]) annotation (Line(points={{-46,-58},
          {-20,-58}},                                                                                      color={0,178,169}));
  connect(stack.coolingPort_a,coolant_Inlet. port) annotation (Line(points={{-62,-64},
          {-62,-78},{-66,-78}},                                                                           color={255,127,36}));
  connect(coolant_Outlet.port[1],stack. coolingPort_b)
    annotation (Line(points={{-42,-78},{-50,-78},{-50,-64},{-49.8,-64}},
                                                                color={255,127,36}));
  connect(m_flow_Ramp.y,coolant_Inlet. m_flowIn)
    annotation (Line(points={{-103,-102},{-90,-102},{-90,-70},{-86,-70}},
                                                                       color={0,0,127}));
  connect(stack.pin_p, busExt.p[2]) annotation (Line(points={{-46,-52},{-34,-52},
          {-34,16},{0,16}}, color={0,0,255}));
  connect(battery_FC_Charging.p1, busExt.p[3]) annotation (Line(points={{
          -58.1818,13.6471},{-40.4546,13.6471},{-40.4546,31.2},{-2.22045e-16,
          31.2}},
        color={0,0,255}));
  connect(controller.actuatingVoltages, modulationMethod.phaseVoltages)
    annotation (Line(points={{65,-40},{76,-40}}, color={0,0,127}));
  connect(inverter.normalizedPhaseVoltages, modulationMethod.normalizedPhaseVoltages)
    annotation (Line(points={{116,-40},{99,-40}}, color={0,0,127}));
  connect(machine.plug_p, inverter.plug)
    annotation (Line(points={{158,-40},{138,-40}}, color={0,0,255}));
  connect(ground2.p, stack.pin_n)
    annotation (Line(points={{-77,-52},{-66.2,-52}}, color={0,0,255}));
  connect(busExt.n[1], hTS_filmboiling2_1.pin_p)
    annotation (Line(points={{2,16},{23,16}}, color={0,0,255}));
  connect(hTS_filmboiling2_1.pin_n, electricDrive.pin_p)
    annotation (Line(points={{41,16},{76,16}}, color={0,0,255}));
  connect(prescribedTemperature.port,thermalConductor. port_b)
    annotation (Line(points={{132,44},{110,44}},color={191,0,0}));
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{154,44},{177,44}}, color={0,0,127}));
  connect(thermalConductor.port_a, hTS_filmboiling2_1.port_a)
    annotation (Line(points={{90,44},{32.2,44},{32.2,20}}, color={191,0,0}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-160},{
            220,80}})),
    Icon(coordinateSystem(extent={{-140,-160},{220,80}},  preserveAspectRatio=false), graphics),
    experiment(
      StopTime=100,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Distributed_FuelCell_local_battery2;
