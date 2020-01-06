within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model FuelCellSystem
  import FuelCell;
    extends Modelon.Icons.Experiment;

  replaceable package Medium_fuel =
      FuelCell.Media.PreDefined.IdealGases.FastReformateLong;
  replaceable package Medium_air =
      FuelCell.Media.PreDefined.IdealGases.FastMoistAir;
  replaceable package Medium_Steam =
      FuelCell.Media.PreDefined.IdealGases.FastPureSteam;
  replaceable package Medium_Water =
      FuelCell.Media.PreDefined.TwoPhase.WaterIF97;
  FuelCell.Sources.GasFlowBoundary flow_cathode(
    redeclare package Medium = Medium_air,
    m_flow=2e-2,
    T=973.15) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={0,-60})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{78,82},{90,94}},     rotation=
           0)));
  Modelica.Electrical.Analog.Sources.RampCurrent current(
    startTime=50,
    offset=10,
    I=90,
    duration=200)
    annotation (Placement(transformation(
        origin={58,106},
        extent={{-8,-8},{8,8}},
        rotation=0)));
  FuelCell.Stacks.SOFC.SOFCStack stack(
    m_flow_nom_an=1e-3,
    m_flow_nom_cath=2e-2,
    p_start_out_anode=anode_volume.pstart,
    p_start_out_cathode=cathode_volume.pstart,
    N=4,
    nbrSubStacks=2,
    n_cell={10,30,10},
    redeclare package Medium_an = Medium_fuel,
    redeclare package Medium_cath = Medium_air,
    p_start_in_anode=anode_volume.pstart + 1e3,
    p_start_in_cathode=cathode_volume.pstart + 10e3,
    m_flow_start_anode=1e-3,
    m_flow_start_cathode=2e-2,
    M_stack=15,
    T_start_in_anode=1073.15,
    T_start_out_anode=1073.15,
    T_start_in_cathode=1073.15,
    T_start_out_cathode=1073.15,
    wallthickness(displayUnit="mm") = 0.01,
    subStack(each positiveFlow_cathode=true),
    feed_Cathode(positiveFlow=true),
    drain_Cathode(positiveFlow=true))
    annotation (Placement(transformation(extent={{34,38},{74,79}})));
  FuelCell.SubComponents.ComponentSummaries.SystemSummary summary(
    N=stack.summary.N,
    fuel_nS=Medium_fuel.nS,
    tAirStkIn=stack.summary.tAirStkIn,
    tFuelStkIn=stack.summary.tFuelStkIn,
    tFuelStkOut=stack.summary.tFuelStkOut,
    tAirStkOut=stack.summary.tAirStkOut,
    tStkOut=stack.summary.tStkOut,
    tStkTopWall=stack.summary.tStkTopWall,
    tStkBottomWall=stack.summary.tStkBottomWall,
    PStkElec=stack.summary.PStkElec,
    QStkHeat=stack.summary.QStkHeat,
    dmAirStkIn=stack.summary.dmAirStkIn,
    dmFuelStkIn=stack.summary.dmFuelStkIn,
    VCstZone=stack.summary.VCstZone,
    PStk=stack.summary.PStk,
    facFuelStkUtil=stack.summary.facFuelStkUtil,
    facFuelStkOutComp=stack.summary.facFuelStkOutComp,
    hBmflow=heatBlock.summary.m_flow_outlet,
    tSteamMix=heatBlock.summary.tSteamMix,
    tGenSteam=heatBlock.summary.tGenSteam,
    tSteamGenLiq=heatBlock.summary.tSteamGenLiq,
    ATRtemp=heatBlock.reformer.T,
    ATRutilCH4=heatBlock.reformer.utilization_CH4,
    ATRutilO2=heatBlock.reformer.utilization_O2,
    ATRthermEff=heatBlock.reformer.therm_eff,
    ATRinletSteamCarb=heatBlock.reformer.steam_carb_ratio,
    ATRinletOxCarb=heatBlock.reformer.oxygen_carb_ratio,
    ATRoutCompMoleFrac=heatBlock.reformer.X_mole)
    annotation (Placement(transformation(extent={{-120,100},{-100,120}})));
  FuelCell.Sources.WaterFlowBoundary water_source(
    redeclare package Medium = Medium_Water,
    m_flow=3.248e-4,
    T=323.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-120,20})));
  FuelCell.Sources.GasFlowBoundary NG_source(
    redeclare package Medium = Medium_fuel,
    X=Medium_fuel.moleToMassFractions({0,0.98,0,0.02,0,0,0}, Medium_fuel.MMX),
    m_flow=2.552e-4,
    T=573.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-120,66})));

  FuelCell.Sources.GasFlowBoundary ATR_air_source(
    redeclare package Medium = Medium_air,
    m_flow=4.55e-4,
    T=373.15)
    annotation (Placement(transformation(extent={{-130,-30},{-110,-10}})));
  FuelCell.Sources.GasPressureBoundary exhaust_sink(
    redeclare package Medium = Medium_air,
    p=101300,
    T=573.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-120,-60})));
  FuelCell.Examples.SOFC.SubSystems.FuelPreProcessor heatBlock(
    redeclare package Medium_Air = Medium_air,
    redeclare package Medium_Fuel = Medium_fuel,
    redeclare package Medium_Water = Medium_Water,
    redeclare package Medium_Steam = Medium_Steam,
    fuelLoss(dp0=100, m_flow0=5e-4),
    steamGeneratorHX(
      staticHXChannel(m_flow_nom=2.1e-2),
      steamGenerator(kc=55),
      T_start=1073.15),
    gasMix(pstart=exhaust_volume.pstart + 1e3),
    NGMix(V_tot=0.001, pstart=heatBlock.Initialization_Record.FuelHeat_pstart_sec),
    NGLoss(m_flow0=2e-4),
    Initialization_Record(
      steamMix_m_flow_start=6e-4,
      Tstart=Modelica.SIunits.Conversions.from_degC(800),
      ATR_pstart=154500,
      steamMix_pstart=155000,
      steamGen_pstart=102400,
      FuelHeat_pstart_prim=122000,
      FuelHeat_pstart_sec=155500,
      AirHeat_pstart_prim=150000,
      AirHeat_pstart_sec=155000),
    steamMix_TZ(Tstart=773.15),
    reformer(Tstart=873.15),
    Geometry_Record(
      FuelHeat_Aheat=0.02,
      AirHeat_Aheat=0.02,
      FuelHeat_m_flow_nom_prim=20e-3,
      Hotgas_m_flow_nom=10e-4,
      FuelHeat_m_flow_nom_sec=2.5e-4,
      ATR_scale_cat=5,
      steamGen_dp_nom_gas=1000,
      AirHeat_kc=55))
    annotation (Placement(transformation(extent={{-66,38},{-26,78}})));

  FuelCell.FlowResistances.TurbulentLoss anode_loss(
    redeclare package Medium = Medium_fuel,
    d0=1,
    m_flow0=1e-3,
    dp0=100) annotation (Placement(transformation(extent={{-10,51},{4,65}})));
  FuelCell.FlowResistances.TurbulentLoss exhaust_loss(
    d0=1,
    redeclare package Medium = Medium_air,
    m_flow0=10e-3,
    dp0=100) annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-60,-20})));
  FuelCell.Volumes.GasVolume_pTX exhaust_volume(
    N_feed=1,
    N_drain=1,
    V_tot=0.001,
    redeclare package Medium = Medium_air,
    pstart=exhaust_sink.p + exhaust_loss.dp0,
    Tstart=773.15) annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-60,3})));
  FuelCell.Volumes.GasVolume_pTX anode_volume(
    redeclare package Medium = Medium_fuel,
    N_feed=1,
    V_tot=0.001,
    initOpt=FuelCell.Internal.Choices.InitOptions.initialValues,
    Xstart={0.5,0.4,0,0,0.1,0,0},
    pstart=fuel_mix.pstart + 1e3,
    Tstart=1073.15,
    N_drain=1)
    annotation (Placement(transformation(extent={{84,58},{96,70}})));
  FuelCell.Volumes.GasVolume_pTX cathode_volume(
    redeclare package Medium = Medium_air,
    V_tot=0.001,
    pstart=air_mix.pstart + 1e3,
    Tstart=1073.15,
    N_drain=1,
    N_feed=1) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={106,50})));
  FuelCell.HeatExchangers.GasGas.EpsNTU air_heater(
    redeclare package PrimaryMedium = Medium_air,
    redeclare package SecondaryMedium = Medium_air,
    redeclare function epsFun =
        Modelon.ThermoFluid.HeatExchangers.Functions.counterFlowEps,
    L_prim=0.1,
    Dhyd_prim=0.01,
    A_prim=0.0001,
    L_sec=0.1,
    Dhyd_sec=0.01,
    A_sec=0.0001,
    A_heat_prim=0.02,
    A_heat_sec=0.02,
    A_heat=0.02,
    redeclare record WallMaterial =
        Modelon.ThermoFluid.Solids.ConstantProperties.SteelV2A,
    redeclare model Friction_prim =
        Modelon.ThermoFluid.FlowChannels.PipeResistances.SinglePhase.LinearOperatingPointLoss
        (
        d0=0.5,
        dp0(displayUnit="kPa") = 100,
        m_flow0=2e-2),
    redeclare model Friction_sec =
        Modelon.ThermoFluid.FlowChannels.PipeResistances.SinglePhase.LinearOperatingPointLoss
        (
        d0=0.5,
        dp0(displayUnit="kPa") = 100,
        m_flow0=2e-2)) annotation (Placement(transformation(
        extent={{-9,9},{9,-9}},
        rotation=90,
        origin={25,-15})));
  FuelCell.Burners.MetalBurner metal_burner(
    redeclare package Medium_fuel = Medium_fuel,
    redeclare package Medium = Medium_air,
    V=0.003,
    startBurning=true,
    initOpt=FuelCell.Internal.Choices.InitOptions.initialValues,
    M_w=0.5,
    pstart=110000,
    Tstart=573.15) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={76,-88})));
  FuelCell.Volumes.GasVolume_pTX fuel_mix(
    redeclare package Medium = Medium_fuel,
    N_feed=1,
    N_drain=1,
    initOpt=FuelCell.Internal.Choices.InitOptions.initialValues,
    Xstart={0.5,0.4,0,0,0.1,0,0},
    V_tot(displayUnit="l") = 0.001,
    pstart=metal_burner.pstart + fuel_loss.dp0,
    Tstart=1073.15)
    annotation (Placement(transformation(extent={{28,8},{40,20}})));
  FuelCell.FlowResistances.TurbulentLoss fuel_loss(
    redeclare package Medium = Medium_fuel,
    m_flow0=1e-3,
    d0=1,
    dp0=10000) annotation (Placement(transformation(extent={{56,7},{70,21}})));
  FuelCell.Volumes.GasVolume_pTX air_mix(
    redeclare package Medium = Medium_air,
    N_feed=1,
    N_drain=1,
    V_tot=0.001,
    pstart=metal_burner.pstart + air_loss.dp0,
    Tstart=973.15) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={138,-28})));
  FuelCell.FlowResistances.TurbulentLoss air_loss(
    redeclare package Medium = Medium_air,
    m_flow0=2e-2,
    dp0(displayUnit="kPa") = 100,
    d0=1) annotation (Placement(transformation(extent={{58,-35},{72,-21}})));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=true) annotation (
     Placement(transformation(
        extent={{-7,7},{7,-7}},
        rotation=180,
        origin={107,-60})));
  Modelica.Blocks.Interaction.Show.RealValue realValue(use_numberPort=false,
      number=stack.pin_p.v - stack.pin_n.v)
    annotation (Placement(transformation(extent={{-26,104},{8,142}})));
equation
  connect(current.n,ground. p) annotation (Line(
      points={{66,106},{84,106},{84,94}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(current.p,stack. pin_p) annotation (Line(
      points={{50,106},{44,106},{44,75.72}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(current.n,stack. pin_n) annotation (Line(
      points={{66,106},{66,75.72},{64,75.72}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(metal_burner.ignition, booleanConstant.y) annotation (Line(
      points={{85,-82},{88,-82},{88,-60},{99.3,-60}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(heatBlock.drain_NGHeat, exhaust_volume.feed[1]) annotation (Line(
      points={{-60,36},{-60,9.3}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(exhaust_volume.drain[1], exhaust_loss.feed) annotation (Line(
      points={{-60,-3.3},{-60,-13.7}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(heatBlock.drain_Reformate, anode_loss.feed) annotation (Line(
      points={{-26,58},{-9.3,58}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(stack.drain_an, anode_volume.feed[1]) annotation (Line(
      points={{74,62.6},{74,64},{84.6,64}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(heatBlock.drain_ATRHeat, fuel_mix.feed[1]) annotation (Line(
      points={{-40,36},{-40,14},{28.6,14}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(fuel_mix.drain[1], fuel_loss.feed) annotation (Line(
      points={{39.4,14},{56.7,14}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(air_mix.drain[1], air_loss.feed) annotation (Line(
      points={{143.4,-28},{58.7,-28}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(air_heater.drain_prim, air_mix.feed[1]) annotation (Line(
      points={{30.4,-23.1},{31,-23.1},{31,-28},{132.6,-28}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(heatBlock.feed_NG, NG_source.fluidPort)
    annotation (Line(points={{-68,66},{-111,66}}, color={255,128,0}));
  connect(heatBlock.feed_Water, water_source.fluidPort) annotation (Line(
        points={{-68,58},{-96,58},{-96,20},{-111,20}}, color={0,0,255}));
  connect(ATR_air_source.fluidPort, heatBlock.feed_ATRAir) annotation (Line(
        points={{-111,-20},{-86,-20},{-86,50},{-68,50}}, color={255,128,0}));
  connect(exhaust_sink.fluidPort, exhaust_loss.drain) annotation (Line(points=
         {{-111,-60},{-60,-60},{-60,-26.3}}, color={255,128,0}));
  connect(stack.feed_an, anode_loss.drain) annotation (Line(points={{34,62.6},
          {14,62.6},{14,58},{3.3,58}}, color={255,128,0}));
  connect(heatBlock.feed_ATRHeat, anode_volume.drain[1]) annotation (Line(
        points={{-32,36},{124,36},{124,64},{95.4,64}}, color={255,128,0}));
  connect(air_heater.feed_prim, cathode_volume.drain[1]) annotation (Line(
        points={{30.4,-6.9},{30.4,-2},{120,-2},{120,50},{111.4,50}}, color={
          255,128,0}));
  connect(air_heater.drain_sec, stack.feed_cath) annotation (Line(points={{
          19.6,-6.9},{19.6,50.3},{34,50.3}}, color={255,128,0}));
  connect(cathode_volume.feed[1], stack.drain_cath) annotation (Line(points={
          {100.6,50},{84,50},{84,50.3},{74,50.3}}, color={255,128,0}));
  connect(metal_burner.gas_out, heatBlock.feed_SteamHeat) annotation (Line(
        points={{67,-88},{-49.2,-88},{-49.2,36}}, color={255,128,0}));
  connect(air_loss.drain, metal_burner.air_in) annotation (Line(points={{71.3,
          -28},{128,-28},{128,-88},{85,-88}}, color={255,128,0}));
  connect(air_heater.feed_sec, flow_cathode.fluidPort) annotation (Line(
        points={{19.6,-23.1},{19.6,-60},{9,-60}}, color={255,128,0}));
  connect(fuel_loss.drain, metal_burner.fuel_in) annotation (Line(points={{
          69.3,14},{80,14},{80,-79}}, color={255,128,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-150,-150},
            {150,150}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-150,-150},{150,150}})),
    experiment(StopTime=4000));
end FuelCellSystem;
