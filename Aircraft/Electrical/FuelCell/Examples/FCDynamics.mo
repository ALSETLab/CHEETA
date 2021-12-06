within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model FCDynamics "Stack with detailed PEM membrane model and Cooling"
  extends DymolaModels.Icons.Basic.Example;
  Hydrogen.Sources.GasPressureBoundary air_Inlet(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    N=1,
    X_set={0.01,0.768,0.222},
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{-60,-12},{-40,8}})));
  Hydrogen.Sources.GasPressureBoundary air_Outlet(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    X_set={0,0.768,0.232},
    N=1,
    usePressureInput=false,
    p_set=190000,
    T_set=343.15) annotation (Placement(transformation(extent={{60,0},{40,20}})));
  Hydrogen.Sources.GasPressureBoundary hydrogen_Inlet(
    redeclare package Medium = Hydrogen.Media.Fuel.MixtureGasH2,
    N=1,
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{-14,52},
            {-26,40}},                                                                                          rotation=0)));
  inner Hydrogen.Common.SystemSettings hydrogenSettings(initType=Hydrogen.Common.Types.InitType.FixedInitial)
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Hydrogen.CellStacks.PEM.StackWithCooling_DetailedMembrane stack60(
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
    redeclare record FuelCellData =
        Hydrogen.Membranes.Records.Data.Generic.Default (T_nom=333.15),
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
    h_cCool=0.002)
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  DassaultSystemes.Fluid.Sources.LiquidFlowBoundary coolant_Inlet(
    redeclare package Medium =
      DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47,
    m_flowSet=0.025,
    useMassFlowInput=false,
    T_set=293.15) annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
  DassaultSystemes.Fluid.Sources.LiquidPressureBoundary coolant_Outlet(N=1, redeclare
      package Medium =
        DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47)
    annotation (Placement(transformation(extent={{40,-40},{20,-20}})));
public
  Modelica.Blocks.Sources.Step  step1(
    height=9,
    startTime=10,
    offset=1)   annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={10,74})));

  Modelica.Electrical.Analog.Basic.VariableResistor resistor1
                                                             annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={10,38})));
  Hydrogen.Sources.GasPressureBoundary air_Inlet1(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    N=1,
    X_set={0.01,0.768,0.222},
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{-214,-20},{-194,
            0}})));
  Hydrogen.Sources.GasPressureBoundary air_Outlet1(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    X_set={0,0.768,0.232},
    N=1,
    usePressureInput=false,
    p_set=190000,
    T_set=343.15) annotation (Placement(transformation(extent={{-94,0},{-114,20}})));
  Hydrogen.Sources.GasPressureBoundary hydrogen_Inlet1(
    redeclare package Medium = Hydrogen.Media.Fuel.MixtureGasH2,
    N=1,
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{-214,20},{-194,
            40}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(transformation(extent={{-168,52},
            {-180,40}},                                                                                         rotation=0)));
  Hydrogen.CellStacks.PEM.StackWithCooling_DetailedMembrane stack80(
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
    redeclare record FuelCellData =
        Hydrogen.Membranes.Records.Data.Generic.Default (T_nom=353.15),
    C_cell=1e4,
    p_bInitCathode=190000,
    w_a=0.55,
    h_a=0.35,
    w_cAn=0.001,
    h_cAn=0.001,
    w_cCat=0.001,
    h_cCat=0.001,
    w_cCool=0.002,
    h_cCool=0.002)
    annotation (Placement(transformation(extent={{-154,0},{-134,20}})));
  DassaultSystemes.Fluid.Sources.LiquidFlowBoundary coolant_Inlet1(
    redeclare package Medium =
        DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47,
    m_flowSet=0.025,
    useMassFlowInput=false,
    T_set=293.15) annotation (Placement(transformation(extent={{-174,-40},{-154,
            -20}})));
  DassaultSystemes.Fluid.Sources.LiquidPressureBoundary coolant_Outlet1(N=1,
      redeclare package Medium =
        DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47)
    annotation (Placement(transformation(extent={{-114,-40},{-134,-20}})));
public
  Modelica.Blocks.Sources.Step  step2(
    height=9,
    startTime=10,
    offset=1)   annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-144,74})));

  Modelica.Electrical.Analog.Basic.VariableResistor resistor2
                                                             annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-144,38})));
  Hydrogen.Sources.GasPressureBoundary air_Inlet2(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    N=1,
    X_set={0.01,0.768,0.222},
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{98,-20},{118,0}})));
  Hydrogen.Sources.GasPressureBoundary air_Outlet2(
    redeclare package Medium = Hydrogen.Media.Air.MoistAirMixtureGas,
    X_set={0,0.768,0.232},
    N=1,
    usePressureInput=false,
    p_set=190000,
    T_set=343.15) annotation (Placement(transformation(extent={{218,0},{198,20}})));
  Hydrogen.Sources.GasPressureBoundary hydrogen_Inlet2(
    redeclare package Medium = Hydrogen.Media.Fuel.MixtureGasH2,
    N=1,
    p_set=200000,
    T_set=343.15) annotation (Placement(transformation(extent={{98,20},{118,40}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
                                                 annotation (Placement(transformation(extent={{144,52},
            {132,40}},                                                                                          rotation=0)));
  Hydrogen.CellStacks.PEM.StackWithCooling_DetailedMembrane stack40(
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
    redeclare record FuelCellData =
        Hydrogen.Membranes.Records.Data.Generic.Default (T_nom=313.15),
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
    h_cCool=0.002)
    annotation (Placement(transformation(extent={{158,0},{178,20}})));
  DassaultSystemes.Fluid.Sources.LiquidFlowBoundary coolant_Inlet2(
    redeclare package Medium =
        DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47,
    m_flowSet=0.025,
    useMassFlowInput=false,
    T_set=293.15) annotation (Placement(transformation(extent={{138,-40},{158,
            -20}})));
  DassaultSystemes.Fluid.Sources.LiquidPressureBoundary coolant_Outlet2(N=1,
      redeclare package Medium =
        DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47)
    annotation (Placement(transformation(extent={{198,-40},{178,-20}})));
public
  Modelica.Blocks.Sources.Step  step3(
    height=9,
    startTime=10,
    offset=1)   annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={168,74})));

  Modelica.Electrical.Analog.Basic.VariableResistor resistor3
                                                             annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={168,38})));
  FuelCell_EquationBased_DetailedRohm fuelCell_EquationBased_DetailedRohm
    annotation (Placement(transformation(extent={{254,16},{274,36}})));
  Modelica.Electrical.Analog.Basic.VariableResistor resistor4
                                                             annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={304,10})));
public
  Modelica.Blocks.Sources.Step  step4(
    height=9,
    startTime=10,
    offset=1)   annotation (Placement(transformation(extent={{348,0},{328,20}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
                                                 annotation (Placement(transformation(extent={{294,-38},
            {314,-18}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=
        333.15)
    annotation (Placement(transformation(extent={{246,-20},{266,0}})));
equation
  connect(stack60.anodePort_a, hydrogen_Inlet.port[1]) annotation (Line(points=
          {{0,14.8},{-30,14.8},{-30,30},{-40,30}}, color={0,178,169}));
  connect(stack60.cathodePort_a, air_Inlet.port[1]) annotation (Line(points={{0,
          6},{-30,6},{-30,-2},{-40,-2}}, color={0,178,169}));
  connect(stack60.cathodePort_b, air_Outlet.port[1]) annotation (Line(points={{
          20,6},{34,6},{34,10},{40,10}}, color={0,178,169}));
  connect(stack60.coolingPort_a, coolant_Inlet.port)
    annotation (Line(points={{4,0},{4,-30},{0,-30}}, color={255,127,36}));
  connect(coolant_Outlet.port[1], stack60.coolingPort_b) annotation (Line(
        points={{20,-30},{16,-30},{16,0},{16.2,0}}, color={255,127,36}));
  connect(resistor1.R, step1.y)
    annotation (Line(points={{10,50},{10,63}}, color={0,0,127}));
  connect(resistor1.n, stack60.pin_n) annotation (Line(points={{0,38},{-10,38},
          {-10,12},{-0.2,12}}, color={0,0,255}));
  connect(ground.p, stack60.pin_n) annotation (Line(points={{-20,40},{-20,38},{
          -10,38},{-10,12},{-0.2,12}}, color={0,0,255}));
  connect(resistor1.p, stack60.pin_p) annotation (Line(points={{20,38},{30,38},
          {30,12},{20,12}}, color={0,0,255}));
  connect(stack80.anodePort_a, hydrogen_Inlet1.port[1]) annotation (Line(points=
         {{-154,14.8},{-184,14.8},{-184,30},{-194,30}}, color={0,178,169}));
  connect(stack80.cathodePort_a, air_Inlet1.port[1]) annotation (Line(points={{
          -154,6},{-184,6},{-184,-10},{-194,-10}}, color={0,178,169}));
  connect(stack80.cathodePort_b, air_Outlet1.port[1]) annotation (Line(points={
          {-134,6},{-120,6},{-120,10},{-114,10}}, color={0,178,169}));
  connect(stack80.coolingPort_a, coolant_Inlet1.port) annotation (Line(points={
          {-150,0},{-150,-30},{-154,-30}}, color={255,127,36}));
  connect(coolant_Outlet1.port[1], stack80.coolingPort_b) annotation (Line(
        points={{-134,-30},{-138,-30},{-138,0},{-137.8,0}}, color={255,127,36}));
  connect(resistor2.R, step2.y)
    annotation (Line(points={{-144,50},{-144,63}}, color={0,0,127}));
  connect(resistor2.n, stack80.pin_n) annotation (Line(points={{-154,38},{-164,
          38},{-164,12},{-154.2,12}}, color={0,0,255}));
  connect(ground1.p, stack80.pin_n) annotation (Line(points={{-174,40},{-174,38},
          {-164,38},{-164,12},{-154.2,12}}, color={0,0,255}));
  connect(resistor2.p, stack80.pin_p) annotation (Line(points={{-134,38},{-124,
          38},{-124,12},{-134,12}}, color={0,0,255}));
  connect(stack40.anodePort_a, hydrogen_Inlet2.port[1]) annotation (Line(points=
         {{158,14.8},{128,14.8},{128,30},{118,30}}, color={0,178,169}));
  connect(stack40.cathodePort_a, air_Inlet2.port[1]) annotation (Line(points={{
          158,6},{128,6},{128,-10},{118,-10}}, color={0,178,169}));
  connect(stack40.cathodePort_b, air_Outlet2.port[1]) annotation (Line(points={
          {178,6},{192,6},{192,10},{198,10}}, color={0,178,169}));
  connect(stack40.coolingPort_a, coolant_Inlet2.port) annotation (Line(points={
          {162,0},{162,-30},{158,-30}}, color={255,127,36}));
  connect(coolant_Outlet2.port[1], stack40.coolingPort_b) annotation (Line(
        points={{178,-30},{174,-30},{174,0},{174.2,0}}, color={255,127,36}));
  connect(resistor3.R, step3.y)
    annotation (Line(points={{168,50},{168,63}}, color={0,0,127}));
  connect(resistor3.n, stack40.pin_n) annotation (Line(points={{158,38},{148,38},
          {148,12},{157.8,12}}, color={0,0,255}));
  connect(ground2.p, stack40.pin_n) annotation (Line(points={{138,40},{138,38},
          {148,38},{148,12},{157.8,12}}, color={0,0,255}));
  connect(resistor3.p, stack40.pin_p) annotation (Line(points={{178,38},{188,38},
          {188,12},{178,12}}, color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm.p1, resistor4.p)
    annotation (Line(points={{274,27},{304,27},{304,20}}, color={0,0,255}));
  connect(resistor4.n, ground3.p)
    annotation (Line(points={{304,0},{304,-18}}, color={0,0,255}));
  connect(step4.y, resistor4.R)
    annotation (Line(points={{327,10},{316,10}}, color={0,0,127}));
  connect(fuelCell_EquationBased_DetailedRohm.port_a, fixedTemperature.port)
    annotation (Line(points={{264,16},{264,6},{272,6},{272,-10},{266,-10}},
        color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
            -100},{360,100}})),                                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={
            {-240,-100},{360,100}}), graphics={
        Rectangle(extent={{-228,92},{-88,-88}},  lineColor={28,108,200}),
        Text(
          extent={{-148,-66},{-94,-100}},
          textColor={28,108,200},
          textString="Core temp = 80C
"),     Rectangle(extent={{-68,92},{72,-88}},    lineColor={28,108,200}),
        Text(
          extent={{14,-62},{64,-96}},
          textColor={28,108,200},
          textString="Core temp = 60C"),
        Rectangle(extent={{90,92},{230,-88}},    lineColor={28,108,200}),
        Text(
          extent={{170,-62},{220,-96}},
          textColor={28,108,200},
          textString="Cold plate = 40 C")}),
    Documentation(info="<html>
<p>This example model shows a stack with the PEM_detailed membrane model with cooling. Coolant boundary 
conditions are components from the DassaultSystemes Library. 
</p>
<p>
At t=230s the input of the current ramp is quickly ramped up from 100A to  150A, resulting in a power increase 
from 4800 W to 6500W. Use the \"PlotVariables\" command from the Command menu to plot some of the variables. 
</p>
</html>"),
    experiment(StopTime=20, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file="modelica://Hydrogen/Resources/Scripts/PlotVariablesCooledStack.mos" "PlotVariables"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(
        EvaluateAlsoTop=false,
        GenerateVariableDependencies=false,
        OutputModelicaCode=false),
      Evaluate=true,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end FCDynamics;
