within CHEETA.Aircraft.Electrical.FuelCell.Templates;
model FullStack
  "Stack template with a number of substacks that runs in parallel, manifolds and endplates"
  import FuelCell;
  extends FuelCell.Icons.Stack;
  import Modelica.Constants.eps;

  replaceable package Medium_an =
      FuelCell.Media.PreDefined.IdealGases.NASAReformateLong                             constrainedby
    FuelCell.Media.Templates.ReactionGas "Medium at anode side" annotation(choicesAllMatching,Dialog(group="General"));
  replaceable package Medium_cath =
      FuelCell.Media.PreDefined.IdealGases.NASAMoistAir                               constrainedby
    FuelCell.Media.Templates.ReactionGas "Medium at cathode side" annotation(choicesAllMatching,Dialog(group="General"));

 /* Substack parameters */
  replaceable model SubStack = FuelCell.Stacks.Templates.SubStack
                                                               constrainedby
    FuelCell.Stacks.Templates.SubStack "Substack model to use" annotation(choicesAllMatching,Dialog(tab="Stack", group="Configuration"));

  parameter Integer N(min = 1) = 1
    "Number of discretization nodes in the flow direction"                                annotation(Dialog(tab="Stack", group="Configuration"));
  parameter Integer nbrSubStacks(min = 2) = 2 "number of substacks in parallel" annotation(Dialog(tab="Stack", group="Configuration"));
  parameter Integer[nbrSubStacks] n_cell = fill(1,nbrSubStacks)
    "number of cells in substacks (vector)"                                          annotation(Dialog(tab="Stack", group="Configuration"));

  parameter Modelica.Units.SI.Area A_cell "active cell area"
    annotation (Dialog(tab="Stack", group="Cell data"));
  parameter Modelica.Units.SI.SpecificHeatCapacity Cp_cell
    "specific heat capacity of cell material"
    annotation (Dialog(tab="Stack", group="Cell data"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer kc_cell
    "cell internal heat transfer coefficient"
    annotation (Dialog(tab="Stack", group="Cell data"));

  parameter Modelica.Units.SI.Mass M_stack=1 "total mass of stack"
    annotation (Dialog(group="General"));

  /* Heat transfer */
  extends FuelCell.Stacks.Interfaces.HeatTransferParameters;

  /* Initialization */
  extends FuelCell.Stacks.Interfaces.DistributedAnodeInitialization(
     final n=N, X_start_anode=Medium_an.reference_X);
  extends FuelCell.Stacks.Interfaces.DistributedCathodeInitialization(
     final n=N, X_start_cathode=Medium_cath.reference_X, p_start_cathode = if not counterFlow then linspace(p_start_in_cathode, p_start_out_cathode, n+1) else linspace(p_start_out_cathode,  p_start_in_cathode, n+1));

  /* Advanced parameters */
  extends FuelCell.Stacks.Interfaces.AdvancedParameters(final
      useHeatTransfer_anode=false, final useHeatTransfer_cathode=false);

  parameter Modelica.Units.SI.Pressure dp_start_anode=5000
    "Initial pressure drop in anode manifolds due to friction/Pipe resistance"
    annotation (Dialog(tab="Initialization", group="Manifold"));

  parameter Modelica.Units.SI.Pressure dp_start_cathode=5000
    "Initial pressure drop in cathode manifolds due to friction/Pipe resistance"
    annotation (Dialog(tab="Initialization", group="Manifold"));

  parameter Real[nbrSubStacks] splitRatio_an = n_cell/sum(n_cell)
    "splitratio anode side"                                                              annotation(Dialog(group="Manifolds"));
  parameter Real[nbrSubStacks] splitRatio_cath = n_cell/sum(n_cell)
    "splitratio cathode side"                                                                annotation(Dialog(group="Manifolds"));

  parameter Modelica.Units.SI.Volume V_manifold_an=0.001 "Manifold_an volume"
    annotation (Dialog(group="Manifolds"));
  parameter Modelica.Units.SI.Volume V_manifold_cath=0.001
    "Manifold_cath volume" annotation (Dialog(group="Manifolds"));

  parameter Modelica.Units.SI.Height height=sum(n_cell)*thickness_cell
    "Stack height" annotation (Dialog(group="Stack parameter"));

  parameter Modelica.Units.SI.Length thickness_cell=3e-2 "Cell thickness"
    annotation (Dialog(group="Cell parameters"));

  parameter Modelica.Units.SI.Length wallthickness=0.01
    "Average metal thickness of wall" annotation (Dialog(group="End plates"));

  parameter Modelica.Units.SI.Area A_cell_tot(min=A_cell) = 400e-4
    "Total cell area" annotation (Dialog(group="Cell parameters"));

  parameter Modelica.Units.SI.MassFlowRate m_flow_nom_an=1e-3
    "Nominal mass flow rate anode side"
    annotation (Dialog(group="Pressure loss"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_nom_cath=1e-3
    "Nominal mass flow rate cathode side"
    annotation (Dialog(group="Pressure loss"));

  FuelCell.HeatTransfer.BasicHeatTransfer[nbrSubStacks-1] heat(
    each n=N,
    each Aheatloss=A_cell,
    each kc=kc_cell) "heat transfer between subStacks";

SubStack subStack[nbrSubStacks](
    each N=N,
    n_cell=n_cell[1:nbrSubStacks],
    redeclare each package Medium_an = Medium_an,
    redeclare each package Medium_cath = Medium_cath,
    each A_cell=A_cell,
    M_cell=(M_stack/sum(n_cell))*n_cell,
    each Cp_cell=Cp_cell,
    each kc=kc_cell,
    each initOpt_anode=initOpt_anode,
    each p_start_anode=p_start_anode,
    each initFromEnthalpy_anode=initFromEnthalpy_anode,
    each h_start_in_anode=h_start_in_anode,
    each h_start_out_anode=h_start_out_anode,
    each h_start_anode=h_start_anode,
    each T_start_in_anode=T_start_in_anode,
    each T_start_out_anode=T_start_out_anode,
    each T_start_anode=T_start_anode,
    each X_start_anode=X_start_anode,
    each m_flow_start_anode=m_flow_start_anode,
    each initOpt_cathode=initOpt_cathode,
    each p_start_cathode=p_start_cathode,
    each initFromEnthalpy_cathode=initFromEnthalpy_cathode,
    each h_start_in_cathode=h_start_in_cathode,
    each h_start_out_cathode=h_start_out_cathode,
    each h_start_cathode=h_start_cathode,
    each T_start_in_cathode=T_start_in_cathode,
    each T_start_out_cathode=T_start_out_cathode,
    each T_start_cathode=T_start_cathode,
    each X_start_cathode=X_start_cathode,
    each m_flow_start_cathode=m_flow_start_cathode,
    each dp_smooth=dp_smooth,
    each mflow_smooth=mflow_smooth,
    each from_dp_anode=from_dp_anode,
    each positiveFlow_anode=positiveFlow_anode,
    each generateEventForReversal_anode=generateEventForReversal_anode,
    each from_dp_cathode=from_dp_cathode,
    each positiveFlow_cathode=positiveFlow_cathode,
    each generateEventForReversal_cathode=generateEventForReversal_cathode,
    each p_start_in_anode=p_start_in_anode - dp_start_anode,
    each p_start_out_anode=p_start_out_anode + dp_start_anode,
    each p_start_in_cathode=if not counterFlow then p_start_in_cathode - dp_start_cathode else p_start_out_cathode + dp_start_cathode,
    each p_start_out_cathode=if not counterFlow then p_start_out_cathode + dp_start_cathode else p_start_in_cathode - dp_start_cathode)
                         annotation (Placement(transformation(extent={{-16,-16},{16,16}})));

  output FuelCell.SubComponents.ComponentSummaries.StackSummary summary(
    N=nbrSubStacks,
    tAirStkIn=feed_Cathode.volume[1].summary.T,
    tFuelStkIn=feed_Anode.volume[1].summary.T,
    tFuelStkOut={subStack[i].anode_channel.channel.gas[end].T_degC for i in 1:
        nbrSubStacks},
    tAirStkOut={subStack[i].cathode_channel.channel.gas[end].T_degC for i in 1:
        nbrSubStacks},
    tStkOut=Modelica.Units.Conversions.to_degC({subStack[i].T_stack[end] for i in
            1:nbrSubStacks}),
    tStkTopWall=sum(topWall.T_degC)/N,
    tStkBottomWall=sum(bottomWall.T_degC)/N,
    PStkElec=(pin_p.v - pin_n.v)*(-pin_p.i),
    QStkHeat=sum({subStack[i].cell.Q_total for i in 1:nbrSubStacks}),
    dmAirStkIn=feed_cath.m_flow,
    dmFuelStkIn=feed_an.m_flow,
    VCstZone={subStack[i].V_stack for i in 1:nbrSubStacks},
    PStk=pin_p.v*pin_n.i,
    facFuelStkUtil={subStack[i].utilization for i in 1:nbrSubStacks},
    facFuelStkOutComp={subStack[i].cell.y_an[end, 1:7] for i in 1:nbrSubStacks})
    "Summary record" annotation (Placement(transformation(extent={{-80,60},{-60,
            80}}, rotation=0)));

  FuelCell.HeatTransfer.BasicHeatTransfer            topHeatTransfer(
    n=N,
    Aheatloss=A_cell,
    kc=kc_cell)                                                             annotation (
      Placement(transformation(
        extent={{-7,-6},{7,6}},
        rotation=0,
        origin={8,26})));
  FuelCell.HeatTransfer.BasicHeatTransfer            bottomHeatTransfer(
    n=N,
    Aheatloss=A_cell,
    kc=kc_cell)
    annotation (Placement(transformation(extent={{1,-32},{15,-20}})));
  FuelCell.Walls.DynamicWall            bottomWall(
    n=N,
    A_wall=fill(A_cell/N, N),
    T0=T_start_cathode[2:N+1],
    dp=wallthickness)
    annotation (Placement(transformation(extent={{-20,-52},{20,-32}})));
  FuelCell.Walls.DynamicWall            topWall(
    n=N,
    A_wall=fill(A_cell/N, N),
    T0=T_start_anode[2:N+1],
    dp=wallthickness)
    annotation (Placement(transformation(extent={{-20,32},{20,52}})));

    replaceable model Feed_Flow_an =
      FuelCell.Manifolds.Templates.GenericFeedManifold
    constrainedby FuelCell.Manifolds.Templates.GenericFeedManifold
    "Feeding manifold configuration, anode side" annotation (
    editButton=true,
    choicesAllMatching,
    Dialog(tab="Flow configuration", group=
          "Manifolds configuration (Parameters -> 'Edit button')"));

   replaceable model Feed_Flow_cath =
      FuelCell.Manifolds.Templates.GenericFeedManifold
    constrainedby FuelCell.Manifolds.Templates.GenericFeedManifold
    "Feeding manifold configuration, cathode side"
    annotation(editButton= true, choicesAllMatching,Dialog(tab="Flow configuration",group="Manifolds configuration (Parameters -> 'Edit button')"));

   replaceable model Drain_Flow_an =
      FuelCell.Manifolds.Templates.GenericDrainManifold
    constrainedby FuelCell.Manifolds.Templates.GenericDrainManifold
    "Draining manifold configuration, anode side"
    annotation(editButton= true, choicesAllMatching,Dialog(tab="Flow configuration",group="Manifolds configuration (Parameters -> 'Edit button')"));

   replaceable model Drain_Flow_cath =
      FuelCell.Manifolds.Templates.GenericDrainManifold
    constrainedby FuelCell.Manifolds.Templates.GenericDrainManifold
    "Draining manifold configuration, cathode side"
    annotation(editButton= true, choicesAllMatching,Dialog(tab="Flow configuration",group="Manifolds configuration (Parameters -> 'Edit button')"));

   parameter Boolean counterFlow = false
    "Invert flow direction at the cathode to obtain counter flow configuration"
    annotation (Dialog(tab="Flow configuration", group="Counter flow at cathode side"));

   replaceable model Friction =
    Modelon.ThermoFluid.FlowResistances.FrictionModels.LinearOperatingPointLoss
                                                                                  constrainedby
    Modelon.ThermoFluid.FlowResistances.FrictionModels.PartialPressureDrop
    "friction model used for counterFlow case"  annotation(Dialog(tab="Flow configuration", group="Counter flow at cathode side"),choicesAllMatching);

    Feed_Flow_an feed_Anode(
    redeclare package Medium = Medium_an,
    N_drain=nbrSubStacks,
    T_start=T_start_in_anode,
    dp_start=dp_start_anode,
    pstart_in=p_start_in_anode,
    m_flow_start=m_flow_start_anode,
    positiveFlow= true,
    height=height)
    annotation (Placement(transformation(extent={{-78,2},{-60,38}})));

    Feed_Flow_cath feed_Cathode(
    redeclare package Medium = Medium_cath,
    N_drain=nbrSubStacks,
    T_start=T_start_in_cathode,
    dp_start=dp_start_cathode,
    pstart_in= if counterFlow then p_start_out_cathode else p_start_in_cathode,
    m_flow_start= m_flow_start_cathode,
    positiveFlow= not counterFlow,
    height=height)
    annotation (Placement(transformation(extent={{-78,-58},{-60,-22}})));

    Drain_Flow_an drain_Anode(
    redeclare package Medium = Medium_an,
    N_feed=nbrSubStacks,
    T_start=T_start_out_anode,
    dp_start=dp_start_anode,
    pstart_out=p_start_out_anode,
    m_flow_start=m_flow_start_anode,
    positiveFlow= true,
    height=height)
    annotation (Placement(transformation(extent={{60,2},{78,38}})));

    Drain_Flow_cath drain_Cathode(
    redeclare package Medium = Medium_cath,
    N_feed=nbrSubStacks,
    T_start=T_start_out_cathode,
    dp_start=dp_start_cathode,
    pstart_out= if counterFlow then p_start_in_cathode else p_start_out_cathode,
    m_flow_start= m_flow_start_cathode,
    positiveFlow= not counterFlow,
    height=height)
    annotation (Placement(transformation(extent={{60,-58},{78,-22}})));

  FuelCell.FlowResistances.GasResistance ifCounterFlow(
    redeclare package Medium = Medium_cath,
    T_start=T_start_out_cathode,
    mflow_start=m_flow_start_cathode,
    redeclare model Friction=Friction) if counterFlow
    annotation (Placement(transformation(extent={{-10,-88},{10,-72}})));

  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
    annotation (Placement(transformation(extent={{-60,80},{-40,100}}),
        iconTransformation(extent={{-60,74},{-40,94}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
    annotation (Placement(transformation(extent={{40,80},{60,100}}),
        iconTransformation(extent={{40,74},{60,94}})));
  FuelCell.Interfaces.GasVolumePort
                               feed_cath(redeclare package Medium =
        Medium_cath)
    annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
  FuelCell.Interfaces.GasVolumePort
                               feed_an(redeclare package Medium = Medium_an)
    annotation (Placement(transformation(extent={{-110,10},{-90,30}}),
        iconTransformation(extent={{-110,10},{-90,30}})));
  FuelCell.Interfaces.GasFlowPort drain_cath(
                                            redeclare package Medium =
        Medium_cath)
    annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
  FuelCell.Interfaces.GasFlowPort drain_an(
                                          redeclare package Medium = Medium_an)
    annotation (Placement(transformation(extent={{90,10},{110,30}}),
        iconTransformation(extent={{90,10},{110,30}})));

  replaceable model StackHeatLoss =
      FuelCell.HeatTransfer.Interfaces.StackHeatLosses "Stack heat loss model"                         annotation(Dialog(tab="Heat transfer", group="Stack heat loss"),choicesAllMatching);

  StackHeatLoss stackHeatLosses(N=N, nbr=nbrSubStacks,
    A_topWall=A_top,
    A_bottomWall=A_bottom,
    A_sideWall=A_side,
    h_conv_top=h_conv_top,
    h_conv_side=h_conv_side,
    e_top=e_top,
    e_side=e_side,
    k_insul=k_insul,
    dl_topInsul=dl_topInsul,
    dl_bottomInsul=dl_bottomInsul,
    dl_sideInsul=dl_sideInsul)
    annotation (Placement(transformation(extent={{-15,62},{15,86}})));
  Modelon.ThermoFluid.Interfaces.FlowHeatPort heat_port annotation (Placement(transformation(extent={{-7,96},{7,108}}),
        iconTransformation(extent={{-6,84},{6,96}})));

  Real fuel_in "Fuel flowing into the system";
  Real fuel_used "Fuel consumed in the fuel cell";
  Real utilization "Fuel utilization";

protected
  parameter Integer[2] ref = Medium_an.substanceIndexVector({"H2","CH4"})
    "index for H2 in the anode medium" annotation(Evaluate=true);

equation
  fuel_in = feed_an.m_flow*actualStream(feed_an.X_outflow[ref[1]])/Medium_an.MMX[ref[1]] +
           feed_an.m_flow*actualStream(feed_an.X_outflow[ref[2]])/Medium_an.MMX[ref[2]];
  fuel_used = fuel_in +
           drain_an.m_flow*drain_an.X_outflow[ref[1]]/Medium_an.MMX[ref[1]] +
           drain_an.m_flow*drain_an.X_outflow[ref[2]]/Medium_an.MMX[ref[2]];
  utilization = max(0, fuel_used) / max(fuel_in, eps);

  // Connections between subStacks
  for i in 2:nbrSubStacks loop
    connect(subStack[i-1].pin_n,subStack[i].pin_p);
    connect(subStack[i-1].wall, heat[i-1].qa);
    connect(heat[i-1].qb, subStack[i].wall);
  end for;

    if counterFlow then
    connect(drain_Cathode.drain, feed_cath) annotation (Line(
      points={{78,-40},{78,-46},{86,-46},{86,-74},{-96,-74},{-96,-40},{-100,-40}},
      color={255,128,0},
      visible=counterFlow,
      pattern=LinePattern.Dash,
      smooth=Smooth.None,
        arrow={Arrow.Filled,Arrow.None}));
    connect(feed_Cathode.feed, ifCounterFlow.portA)     annotation (Line(
      points={{-78,-40},{-78,-46},{-86,-46},{-86,-80},{-9,-80}},
      color={255,128,0},
      visible=counterFlow,
      pattern=LinePattern.Dash,
      smooth=Smooth.None,
        arrow={Arrow.None,Arrow.Filled}));
    connect(ifCounterFlow.portB, drain_cath)     annotation (Line(
      points={{9,-80},{96,-80},{96,-40},{100,-40}},
      color={255,128,0},
      visible=counterFlow,
      pattern=LinePattern.Dash,
      smooth=Smooth.None,
        arrow={Arrow.None,Arrow.Filled}));
  else
    connect(feed_Cathode.feed, feed_cath) annotation (Line(
      points={{-78,-40},{-100,-40}},
      color={255,128,0},
      visible= not counterFlow,
      smooth=Smooth.None));
    connect(drain_Cathode.drain, drain_cath) annotation (Line(
      points={{78,-40},{100,-40}},
      color={255,128,0},
      visible= not counterFlow,
      smooth=Smooth.None));

  end if;

  connect(pin_p, subStack[1].pin_p) annotation (Line(
      points={{-50,90},{-50,90},{-50,84},{-50,84},{-50,10.56},{0,10.56}},
      color={0,0,255},
      smooth=Smooth.None));

  connect(subStack[nbrSubStacks].pin_n, pin_n) annotation (Line(
      points={{0,-10.56},{50,-10.56},{50,-8},{50,-8},{50,90},{50,90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(topHeatTransfer.qb, subStack[1].wall) annotation (Line(
      points={{8,20},{8,10},{8,0},{6.4,0}},
      color={191,0,0},
      thickness=0.5,
      smooth=Smooth.None));
  connect(bottomHeatTransfer.qa, subStack[nbrSubStacks].wall) annotation (Line(
      points={{8,-20},{8,-10},{8,0},{6.4,0}},
      color={191,0,0},
      thickness=0.5,
      smooth=Smooth.None));
  connect(bottomHeatTransfer.qb, bottomWall.qa) annotation (Line(
      points={{8,-32},{4,-32},{4,-36},{0,-36}},
      color={191,0,0},
      thickness=0.5,
      smooth=Smooth.None));
  connect(topHeatTransfer.qa, topWall.qb) annotation (Line(
      points={{8,32},{4,32},{4,36},{0,36}},
      color={191,0,0},
      thickness=0.5,
      smooth=Smooth.None));
  connect(feed_an, feed_an) annotation (Line(
      points={{-100,20},{-100,20}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(feed_an, feed_Anode.feed) annotation (Line(
      points={{-100,20},{-78,20}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(drain_an, drain_Anode.drain) annotation (Line(
      points={{100,20},{78,20}},
      color={255,128,0},
      smooth=Smooth.None));

  connect(drain_Anode.feed, subStack.drain_anode) annotation (Line(
      points={{60,20},{32,20},{32,6.4},{14.4,6.4}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(drain_Cathode.feed, subStack.drain_cathode) annotation (Line(
      points={{60,-40},{40,-40},{40,-6.4},{14.4,-6.4}},
      color={255,128,0},
      smooth=Smooth.None));
  connect(feed_Anode.drain, subStack.feed_anode) annotation (Line(
      points={{-60,20},{-38,20},{-38,6.4},{-14.4,6.4}},
      color={255,128,0},
      smooth=Smooth.None));

  connect(feed_Cathode.drain, subStack.feed_cathode) annotation (Line(
      points={{-60,-40},{-38,-40},{-38,-6.4},{-14.4,-6.4}},
      color={255,128,0},
      smooth=Smooth.None));

  connect(stackHeatLosses.topWall, topWall.qa) annotation (Line(
      points={{0,82},{-32,82},{-32,48},{0,48}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(stackHeatLosses.bottomWall, bottomWall.qb) annotation (Line(
      points={{0,62},{34,62},{34,-62},{0,-62},{0,-48}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(feed_Anode.heat_port, subStack.wall[1]) annotation (Line(
      points={{-69,38},{-69,48},{-31,48},{-31,0},{6.4,0}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(feed_Cathode.heat_port, subStack.wall[1]) annotation (Line(
      points={{-69,-22},{-69,-16},{-31,-16},{-31,0},{6.4,0}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(stackHeatLosses.leftWall, subStack.wall[1]) annotation (Line(
      points={{-15,72},{-26,72},{-26,0},{6.4,0}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(stackHeatLosses.rightWall, subStack.wall[N]) annotation (Line(
      points={{15,72},{28,72},{28,0},{6.4,0}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(drain_Anode.heat_port, subStack.wall[N]) annotation (Line(
      points={{69,38},{69,46},{43,46},{43,0},{6.4,0}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(drain_Cathode.heat_port, subStack.wall[N]) annotation (Line(
      points={{69,-22},{69,-12},{43,-12},{43,0},{6.4,0}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(heat_port, stackHeatLosses.environment) annotation (Line(
      points={{8.88178e-016,102},{8.88178e-016,94},{0,94},{0,86.8}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
            -100},{100,100}})),     Icon(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}}), graphics),
    Documentation(revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2019, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>", info="<html>
<h4>FullStack</h4>
<p>This template is suitable for easy setup of complete high and mid-temperature fuel cell stacks. The model contains the following components:</p>
<p><ul>
<li>End plates</li>
<li>Feed and drain connectors of reaction gas type (ideal gas) for both anode and cathode</li>
<li>Electrical pins (positive and negative)</li>
<li>Replaceable substack model (used as placeholder) - needs to be replaced before simulation</li>
<li>Replaceable heat loss model (with or without insultation)</li>
<li>Mixing volumes and linear flow resistance before the fluid flow outlets</li>
</ul></p>

<p>Customized flow structures for both the anode and cathode sides (u-flow, z-flow, mid-flow, external manifolds, etc.) can be set up using manifolds at the inlet and outlet on the anode and cathode side. The model supports both co-flow and counter-flow configurations, this is determined by the <code>counterFlow</code> parameter.</p>

<h4>Parametrization</h4>
The discretization of the stack is defined using the following parameters:
<p><ul>
<li>The parameter <code>nbrSubStacks</code> describes the number of substacks in parallel</li>
<li>The parameter (array) <code>n_cell</code> describes the number of cells in each substack</li>
<li>The parameter <code>N</code> describes the number of discretizations in the flow direction (membrane and flow channels in each substack divided in <code>N</code> segments) </li>
</ul></p>
<h4>Substack model</h4>
<p>The Stack template uses a replaceable model (<a href=\"modelica://FuelCell.Stacks.Templates.SubStack\">FuelCell.Stacks.Templates.SubStack</a>) that needs to be replaced with a finalized and parameterized substack model before use. Any preparameterized substack model based on the SubStack template may be used, see for instance the predefined <a href=\"modelica://FuelCell.Stacks.SOFC\">SOFC</a> substacks. </p>
<p>The parameter <code>nbrSubStacks</code> describes the number of parallel substacks, connected electrically and thermally in series. The number of cells modelled by each substack is described by the array parameter <code>n_cell</code> and the number discrete segments in the fluid flow direction of each substack is described by the parameter <code>N</code>.</p>

<h4>Manifold configurations</h4>
<p>The manifolds are used to split the flow between the substacks. Different flow configurations are obtained dependning on the type of manifolds that are selected at inlet and outlet on the anode and cathode sides. A large number of configurations can be obtained by using the predefined manifolds.</p>
<p>Four main manifold configurations can be observed (presented below in co-flow configuration).Counter flow configuration is obtained by changing the parameter <code>counterFlow</code> to true (affects the flow direction at the cathode side).</p>
<h5>U configuration</h5>
<p>Obtained by using the BottomInlet and BottomOutlet manifolds. Flow maldistribution can be observed as the majority of the flow takes the shortest way at the bottom of the stack.</p>
<p><img src=\"modelica://FuelCell/Resources/images/stacks/Uflow.png\"/></p>
<h5>M configuration</h5>
<p>Obtained by using the MiddleInlet and MiddleOutlet manifolds. Flow maldistribution can be observed as the majority of the flow takes the shortest way at the middle of the stack.</p>
<p><br><br><img src=\"modelica://FuelCell/Resources/images/stacks/Mflow.png\"/></p>
<h5>Z configuration</h5>
<p>Obtained by using the BottomInlet and TopOutlet manifolds. Relative even distribution of the flow with this configuration.</p>
<p><br><img src=\"modelica://FuelCell/Resources/images/stacks/Zflow.png\"/></p>
<h5>External configuration</h5>
<p>Obtained by using External manifolds at both inlet and outlet.</p>
<p><img src=\"modelica://FuelCell/Resources/images/stacks/External.png\"/></p>
<h4>Heat losses</h4>
<p>Two heat loss models are available:</p>
<h5>SHLWithoutInsulation</h5>
<p>The stack is considered to operate in a furnace, represented by the &QUOT;environment&QUOT; heat port on the top of the component. The heat transfer is then exchanged by <i>convection</i> and <i>radiation</i> only. The bottom of the stack (connected to &QUOT;bottomWall&QUOT; port) is thermally connected with a <i>conduction</i> model, as the stack lays on the floor of the furnace. </p>
<p><br><img src=\"modelica://FuelCell/Resources/images/stacks/withouthinsulation.png\"/></p>
<h5>SHLWithInsulation</h5>
<p>The stack is considered to operate in ambiant air medium, but insulation is set around the stack. This model contains the <i>same heat transfer properties than SHLWithoutInsulation</i>, plus a <i>conductive heat transfer model</i> all around the stack, bottom included. The thermal conductivity of the insulation has to be set by the user, as well as the thicknesses of each of the walls.</p>
<p><br><br><br><br><img src=\"modelica://FuelCell/Resources/images/stacks/withinsulation.png\"/></p>
<p></p>
</html>"));
end FullStack;
