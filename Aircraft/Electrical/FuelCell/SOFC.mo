within CHEETA.Aircraft.Electrical.FuelCell;
model SOFC
  "SOFC stack test model with array of substacks, manifolds and endplates"
  import FuelCell;

  FuelCell.Sources.GasFlowBoundary flowCathode(
    redeclare package Medium = Medium_cath,
    m_flow=4.711e-2,
    T=1073.15) annotation (Placement(transformation(extent={{-90,-30},{-70,-10}},
          rotation=0)));
  replaceable package Medium_an =
      FuelCell.Media.PreDefined.IdealGases.NASAReformateLong;
  replaceable package Medium_cath =
      FuelCell.Media.PreDefined.IdealGases.NASAMoistAir;
  FuelCell.Sources.GasFlowBoundary flowAnode(
    redeclare package Medium = Medium_an,
    m_flow=1.035e-3,
    X=Medium_an.moleToMassFractions({0.2766,0.1696,0.018,0.0833,0.2453,0.2055,0.0017},
        Medium_an.MMX),
    T=1073.15) annotation (Placement(transformation(extent={{-90,10},{-70,30}},
          rotation=0)));

  FuelCell.Sources.GasPressureBoundary sinkCathode(
    redeclare package Medium = Medium_cath,
    p=101300,
    T=1073.15) annotation (Placement(transformation(extent={{90,-30},{70,-10}},
          rotation=0)));
  FuelCell.Sources.GasPressureBoundary sinkAnode(
    redeclare package Medium = Medium_an,
    p=101300,
    T=1173.15) annotation (Placement(transformation(extent={{90,10},{70,30}},
          rotation=0)));

  SOFCStack                      stack(
    m_flow_nom_an=1e-3,
    m_flow_nom_cath=4.7e-2,
    redeclare package Medium_an = Medium_an,
    redeclare package Medium_cath = Medium_cath,
    N=4,
    nbrSubStacks=3,
    n_cell={10,30,10},
    A_cell=361e-4,
    Cp_cell=500,
    kc_cell=250,
    p_start_in_anode=130000,
    T_start_in_anode=1073.15,
    T_start_out_anode=1073.15,
    p_start_in_cathode=130000,
    T_start_in_cathode=1073.15,
    T_start_out_cathode=1073.15,
    subStack(anode_channel(channel(reaction(each Zx_nominal=1e-4)))))
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  FuelCell.Internal.Conservation.CheckMassBalanceStack checkMassBalanceStack(
    nS_an=Medium_an.nS,
    MMX_an=Medium_an.MMX,
    nH_an=Medium_an.H_atoms,
    nC_an=Medium_an.C_atoms,
    nO_an=Medium_an.O_atoms,
    m_flow_in_an=flowAnode.m_flow,
    X_in_an=Medium_an.moleToMassFractions({0.2766,0.1696,0.018,0.0833,0.2453,0.2055,
        0.0017}, Medium_an.MMX),
    m_flow_out_an=stack.drain_an.m_flow,
    X_out_an=actualStream(stack.drain_an.X_outflow),
    nS_cath=Medium_cath.nS,
    MMX_cath=Medium_cath.MMX,
    nH_cath=Medium_cath.H_atoms,
    nC_cath=Medium_cath.C_atoms,
    nO_cath=Medium_cath.O_atoms,
    m_flow_in_cath=flowCathode.m_flow,
    m_flow_out_cath=stack.drain_cath.m_flow,
    X_in_cath=flowCathode.X,
    X_out_cath=actualStream(stack.drain_cath.X_outflow))
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p1 annotation (
      Placement(transformation(extent={{-50,70},{-30,90}}), iconTransformation(
          extent={{-50,70},{-30,90}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n1 annotation (
      Placement(transformation(extent={{28,70},{48,90}}), iconTransformation(
          extent={{28,70},{48,90}})));
equation

  connect(flowAnode.fluidPort, stack.feed_an) annotation (Line(points={{-71,20},
          {-40,20},{-40,4},{-20,4}}, color={255,128,0}));
  connect(stack.drain_an, sinkAnode.fluidPort) annotation (Line(points={{20,4},
          {40,4},{40,20},{71,20}}, color={255,128,0}));
  connect(stack.feed_cath, flowCathode.fluidPort) annotation (Line(points={{-20,
          -8},{-40,-8},{-40,-20},{-71,-20}}, color={255,128,0}));
  connect(stack.drain_cath, sinkCathode.fluidPort) annotation (Line(points={{20,
          -8},{40,-8},{40,-20},{71,-20}}, color={255,128,0}));
  connect(stack.pin_p, pin_p1)
    annotation (Line(points={{-10,16.8},{-10,80},{-40,80}}, color={0,0,255}));
  connect(stack.pin_n, pin_n1)
    annotation (Line(points={{10,16.8},{10,80},{38,80}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=6000,Tolerance=1e-6),
    __Dymola_experimentSetupOutput(equdistant=false),
    Documentation(revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2019, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>", info="<html>
<h4>TestSOFCFullStack</h4>
<p>Experiment with solid oxide fuel cell stack. The stack has three substacks with 10, 30 and 10 cells respectively and uses external manifolds to split the flow between the substacks.
The current is ramped from 10 to 110 ampere in 3000 seconds, starting after 1000 s. This results in an increased heat production from the stack.</p>
</html>"),
    Icon(graphics={
        Polygon(
          points={{-88,58},{-86,62},{-78,68},{-60,76},{-26,82},{26,82},{66,74},
              {86,62},{88,58},{88,26},{92,22},{98,22},{98,16},{92,16},{88,12},{
              88,-32},{92,-36},{98,-36},{98,-42},{94,-42},{88,-48},{88,-80},{
              -88,-80},{-88,-46},{-92,-42},{-98,-42},{-98,-36},{-92,-36},{-88,
              -32},{-88,12},{-92,16},{-98,16},{-98,22},{-92,22},{-88,26},{-88,
              58}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillPattern=FillPattern.VerticalCylinder,
          fillColor={0,0,161}),
        Rectangle(
          extent={{-76,52},{76,48}},
          lineColor={0,0,0},
          fillColor={222,169,9},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-68},{76,-72}},
          lineColor={0,0,0},
          fillColor={222,169,9},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,48},{76,44}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-64},{76,-68}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,44},{76,36}},
          lineColor={94,55,27},
          fillColor={94,55,27},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,36},{76,32}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,28},{76,24}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,32},{76,28}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,16},{76,12}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,8},{76,4}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,12},{76,8}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,-4},{76,-8}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-12},{76,-16}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-8},{76,-12}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,-24},{76,-28}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-32},{76,-36}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-28},{76,-32}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,-44},{76,-48}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-52},{76,-56}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-76,-48},{76,-52}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,24},{76,16}},
          lineColor={94,55,27},
          fillColor={94,55,27},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,4},{76,-4}},
          lineColor={94,55,27},
          fillColor={94,55,27},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,-16},{76,-24}},
          lineColor={94,55,27},
          fillColor={94,55,27},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,-36},{76,-44}},
          lineColor={94,55,27},
          fillColor={94,55,27},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-76,-56},{76,-64}},
          lineColor={94,55,27},
          fillColor={94,55,27},
          fillPattern=FillPattern.Solid),
                              Text(
          extent={{-100,-82},{100,-94}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="%name")}));
end SOFC;
