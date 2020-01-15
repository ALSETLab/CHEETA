within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model SOFCSystem
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

  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{12,42},{30,60}},     rotation=
           0)));
  Modelica.Electrical.Analog.Sources.RampCurrent current(
    offset=10,
    duration=3000,
    startTime=1000,
    I=100)
    annotation (Placement(transformation(
        origin={0,60},
        extent={{-8,-8},{8,8}},
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
  Modelon.Visualizers.RealValue display_I(precision=3, number=stack.summary.VCst)
    annotation (Placement(transformation(extent={{-88,-86},{-68,-66}})));
  Modelon.Visualizers.RealValue display_P(number=current.p.i*(current.p.v -
        current.n.v), precision=0)
    annotation (Placement(transformation(extent={{-64,-86},{-44,-66}})));
  Modelon.Visualizers.RealValue display_P1(number=current.p.i, precision=2)
    annotation (Placement(transformation(extent={{-88,-102},{-68,-82}})));
  Modelon.Visualizers.RealValue display_P2(number=stack.utilization*100,
      precision=0)
    annotation (Placement(transformation(extent={{-64,-102},{-44,-82}})));
  Modelon.Visualizers.RealValue display_P3(
      precision=0, number=-stack.heat_port.Q_flow)
    annotation (Placement(transformation(extent={{-32,-102},{-12,-82}})));
  Modelon.Visualizers.RealValue display_P6(precision=1, number=sum(stack.subStack.summary.j_external)
        /stack.nbrSubStacks*1e-4*1e3)
    annotation (Placement(transformation(extent={{-32,-86},{-12,-66}})));
  FuelCell.Sensors.MultiDisplay_phTmdot display_phTmdot3(displayUnits=true,
      redeclare package Medium = Medium_an)
    annotation (Placement(transformation(extent={{-70,18},{-50,38}})));
  FuelCell.Sensors.GasMultiDisplaySensor gasSensor3(redeclare package Medium =
        Medium_an)
    annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
  FuelCell.Sensors.ReformateLongCompositionDisplay display_MoleFractions1(
      redeclare package Medium = Medium_an)
    annotation (Placement(transformation(extent={{-46,30},{-22,50}})));
  FuelCell.Sensors.MultiDisplay_phTmdot display_phTmdot2(displayUnits=true,
      redeclare package Medium = Medium_cath)
    annotation (Placement(transformation(extent={{-70,-22},{-50,-2}})));
  FuelCell.Sensors.AirCompositionDisplay display_MoleFractions3(redeclare
      package Medium = Medium_cath)
    annotation (Placement(transformation(extent={{-46,-16},{-24,2}})));
  FuelCell.Sensors.ReformateLongCompositionDisplay display_MoleFractions4(
      redeclare package Medium = Medium_an)
    annotation (Placement(transformation(extent={{48,30},{72,50}})));
  FuelCell.Sensors.MultiDisplay_phTmdot display_phTmdot6(displayUnits=true,
      redeclare package Medium = Medium_an)
    annotation (Placement(transformation(extent={{24,18},{44,38}})));
  FuelCell.Sensors.AirCompositionDisplay display_MoleFractions2(redeclare
      package Medium = Medium_cath)
    annotation (Placement(transformation(extent={{48,-16},{68,2}})));
  FuelCell.Sensors.MultiDisplay_phTmdot display_phTmdot5(displayUnits=true,
      redeclare package Medium = Medium_cath)
    annotation (Placement(transformation(extent={{24,-22},{44,-2}})));
  FuelCell.Sensors.GasMultiDisplaySensor gasSensor1(redeclare package Medium =
        Medium_cath)
    annotation (Placement(transformation(extent={{24,-30},{44,-10}})));
  FuelCell.Sensors.GasMultiDisplaySensor gasSensor2(redeclare package Medium =
        Medium_an)
    annotation (Placement(transformation(extent={{24,10},{44,30}})));
  FuelCell.Sensors.GasMultiDisplaySensor gasSensor4(redeclare package Medium =
        Medium_cath)
    annotation (Placement(transformation(extent={{-70,-30},{-50,-10}})));
equation
  connect(current.n,ground. p) annotation (Line(
      points={{8,60},{21,60}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(current.p, stack.pin_p) annotation (Line(
      points={{-8,60},{-10,60},{-10,16.8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(current.n, stack.pin_n) annotation (Line(
      points={{8,60},{8,16.8},{10,16.8}},
      color={0,0,255},
      smooth=Smooth.None));

  connect(display_MoleFractions1.data,gasSensor3. u) annotation (Line(points={{-34,
          25.2941},{-34,22},{-59.95,22},{-59.95,20.05}}, color={0,0,0}));
  connect(display_phTmdot3.y,gasSensor3. u) annotation (Line(points={{-60,28},{
          -60,20.05},{-59.95,20.05}},
                                  color={0,0,0}));
  connect(gasSensor4.u,display_MoleFractions3. data) annotation (Line(points={{-59.95,
          -19.95},{-59.95,-17.975},{-35,-17.975},{-35,-16}}, color={0,0,0}));
  connect(gasSensor4.u,display_phTmdot2. y) annotation (Line(points={{-59.95,
          -19.95},{-59.95,-14.975},{-60,-14.975},{-60,-12}},
                                                     color={0,0,0}));
  connect(stack.feed_an, gasSensor3.portB)
    annotation (Line(points={{-20,4},{-20,20},{-54,20}}, color={255,128,0}));
  connect(gasSensor1.u, display_MoleFractions2.data) annotation (Line(points={{
          34.05,-19.95},{34.05,-18},{58,-18},{58,-16}}, color={0,0,0}));
  connect(gasSensor1.u,display_phTmdot5. y) annotation (Line(points={{34.05,
          -19.95},{34.05,-14.975},{34,-14.975},{34,-12}},
                                                  color={0,0,0}));
  connect(sinkCathode.fluidPort, gasSensor1.portB)
    annotation (Line(points={{71,-20},{40,-20}}, color={255,128,0}));
  connect(gasSensor1.portA, stack.drain_cath)
    annotation (Line(points={{28,-20},{20,-20},{20,-8}}, color={255,128,0}));
  connect(sinkAnode.fluidPort, gasSensor2.portB)
    annotation (Line(points={{71,20},{40,20}}, color={255,128,0}));
  connect(gasSensor2.portA, stack.drain_an)
    annotation (Line(points={{28,20},{20,20},{20,4}}, color={255,128,0}));
  connect(gasSensor2.u, display_phTmdot6.y) annotation (Line(points={{34.05,
          20.05},{34.05,24.025},{34,24.025},{34,28}}, color={0,0,0}));
  connect(display_MoleFractions4.data, gasSensor2.u) annotation (Line(points={{60,
          25.2941},{60,22},{34,22},{34,20.05},{34.05,20.05}},    color={0,0,0}));
  connect(gasSensor4.portA, flowCathode.fluidPort)
    annotation (Line(points={{-66,-20},{-71,-20}}, color={255,128,0}));
  connect(gasSensor4.portB, stack.feed_cath) annotation (Line(points={{-54,-20},
          {-20,-20},{-20,-8}}, color={255,128,0}));
  connect(gasSensor3.portA, flowAnode.fluidPort)
    annotation (Line(points={{-66,20},{-71,20}}, color={255,128,0}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),
                      graphics={
        Rectangle(
          extent={{-94,-60},{2,-98}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          radius=2),
        Text(
          extent={{-86,-66},{-68,-72}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Voltage [V]"),
        Text(
          extent={{-64,-66},{-46,-72}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Power [W]"),
        Line(points={{-94,-66},{2,-66}},  color={0,0,0}),
        Text(
          extent={{-92,-66},{-66,-60}},
          lineColor={0,0,0},
          textStyle={TextStyle.Bold},
          textString="Stack"),
        Text(
          extent={{-86,-82},{-68,-88}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Current [A]"),
        Text(
          extent={{-66,-82},{-38,-88}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Fuel Utilization %%"),
        Text(
          extent={{-34,-82},{-8,-88}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Heat loss [W]"),
        Text(
          extent={{-40,-64},{0,-74}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Current density [mA/cm^2]")}),
    experiment(StopTime=6000,Tolerance=1e-6),
    __Dymola_experimentSetupOutput(equdistant=false),
    Documentation(revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2019, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>", info="<html>
<h4>TestSOFCFullStack</h4>
<p>Experiment with solid oxide fuel cell stack. The stack has three substacks with 10, 30 and 10 cells respectively and uses external manifolds to split the flow between the substacks.
The current is ramped from 10 to 110 ampere in 3000 seconds, starting after 1000 s. This results in an increased heat production from the stack.</p>
</html>"));
end SOFCSystem;
