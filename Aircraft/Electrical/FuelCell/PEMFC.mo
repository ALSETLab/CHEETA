within CHEETA.Aircraft.Electrical.FuelCell;
model PEMFC
  "Test of PEMFC stack with cooling and support for condensation"
  import FuelCell;


  package Medium_an =
      FuelCell.Media.PreDefined.IdealGases.NASAReformateShort
    "Medium at anode side";
  package Medium_an_condensing =
    FuelCell.Media.PreDefined.CondensingGases.CondensingReformateShort
    "Medium at anode side - condensing media";

  package Medium_cath =
    FuelCell.Media.PreDefined.IdealGases.NASAMoistAir
    "Medium at cathode side";
  package Medium_cath_condensing =
      FuelCell.Media.PreDefined.CondensingGases.CondensingMoistAir
    "Medium at cathode side -condensing media";
  package MediumWater = Modelon.Media.PreDefined.TwoPhase.WaterIF97
    "Water medium";

  parameter Integer[1] cath = Medium_cath.substanceIndexVector({"O2"})
    "index of required substances on the cathode side"                                                                        annotation(Evaluate=true);

  parameter Modelica.SIunits.MassFraction[Medium_an.nS] X_fuel= {0.04,0,0.41,0.17,0.3785,0.0015}
    "Inlet fuel composition";

  parameter Modelica.SIunits.MassFraction[Medium_cath.nS] X_air = {0,0,0.05,0.74,0.21}
    "Inlet air composition";

  parameter Modelica.SIunits.Temperature Tin_fuel = 323.15
    "Inlet fuel temperature";
  parameter Modelica.SIunits.Temperature Tin_air = 323.15
    "Inlet air temperature";
  parameter Modelica.SIunits.Temperature Tin_water = 330.15
    "Inlet water temperature";

  parameter Modelica.SIunits.Pressure pstart_an = 1.3e5
    "Start pressure (anode side)";
  parameter Modelica.SIunits.Pressure pstart_cath = 1.2e5
    "Start pressure (cathode side)";
  parameter Modelica.SIunits.Pressure pin_water = 2.06e5 "Inlet water pressure";

  parameter Modelica.SIunits.MassFlowRate m_fuel= 0.0024 "Inlet fuel flow rate";
  parameter Modelica.SIunits.MassFlowRate m_air = 0.007 "Inlet air flow rate";
  parameter Modelica.SIunits.MassFlowRate m_water = 0.1 "Inlet water flow rate";
  parameter Real cath_stoich = 1.7 "cathode stoichiometry";
  parameter Real anode_stoich = 1.2 "anode stoichiometry";

  parameter Modelica.SIunits.MassFlowRate m_out = m_air*X_air[cath[1]]*(1-1/cath_stoich)
    "calculated mass flow rate of oxygen gas at cathode outlet, may be used for control of the inlet cathode flow rate";
  FuelCell.Sources.GasFlowBoundary flowAnode(
    redeclare package Medium = Medium_an,
    m_flow=m_fuel,
    X=X_fuel,
    T=Tin_fuel,
    use_flow_in=false,
    use_Th_in=false)
                    annotation (Placement(transformation(extent={{-102,20},{-82,
            40}}, rotation=0)));

  FuelCell.Sources.CondensingGasPressureBoundary sinkAnode(
    X=X_fuel,
    redeclare package Medium = Medium_an_condensing,
    p=120000,
    T(displayUnit="K") = 180)
              annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={88,30})));

  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-22,50},{-10,62}},   rotation=
           0)));
  FuelCell.Sources.CondensingGasFlowBoundary flowCathode(
    X=X_air,
    m_flow=m_air,
    T=Tin_air,
    use_flow_in=false,
    use_Th_in=false,
    redeclare package Medium = Medium_cath_condensing) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-90,-20})));
  FuelCell.Sources.CondensingGasPressureBoundary sinkCathode(
    redeclare package Medium = Medium_cath_condensing,
    p=102000,
    T(displayUnit="K") = 180)
              annotation (Placement(transformation(extent={{100,-28},{80,-8}},
          rotation=0)));
  FuelCell.Stacks.PEMFC.CondPEMFCStack coolStack(
    redeclare package Medium_an = Medium_an,
    redeclare package Medium_cath = Medium_cath,
    redeclare package Medium_cooling =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    n_cell=10,
    L_cooling=1,
    pstart_an=pstart_an,
    pstart_cath=pstart_cath,
    redeclare package CondensingMedium_an =
        FuelCell.Media.PreDefined.CondensingGases.CondensingReformateShort,
    redeclare package CondensingMedium_cath =
        FuelCell.Media.PreDefined.CondensingGases.CondensingMoistAir,
    D_cooling=0.05)
    annotation (Placement(transformation(extent={{-20,6},{20,38}})));

  FuelCell.Sources.WaterFlowBoundary sourceW(redeclare package Medium =
        MediumWater, m_flow=m_water,
    T(displayUnit="degC") = 298.15)
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  FuelCell.Sources.WaterPressureBoundary sinkP(redeclare package Medium =
        MediumWater, T(displayUnit="degC") = 303.15)
                     annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={50,-40})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n1 annotation (
      Placement(transformation(extent={{-70,60},{-50,80}}), iconTransformation(
          extent={{-70,60},{-50,80}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p1 annotation (
      Placement(transformation(extent={{50,60},{70,80}}), iconTransformation(
          extent={{50,60},{70,80}})));
equation

  connect(coolStack.pin_n, pin_n1)
    annotation (Line(points={{-8,35.2},{-8,70},{-60,70}}, color={0,0,255}));
  connect(ground.p, pin_n1)
    annotation (Line(points={{-16,62},{-16,70},{-60,70}}, color={0,0,255}));
  connect(coolStack.pin_p, pin_p1)
    annotation (Line(points={{8,35.2},{8,70},{60,70}}, color={0,0,255}));
  connect(pin_p1, pin_p1)
    annotation (Line(points={{60,70},{60,70}}, color={0,0,255}));
  connect(flowCathode.fluidPort, coolStack.feed_cath) annotation (Line(points={
          {-81,-20},{-48,-20},{-48,14},{-18,14}}, color={209,60,0}));
  connect(coolStack.feed_an, flowAnode.fluidPort)
    annotation (Line(points={{-18,30},{-83,30}}, color={255,128,0}));
  connect(coolStack.feed_cooling, sourceW.fluidPort)
    annotation (Line(points={{-8,8.8},{-8,-40},{-41,-40}}, color={0,0,255}));
  connect(coolStack.drain_cooling, sinkP.fluidPort)
    annotation (Line(points={{8,8.8},{8,-40},{41,-40}}, color={0,0,255}));
  connect(coolStack.drain_cath, sinkCathode.fluidPort) annotation (Line(points=
          {{18,14},{50,14},{50,-18},{81,-18}}, color={209,60,0}));
  connect(coolStack.drain_an, sinkAnode.fluidPort)
    annotation (Line(points={{18,30},{79,30}}, color={209,60,0}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -80},{120,80}})),
    experiment(StopTime=1000, Tolerance=1e-004),
    __Dymola_experimentSetupOutput(equdistant=true),
    Documentation(revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2019, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>", info="<html>
<h4>TestCondPEMFCoolStack</h4>
<p>Experiment with proton exchange membrane fuel cell stack with cooling. All boundary conditions are constant, making the model converge to steady state heat production and temperature.</p>
</html>"),
    Icon(coordinateSystem(extent={{-120,-80},{120,80}}), graphics={
        Rectangle(
          extent={{-90,24},{88,-26}},
          lineColor={215,215,215},
          fillColor={175,175,175},
          fillPattern=FillPattern.CrossDiag),
        Rectangle(
          extent={{-100,60},{100,20}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Rectangle(
          extent={{-100,-20},{100,-60}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215})}));
end PEMFC;
