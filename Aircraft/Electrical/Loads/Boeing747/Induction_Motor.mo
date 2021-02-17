within CHEETA.Aircraft.Electrical.Loads.Boeing747;
model Induction_Motor
  extends CHEETA.Aircraft.Electrical.Interfaces.Loads;
  parameter Real R_l "Aux AC Load Resistance";
  parameter Real L_l "Aux AC Load Inductance";
  Modelica.Electrical.Machines.BasicMachines.InductionMachines.IM_SquirrelCage
    AC_Hydraulic_Pump(
    p=Data.p,
    fsNominal=Data.fsNominal,
    TsOperational=298.15,
    Rs=Data.Rs,
    TsRef=298.15,
    alpha20s(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    Lssigma=Data.Lss,
    Jr=Data.J,
    frictionParameters(
      PRef=0.000001,
      wRef=1.0471975511966e-7,
      power_w=0.000001),
    phiMechanical(fixed=true, start=0),
    wMechanical(start=0, fixed=true),
    statorCoreParameters(VRef=115),
    strayLoadParameters(power_w=0.00001),
    Lm=Data.Lm,
    Lrsigma=Data.Lr,
    Rr=Data.Rr,
    TrRef=298.15,
    alpha20r(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    TrOperational=298.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={6,-28})));

  Modelica.Electrical.Polyphase.Basic.Star star(final m=3) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-22,-16})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        origin={-50,-16},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    HydraulicPumpLoad(
    useSupport=false,
    tau_nominal=-L,
    w_nominal=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={8,-66})));
  Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch switch annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={46,-18})));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=startTime,
      startValue=false)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-4,12})));
  Modelica.Blocks.Routing.BooleanReplicator booleanReplicator(nout=3)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={28,12})));
  parameter Modelica.Units.SI.Time startTime=20
    "Time instant of Motor Connection";
    parameter Real L = -28/500
    "Mechanical Linear Load Slope";
  replaceable parameter Records.Boeing747electricalModel.InductionMachine.IM_30KVA
                                                                              Data
    constrainedby CHEETA.Records.Boeing747electricalModel.Base.IM
    annotation (Placement(transformation(extent={{-52,-76},{-32,-56}})));
equation
  connect(star.pin_n,ground1. p)
    annotation (Line(points={{-32,-16},{-40,-16}},
                                                 color={0,0,255}));
  connect(AC_Hydraulic_Pump.plug_sn,star. plug_p)
    annotation (Line(points={{0,-18},{0,-16},{-12,-16}},
                                                      color={0,0,255}));
  connect(HydraulicPumpLoad.flange,AC_Hydraulic_Pump. flange) annotation (
      Line(points={{18,-66},{26,-66},{26,-28},{16,-28}},
                                                     color={0,0,0}));
  connect(switch.plug_p,AC_Hydraulic_Pump. plug_sp)
    annotation (Line(points={{36,-18},{12,-18}}, color={0,0,255}));
  connect(booleanReplicator.u,booleanStep. y)
    annotation (Line(points={{16,12},{7,12}},    color={255,0,255}));
  connect(booleanReplicator.y,switch. control) annotation (Line(points={{39,12},
          {46,12},{46,-6}},                 color={255,0,255}));
  connect(switch.plug_n, AC_in) annotation (Line(points={{56,-18},{56,48},{-60,
          48},{-60,0},{-100,0}},
                             color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
      Icon(coordinateSystem(extent={{-100,-80},{100,100}})));
end Induction_Motor;
