within CHEETA.Aircraft.Electrical.Machines.Examples.Boeing747;
model SynGenwAVR
  import CHEETA;
  extends CHEETA.Aircraft.Electrical.Interfaces.Generation;
  replaceable CHEETA.Aircraft.Controls.AVR.AVR_TypeI AVR(
    T_R=Data_AVR.T_R,
    T_C=Data_AVR.T_C,
    T_B=Data_AVR.T_B,
    K_A=Data_AVR.K_A,
    T_A=Data_AVR.T_A,
    K_E=Data_AVR.K_E,
    T_E=Data_AVR.T_E,
    K_F=Data_AVR.K_F,
    T_F=Data_AVR.T_F,
    Vmax=Data_AVR.Vmax,
    Vmin=Data_AVR.Vmin) constrainedby
    CHEETA.Aircraft.Electrical.Interfaces.Controller
    annotation (Placement(transformation(extent={{-184,-4},{-146,-24}})));
  replaceable CHEETA.Records.Boeing747electricalModel.Controllers.TypeI_AVR
                                                                Data_AVR
    constrainedby CHEETA.Records.Boeing747electricalModel.Base.AVR
    annotation (Placement(transformation(extent={{18,24},{38,44}})));

  Modelica.Blocks.Sources.Constant Vref(k=1) annotation (Placement(
        transformation(extent={{-204,-22},{-196,-14}})));
  parameter Modelica.SIunits.Inertia J=0.02 "Rotor's moment of inertia";

  replaceable parameter CHEETA.Records.Boeing747electricalModel.SynchronousMachine.SM300kVA
    SG_Data constrainedby CHEETA.Records.Boeing747electricalModel.Base.Synch
    annotation (Placement(transformation(extent={{56,24},{76,44}})));

  Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited
    smee(
    fsNominal=SG_Data.fsNominal,
    Rs=SG_Data.Rs,
    TsRef=SG_Data.TsRef,
    Lssigma=SG_Data.Lssigma,
    phiMechanical(fixed=false),
    wMechanical(fixed=false),
    Lmd=SG_Data.Lmd,
    Lmq=SG_Data.Lmq,
    Lrsigmad=SG_Data.Lrsigmad,
    Lrsigmaq=SG_Data.Lrsigmaq,
    Rrd=SG_Data.Rrd,
    Rrq=SG_Data.Rrq,
    TrRef=SG_Data.TrRef,
    VsNominal=SG_Data.VsNominal,
    IeOpenCircuit=SG_Data.IeOpenCircuit,
    Re=SG_Data.Re,
    TeRef=SG_Data.TeRef,
    sigmae=SG_Data.sigmae,
    p=SG_Data.Poles,
    Jr=0.02,
    Js=0.29,
    useDamperCage=useDamperCage,
    statorCoreParameters(VRef=115),
    strayLoadParameters(IRef=100),
    brushParameters(ILinear=0.01),
    TsOperational=293.15,
    alpha20s=SG_Data.alpha20s,
    TrOperational=293.15,
    alpha20r=SG_Data.alpha20r,
    alpha20e(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    TeOperational=293.15)
    annotation (Placement(transformation(extent={{-62,-20},{-42,0}})));

  Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle
    rotorDisplacementAngle(p=SG_Data.Poles)
                                annotation (Placement(transformation(
        origin={-8,-10},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor
    mechanicalPowerSensor
    annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-62,-4},{-42,16}})));
  Modelica.Mechanics.Rotational.Sources.Speed speed(exact=false)
                                                    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={62,0})));
  Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-28,28})));
  Modelica.Electrical.MultiPhase.Blocks.QuasiRMS rms annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-68,46})));
  Modelica.Blocks.Math.Gain PerUnitConversion(k=1/SG_Data.VsNominal)
                                                                  annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-120,40})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-84,-14})));
  Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
      Placement(transformation(
        origin={-84,-40},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Modelica.Blocks.Math.Gain PerUnitConversion1(k=SG_Data.VsNominal)
                                                                  annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-120,-14})));
  parameter Boolean useDamperCage=true "Enable / disable damper cage";
equation
  connect(Vref.y, AVR.Reference) annotation (Line(points={{-195.6,-18},{-184,
          -18}},                            color={0,0,127}));
  connect(rotorDisplacementAngle.plug_n,smee. plug_sn) annotation (Line(
        points={{-2,-3.55271e-15},{-2,12},{-58,12},{-58,0}},
                                                        color={0,0,255}));
  connect(rotorDisplacementAngle.plug_p,smee. plug_sp)
    annotation (Line(points={{-14,0},{-46,0}},   color={0,0,255}));
  connect(terminalBox.plug_sn,smee. plug_sn) annotation (Line(
      points={{-58,0},{-58,0}},
      color={0,0,255}));
  connect(terminalBox.plug_sp,smee. plug_sp) annotation (Line(
      points={{-46,0},{-46,0}},
      color={0,0,255}));
  connect(smee.flange,rotorDisplacementAngle. flange) annotation (Line(
      points={{-42,-10},{-18,-10}}));
  connect(smee.flange,mechanicalPowerSensor. flange_a) annotation (Line(
      points={{-42,-10},{-30,-10},{-30,-30},{-10,-30}}));
  connect(speed.flange,mechanicalPowerSensor. flange_b)
    annotation (Line(points={{52,8.88178e-16},{20,8.88178e-16},{20,-30},{10,-30}},
                                                 color={0,0,0}));
  connect(voltageSensor.plug_p,rotorDisplacementAngle. plug_p)
    annotation (Line(points={{-38,28},{-38,0},{-14,0}},color={0,0,255}));
  connect(rms.u,voltageSensor. v)
    annotation (Line(points={{-56,46},{-28,46},{-28,39}},
                                                        color={0,0,127}));
  connect(signalVoltage.p,smee. pin_ep) annotation (Line(points={{-84,-4},{-84,
          -2},{-62,-2},{-62,-4}},
                              color={0,0,255}));
  connect(signalVoltage.n,smee. pin_en) annotation (Line(points={{-84,-24},{-62,
          -24},{-62,-16}},           color={0,0,255}));
  connect(speed.w_ref, w_ref) annotation (Line(points={{74,-1.9984e-15},{72,
          -1.9984e-15},{72,0},{110,0}},
                           color={0,0,127}));
  connect(PerUnitConversion.u, rms.y)
    annotation (Line(points={{-108,40},{-94,40},{-94,46},{-79,46}},
                                                  color={0,0,127}));
  connect(groundExcitation.p,signalVoltage. n) annotation (Line(points={{-84,-30},
          {-84,-24}},                     color={0,0,255}));
  connect(PerUnitConversion1.y, signalVoltage.v)
    annotation (Line(points={{-109,-14},{-96,-14}}, color={0,0,127}));
  connect(PerUnitConversion1.u, AVR.Actuation)
    annotation (Line(points={{-132,-14},{-145,-14}}, color={0,0,127}));
  connect(PerUnitConversion.y, AVR.Measurement) annotation (Line(points={{-131,
          40},{-194,40},{-194,-10},{-184,-10}}, color={0,0,127}));
  connect(AC_out, terminalBox.plugSupply) annotation (Line(points={{-222,2},{
          -136,2},{-136,6},{-52,6},{-52,2}}, color={0,0,255}));
  connect(voltageSensor.plug_n, smee.plug_sn) annotation (Line(points={{-18,28},
          {-2,28},{-2,12},{-58,12},{-58,0}}, color={0,0,255}));
 annotation (Placement(visible=false,
      transformation(extent={{-70,8},{-70,8}}),
      iconTransformation(extent={{-70,8},{-70,8}})),
              Icon(coordinateSystem(extent={{-220,-100},{100,100}}),
                   graphics={
        Rectangle(
          extent={{-106,68},{14,-52}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,128,255}),
        Rectangle(
          extent={{-106,68},{-126,-52}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={128,128,128}),
        Rectangle(
          extent={{14,12},{82,-8}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Rectangle(
          extent={{-106,78},{-26,58}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-116,-82},{-106,-82},{-76,-12},{-26,-12},{4,-82},{14,-82},{14,
              -92},{-116,-92},{-116,-82}},
          fillPattern=FillPattern.Solid),
        Ellipse(extent={{-204,42},{-136,-26}},lineColor={0,0,255}),
        Line(points={{-170,58},{-170,28},{-200,28},{-200,4}},  color={0,0,255}),
        Line(points={{-200,4},{-199,9},{-195,13},{-190,14},{-185,13},{-181,9},{-180,
              4}},           color={0,0,255}),
        Line(points={{-180,4},{-179,9},{-175,13},{-170,14},{-165,13},{-161,9},{-160,
              4}},       color={0,0,255}),
        Line(points={{-160,4},{-159,9},{-155,13},{-150,14},{-145,13},{-141,9},{-140,
              4}},  color={0,0,255}),
        Line(points={{-170,-42},{-170,-12},{-140,-12},{-140,6}},color={0,
              0,255})}),
    Diagram(coordinateSystem(extent={{-220,-100},{100,100}})));
end SynGenwAVR;
