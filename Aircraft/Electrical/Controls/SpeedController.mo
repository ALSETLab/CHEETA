within CHEETA.Aircraft.Electrical.Controls;
model SpeedController
  import Modelica.Constants.pi;
  constant Integer m=3 "Number of phases";
  parameter Real K_ref = 1 "Reference Proportional Gain";
  parameter Real T_ref = 0.1 "Reference Integrator Time Constant";
  parameter Integer p "Number of pole pairs";
  parameter Modelica.SIunits.Frequency fsNominal "Nominal frequency";
  parameter Modelica.SIunits.Voltage VsOpenCircuit
    "Open circuit RMS voltage per phase @ fsNominal";
  parameter Modelica.SIunits.Resistance Rs "Stator resistance per phase";
  parameter Modelica.SIunits.Inductance Ld "Inductance in d-axis";
  parameter Modelica.SIunits.Inductance Lq "Inductance in q-axis";
  //Decoupling
  parameter Boolean decoupling=false "Use decoupling network";
  final parameter Modelica.SIunits.MagneticFlux psiM=sqrt(2)*VsOpenCircuit/
      (2*pi*fsNominal);
  Modelica.SIunits.AngularVelocity omega;
  Modelica.SIunits.Voltage Vd;
  Modelica.SIunits.Voltage Vq;
  Modelica.SIunits.Current iq_rms;
  Modelica.SIunits.Current id_rms;

  Modelica.Blocks.Interfaces.RealOutput a
    annotation (Placement(transformation(extent={{380,78},{400,98}})));
  Modelica.Blocks.Interfaces.RealOutput b
    annotation (Placement(transformation(extent={{380,-10},{400,10}}),
        iconTransformation(extent={{380,-10},{400,10}})));
  Modelica.Blocks.Interfaces.RealOutput c
    annotation (Placement(transformation(extent={{380,-120},{400,-100}}),
        iconTransformation(extent={{380,-120},{400,-100}})));
  Modelica.Electrical.Machines.Utilities.FromDQ
                            fromDQ(final p=p, final m=m)
    annotation (Placement(transformation(extent={{228,-4},{248,16}})));
  Modelica.Electrical.Machines.Utilities.ToDQ
                          toDQ(final p=p, final m=m)
                                          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-90,-64})));
  Modelica.Blocks.Math.Division division
    annotation (Placement(transformation(extent={{316,78},{336,98}})));
  Modelica.Blocks.Math.Division division1
    annotation (Placement(transformation(extent={{318,-10},{338,10}})));
  Modelica.Blocks.Math.Division division2
    annotation (Placement(transformation(extent={{322,-102},{342,-82}})));
  Modelica.Blocks.Math.Max max annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={228,-78})));
  Modelica.Blocks.Sources.Constant const(k=0.0001)
    annotation (Placement(transformation(extent={{140,-102},{160,-82}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1, uMin=-1)
    annotation (Placement(transformation(extent={{348,78},{368,98}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=1, uMin=-1)
    annotation (Placement(transformation(extent={{348,-10},{368,10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=1, uMin=-1)
    annotation (Placement(transformation(extent={{354,-102},{374,-82}})));
  Modelica.Blocks.Interfaces.RealInput Vdc1 annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={234,-120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={254,-100})));
  Modelica.Blocks.Interfaces.RealInput iActual[m] annotation (Placement(
        transformation(
        origin={-90,-120},
        extent={{20,-20},{-20,20}},
        rotation=270), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=270,
        origin={22,-100})));
  Modelica.Blocks.Interfaces.RealInput phi(unit="rad") annotation (Placement(
        transformation(
        origin={60,-120},
        extent={{20,-20},{-20,20}},
        rotation=270), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=270,
        origin={142,-100})));

  Modelica.Blocks.Routing.Multiplex2 multiplex2_1
    annotation (Placement(transformation(extent={{154,2},{174,22}})));
  Modelica.Blocks.Sources.Constant const1(k=0)
    annotation (Placement(transformation(extent={{-8,54},{12,74}})));
  Modelica.Blocks.Math.Add add1
                              [2](final k1=fill(+1, 2), final k2=fill(if
        decoupling then +1 else 0, 2))
    annotation (Placement(transformation(extent={{202,-4},{222,16}})));
  Modelica.Blocks.Sources.RealExpression deCoupling[2](y={Vd,Vq})
    annotation (Placement(transformation(extent={{130,-36},{150,-16}})));
  Modelica.Blocks.Continuous.LimPID Iq_ref(
    k=K_ref,
    Ti=T_ref,
    Td=0,
    yMax=500,
    yMin=0,
    withFeedForward=false,
    kFF=0.01)
    annotation (Placement(transformation(extent={{6,-4},{26,16}})));
  Modelica.Blocks.Routing.DeMultiplex demux1(n=2)
                                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={20,-42})));
  Modelica.Blocks.Continuous.LimPID Id(
    k=1/Rs,
    Ti=Ld/Rs,
    Td=0,
    yMax=200,
    yMin=-200,
    withFeedForward=false,
    kFF=0.01,
    initType=Modelica.Blocks.Types.InitPID.NoInit,
    xi_start=0,
    xd_start=0)
    annotation (Placement(transformation(extent={{102,54},{122,74}})));
  Modelica.Blocks.Continuous.LimPID Iq(
    k=1/Rs,
    Ti=Lq/Rs,
    Td=0,
    yMax=200,
    yMin=-200,
    withFeedForward=false,
    kFF=0.01,
    initType=Modelica.Blocks.Types.InitPID.NoInit,
    xi_start=0,
    xd_start=0)
    annotation (Placement(transformation(extent={{100,-4},{120,16}})));
  Modelica.Blocks.Interfaces.RealInput Speed annotation (Placement(
        transformation(
        origin={-100,-6},
        extent={{20,-20},{-20,20}},
        rotation=180), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-80,0})));
  Modelica.Blocks.Routing.DeMultiplex demux2(n=3)
                                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={270,6})));
  Modelica.Blocks.Math.Gain gain(k=1/sqrt(2))
    annotation (Placement(transformation(extent={{72,30},{92,50}})));
  Modelica.Blocks.Math.Gain gain1(k=1/sqrt(2))
    annotation (Placement(transformation(extent={{78,-56},{98,-36}})));
  parameter Real Iqmax=100 "Upper Limit of Currents";
  parameter Modelica.SIunits.Time ramping=10
    "Duration of ramp (= 0.0 gives a Step)";
  parameter Modelica.SIunits.Time startTime=20
    "Output y = offset for time < startTime";
  Modelica.Blocks.Interfaces.RealInput Ref "Reference signal input" annotation (
     Placement(transformation(extent={{-100,52},{-60,92}}), iconTransformation(
          extent={{-100,52},{-60,92}})));
protected
  constant Modelica.SIunits.Resistance unitResistance=1;

equation
  omega=Speed;
  Vd = (Rs*id_rms - omega*Lq*iq_rms);
  Vq = (Rs*iq_rms + omega*Ld*id_rms) + omega
      *psiM;
  iq_rms = toDQ.y[2]/sqrt(2);
  id_rms = toDQ.y[1]/sqrt(2);

  connect(iActual,toDQ. u) annotation (Line(
      points={{-90,-120},{-90,-76}}, color={0,0,127}));
  connect(max.u2, Vdc1)
    annotation (Line(points={{234,-90},{234,-120}}, color={0,0,127}));
  connect(const.y, max.u1)
    annotation (Line(points={{161,-92},{222,-92},{222,-90}}, color={0,0,127}));
  connect(limiter.u, division.y)
    annotation (Line(points={{346,88},{337,88}}, color={0,0,127}));
  connect(limiter2.u, division2.y) annotation (Line(points={{352,-92},{343,-92}},
                                color={0,0,127}));
  connect(limiter1.u, division1.y)
    annotation (Line(points={{346,0},{339,0}},   color={0,0,127}));
  connect(division.u2, division1.u2) annotation (Line(points={{314,82},{304,82},
          {304,-6},{316,-6}},   color={0,0,127}));
  connect(division2.u2, division1.u2) annotation (Line(points={{320,-98},{304,-98},
          {304,-6},{316,-6}},   color={0,0,127}));
  connect(max.y, division1.u2) annotation (Line(points={{228,-67},{228,-56},{304,
          -56},{304,-6},{316,-6}},   color={0,0,127}));
  connect(add1.y, fromDQ.u)
    annotation (Line(points={{223,6},{226,6}}, color={0,0,127}));
  connect(deCoupling.y, add1.u2) annotation (Line(points={{151,-26},{184,-26},{
          184,0},{200,0}}, color={0,0,127}));
  connect(add1.u1, multiplex2_1.y)
    annotation (Line(points={{200,12},{175,12}}, color={0,0,127}));
  connect(limiter.y, a)
    annotation (Line(points={{369,88},{390,88}}, color={0,0,127}));
  connect(limiter1.y, b) annotation (Line(points={{369,0},{374,0},{374,0},{390,0}},
                color={0,0,127}));
  connect(limiter2.y, c) annotation (Line(points={{375,-92},{382,-92},{382,-110},
          {390,-110}},color={0,0,127}));
  connect(demux1.u, toDQ.y) annotation (Line(points={{8,-42},{-90,-42},{-90,-53}},
                             color={0,0,127}));
  connect(Id.y, multiplex2_1.u1[1]) annotation (Line(points={{123,64},{
          126,64},{126,18},{152,18}}, color={0,0,127}));
  connect(Iq.y, multiplex2_1.u2[1])
    annotation (Line(points={{121,6},{152,6}}, color={0,0,127}));
  connect(Id.u_s, const1.y)
    annotation (Line(points={{100,64},{13,64}}, color={0,0,127}));
  connect(Iq_ref.u_m, Speed)
    annotation (Line(points={{16,-6},{-100,-6}}, color={0,0,127}));
  connect(Iq.u_s, Iq_ref.y)
    annotation (Line(points={{98,6},{27,6}}, color={0,0,127}));
  connect(division.u1, demux2.y[1]) annotation (Line(points={{314,94},{298,94},
          {298,10.6667},{280,10.6667}},color={0,0,127}));
  connect(division1.u1, demux2.y[2]) annotation (Line(points={{316,6},{298,6},{298,
          6},{280,6}}, color={0,0,127}));
  connect(division2.u1, demux2.y[3]) annotation (Line(points={{320,-86},{302,-86},
          {302,1.33333},{280,1.33333}}, color={0,0,127}));
  connect(demux2.u, fromDQ.y)
    annotation (Line(points={{258,6},{249,6}}, color={0,0,127}));
  connect(fromDQ.phi, phi) annotation (Line(points={{238,-6},{236,-6},{236,-46},
          {230,-46},{230,-62},{60,-62},{60,-120}}, color={0,0,127}));
  connect(toDQ.phi, phi) annotation (Line(points={{-78,-64},{-10,-64},{-10,-66},
          {60,-66},{60,-120}}, color={0,0,127}));
  connect(gain.y, Id.u_m)
    annotation (Line(points={{93,40},{112,40},{112,52}}, color={0,0,127}));
  connect(gain.u, demux1.y[1]) annotation (Line(points={{70,40},{62,40},{62,-38.5},
          {30,-38.5}}, color={0,0,127}));
  connect(gain1.y, Iq.u_m) annotation (Line(points={{99,-46},{99,-24},{110,-24},
          {110,-6}}, color={0,0,127}));
  connect(gain1.u, demux1.y[2]) annotation (Line(points={{76,-46},{64,-46},{64,-45.5},
          {30,-45.5}}, color={0,0,127}));
  connect(Iq_ref.u_s, Ref) annotation (Line(points={{4,6},{-38,6},{-38,72},{-80,
          72}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},
            {380,100}}), graphics={Text(
          extent={{50,36},{316,-24}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={215,215,215},
          fillPattern=FillPattern.None,
          textStyle={TextStyle.Bold},
          textString="PI Speed Control
"), Rectangle(
          extent={{-100,100},{380,-120}},
          lineColor={28,108,200},
          fillColor={215,215,215},
          fillPattern=FillPattern.None)}),
                                     Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-120},{380,100}})),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=500000,
      Tolerance=1e-08));
end SpeedController;
