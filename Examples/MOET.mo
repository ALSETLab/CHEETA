within CHEETA.Examples;
package MOET
  model SingleGenerator "Part of a MOET airplane with a single generator"
    extends DymolaModels.Icons.Basic.Example;

    inner ElectricPowerSystems.SettingsEPS settingsEPS(variableFrequency=true)
   annotation (Placement(transformation(extent={{80,160},{100,180}})));
    ElectricPowerSystems.ThreePhase.Generation.ExternalExcited SG1(
      wiring="Star",
      pp=2,
      eta=0.9,
      variableTorqueConstant=true) "Variable Frequency Starter Generator of power system 1"
      annotation (Placement(transformation(extent={{-10,150},{10,130}})));
    inner ElectricPowerSystems.OnePhase.Basic.PowerGround powerGround
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={0,170})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=GCU1.w_nom, phi(start=0, fixed=true))
   annotation (Placement(transformation(extent={{-80,130},{-60,150}})));
    ElectricPowerSystems.ThreePhase.Conversion.TransformerRectifier ATRU1(
      R1=1e-6,
      L1=1e-9,
      L2=1e-9,
      R2=1e-6,
      Rc=1e6,
      Lm=1e3,
      eta=0.95,
      useHeatPort=true) annotation (Placement(transformation(extent={{-90,-20},{-70,0}})));
    ElectricPowerSystems.OnePhase.Consumption.Drive ECS1(torqueInput=false, eta=0.98,
      useHeatPort=true,
      flange(phi(start=0, fixed=true)))
   annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
    Modelica.Blocks.Sources.Ramp speed_ECS1(
      duration=0.25,
      height=100,
      startTime=0.65) annotation (Placement(transformation(extent={{-70,-54},{-78,-46}})));
    ElectricPowerSystems.OnePhase.Basic.PowerGround.SignalGround signalGround
   annotation (Placement(transformation(
       extent={{-4,-4},{4,4}},
       rotation=90,
       origin={-56,-48})));
    Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque linearSpeedDependentTorque(w_nominal=100, tau_nominal=-300)
   annotation (Placement(transformation(
       extent={{-10,-10},{10,10}},
       rotation=90,
       origin={-80,-102})));
    ElectricPowerSystems.ThreePhase.Consumption.Load     wips(
      P_ref=60e3,
      alpha=2,
      beta=2,
      V_ref=230)
   annotation (Placement(transformation(
       extent={{-10,-10},{10,10}},
       rotation=0,
       origin={-20,-70})));
    ElectricPowerSystems.ThreePhase.Conversion.Rectifier CRU1(
      uncontrolled=false,
      eta=0.98,
      V=270,
      useHeatPort=true,
      enableSmoothing=false,
      variableVoltageDC=false)
                              "Controlled Rectifier Unit 1"
   annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
    ElectricPowerSystems.Common.Blocks.GeneratorControl GCU1(pp=SG1.pp, w_nom=2*Modelica.Constants.pi
                                                                                  *900/SG1.pp) "Generator Control Unit of VFSG1"
      annotation (Placement(transformation(extent={{-40,110},{-20,130}})));
    ElectricPowerSystems.ThreePhase.Distribution.IdealOpeningSwitch S1
   annotation (Placement(transformation(
       extent={{-10,-10},{10,10}},
       rotation=270,
       origin={0,70})));
    ElectricPowerSystems.OnePhase.Consumption.Drive EMA1(
      enableTorqueLimitation=true,
      maxPower=20e3,
      maxTorque=200,
      eta=0.95,
      useHeatPort=true)
                annotation (Placement(transformation(extent={{20,-80},{40,-60}})));
    Modelica.Blocks.Sources.Step tau_EMA1(height=EMA1.maxTorque, startTime=0.25)
      annotation (Placement(transformation(extent={{20,-54},{28,-46}})));
    Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque linearSpeedDependentTorque1(tau_nominal=-EMA1.maxTorque,
        w_nominal=EMA1.maxPower/EMA1.maxTorque)
   annotation (Placement(transformation(
       extent={{-10,-10},{10,10}},
       rotation=90,
       origin={30,-138})));
    Modelica.Mechanics.Rotational.Components.Inertia jEMA1(J=30e-3,
      phi(fixed=true, start=0),
      w(fixed=true, start=0))
   annotation (Placement(transformation(
       extent={{-10,10},{10,-10}},
       rotation=270,
       origin={30,-110})));
    ElectricPowerSystems.ThreePhase.Conversion.Rectifier CRU2(
      uncontrolled=false,
      eta=0.98,
      V=270,
      useHeatPort=true,
      enableSmoothing=false,
      variableVoltageDC=false)
                              "Controlled Rectifier Unit 1"
   annotation (Placement(transformation(extent={{80,-30},{100,-10}})));
    ElectricPowerSystems.OnePhase.Consumption.Drive EMA2(
      enableTorqueLimitation=true,
      maxPower=5e3,
      maxTorque=50,
      eta=0.9,
      useHeatPort=true)
               annotation (Placement(transformation(extent={{80,-80},{100,-60}})));
    Modelica.Blocks.Sources.Step tau_EMA2(height=EMA2.maxTorque, startTime=0.5)
      annotation (Placement(transformation(extent={{80,-54},{88,-46}})));
    Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque linearSpeedDependentTorque2(
        w_nominal=EMA2.maxPower/EMA2.maxTorque, tau_nominal=-EMA2.maxTorque)
   annotation (Placement(transformation(
       extent={{-10,-10},{10,10}},
       rotation=90,
       origin={90,-138})));
    Modelica.Blocks.Sources.BooleanConstant controlS1(k=false)
      annotation (Placement(transformation(
          extent={{-7,7},{7,-7}},
          rotation=180,
          origin={27,77})));
    Modelica.Mechanics.Rotational.Components.Inertia jEMA2(J=10e-3,
      phi(fixed=true, start=0),
      w(fixed=true, start=0))
   annotation (Placement(transformation(
       extent={{-10,10},{10,-10}},
       rotation=270,
       origin={90,-110})));
    ElectricPowerSystems.ThreePhase.Distribution.BusBar AC_ESS annotation (Placement(transformation(extent={{14,0},{106,2}})));
    ElectricPowerSystems.OnePhase.Basic.PowerGround.SignalGround signalGround2
   annotation (Placement(transformation(
       extent={{-4,-4},{4,4}},
       rotation=90,
       origin={48,-50})));
    ElectricPowerSystems.OnePhase.Basic.PowerGround.SignalGround signalGround3
   annotation (Placement(transformation(
       extent={{-4,-4},{4,4}},
       rotation=90,
       origin={108,-50})));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ambient(T=298.15)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-30,-140})));
    ElectricPowerSystems.ThreePhase.Distribution.CablePi               cableGen1(
      f_ref=400,
      l(displayUnit="m") = 10,
      r_ph=1e-3,
      x_ph=1e-3,
      b_ph=1e-6,
      R_cc=0,
      X_cc=0,
      B_cc=0) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,100})));
    ElectricPowerSystems.ThreePhase.Distribution.CablePi               cableATRU1(
      l(displayUnit="m") = 20,
      f_ref=400,
      r_ph=1e-3,
      x_ph=1e-3,
      b_ph=1e-6,
      R_cc=0,
      X_cc=0,
      B_cc=0) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-40,30})));
    ElectricPowerSystems.ThreePhase.Distribution.CablePi               cableWIPS(
      l(displayUnit="m") = 20,
      f_ref=400,
      r_ph=1e-3,
      x_ph=1e-3,
      b_ph=1e-6,
      R_cc=0,
      X_cc=0,
      B_cc=0) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-20,-10})));
    ElectricPowerSystems.ThreePhase.Distribution.CablePi               cableESS(
      l(displayUnit="m") = 20,
      f_ref=400,
      r_ph=1e-3,
      x_ph=1e-3,
      b_ph=1e-6,
      R_cc=0,
      X_cc=0,
      B_cc=0) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={42,30})));
    ElectricPowerSystems.ThreePhase.Distribution.BusBar HVAC1 annotation (Placement(transformation(extent={{-46,48},{46,50}})));
    ElectricPowerSystems.ThreePhase.Distribution.IdealClosingSwitch S1_WIPS
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-20,30})));
    Modelica.Blocks.Sources.BooleanStep controlS1_WIPS(startTime=0.1)
      annotation (Placement(transformation(
          extent={{-7,7},{7,-7}},
          rotation=180,
          origin={7,37})));
  equation
    connect(powerGround.p, SG1.starPoint) annotation (Line(points={{-8.88178e-16,160},{0,160},{0,150}},
                                                                                              color={35,82,132}));
    connect(constantSpeed.flange, SG1.flange) annotation (Line(points={{-60,140},{-10,140}}, color={0,0,0}));
    connect(ECS1.p, ATRU1.p) annotation (Line(points={{-90,-70},{-100,-70},{-100,-10},{-90,-10}}, color={35,82,132}));
    connect(ATRU1.n, ECS1.n) annotation (Line(points={{-70,-10},{-60,-10},{-60,-70},{-70,-70}}, color={35,82,132}));
    connect(speed_ECS1.y, ECS1.speed) annotation (Line(points={{-78.4,-50},{-84,-50},{-84,-58}}, color={0,0,127}));
    connect(ATRU1.n, signalGround.p) annotation (Line(points={{-70,-10},{-60,-10},{-60,-48}}, color={35,82,132}));
    connect(GCU1.kEMF, SG1.k_EMF_variable) annotation (Line(points={{-19,120},{-6,120},{-6,128}}, color={0,0,127}));
    connect(GCU1.flange, SG1.flange) annotation (Line(points={{-40,120},{-46,120},{-46,140},{-10,140}}, color={0,0,0}));
    connect(CRU1.p0, AC_ESS.pBar) annotation (Line(points={{30,-10},{30,0.5},{60,0.5}}, color={35,82,132}));
    connect(tau_EMA1.y, EMA1.desiredTorque) annotation (Line(points={{28.4,-50},{34,-50},{34,-58}}, color={0,0,127}));
    connect(jEMA1.flange_b, linearSpeedDependentTorque1.flange) annotation (Line(points={{30,-120},{30,-128}}, color={0,0,0}));
    connect(EMA1.flange, jEMA1.flange_a) annotation (Line(points={{30,-80},{30,-100}},  color={0,0,0}));
    connect(tau_EMA2.y, EMA2.desiredTorque) annotation (Line(points={{88.4,-50},{94,-50},{94,-58}}, color={0,0,127}));
    connect(jEMA2.flange_b, linearSpeedDependentTorque2.flange) annotation (Line(points={{90,-120},{90,-128}}, color={0,0,0}));
    connect(EMA2.flange, jEMA2.flange_a) annotation (Line(points={{90,-80},{90,-100}},  color={0,0,0}));
    connect(CRU2.p0, AC_ESS.pBar) annotation (Line(points={{90,-10},{90,0.5},{60,0.5}}, color={35,82,132}));
    connect(EMA1.n, signalGround2.p) annotation (Line(points={{40,-70},{44,-70},{44,-50}}, color={35,82,132}));
    connect(EMA2.n, signalGround3.p) annotation (Line(points={{100,-70},{104,-70},{104,-50}}, color={35,82,132}));
    connect(ATRU1.heatPort, ambient.port)
      annotation (Line(points={{-90,-20},{-104,-20},{-104,-160},{-30,-160},{-30,-150}}, color={191,0,0}));
    connect(ECS1.heatPort, ambient.port)
      annotation (Line(points={{-90,-80},{-104,-80},{-104,-160},{-30,-160},{-30,-150}}, color={191,0,0}));
    connect(CRU1.heatPort, ambient.port)
      annotation (Line(points={{20,-30},{10,-30},{10,-160},{-30,-160},{-30,-150}}, color={191,0,0}));
    connect(EMA1.heatPort, ambient.port)
      annotation (Line(points={{20,-80},{10,-80},{10,-160},{-30,-160},{-30,-150}}, color={191,0,0}));
    connect(CRU2.heatPort, ambient.port)
      annotation (Line(points={{80,-30},{70,-30},{70,-160},{-30,-160},{-30,-150}}, color={191,0,0}));
    connect(EMA2.heatPort, ambient.port)
      annotation (Line(points={{80,-80},{70,-80},{70,-160},{-30,-160},{-30,-150}}, color={191,0,0}));
    connect(linearSpeedDependentTorque.flange, ECS1.flange) annotation (Line(points={{-80,-92},{-80,-80}},  color={0,0,0}));
    connect(CRU1.p, EMA1.p) annotation (Line(points={{20,-20},{16,-20},{16,-70},{20,-70}}, color={35,82,132}));
    connect(signalGround2.p, CRU1.n) annotation (Line(points={{44,-50},{44,-20},{40,-20}}, color={35,82,132}));
    connect(CRU2.p, EMA2.p) annotation (Line(points={{80,-20},{76,-20},{76,-70},{80,-70}}, color={35,82,132}));
    connect(CRU2.n, signalGround3.p) annotation (Line(points={{100,-20},{104,-20},{104,-50}}, color={35,82,132}));
    connect(SG1.p, cableGen1.p) annotation (Line(points={{0,130},{0,110}}, color={35,82,132}));
    connect(cableGen1.n, S1.p) annotation (Line(points={{0,90},{0,80}}, color={35,82,132}));
    connect(S1.control, controlS1.y) annotation (Line(points={{8,77},{19.3,77}}, color={255,0,255}));
    connect(cableATRU1.n,ATRU1.p0)  annotation (Line(points={{-40,20},{-40,10},{-80,10},{-80,0}}, color={35,82,132}));
    connect(cableWIPS.n, wips.p) annotation (Line(points={{-20,-20},{-20,-60}}, color={35,82,132}));
    connect(cableESS.n, AC_ESS.pUp) annotation (Line(points={{42,20},{42,12},{60,12},{60,1.5}},
                                                                                              color={35,82,132}));
    connect(SG1.angle, settingsEPS.angleInput) annotation (Line(
        points={{11,140},{60,140},{60,170},{80,170}},
        color={0,127,127},
        pattern=LinePattern.Dash));
    connect(S1.n, HVAC1.pUp) annotation (Line(points={{-1.77636e-15,60},{-1.77636e-15,55},{0,55},{0,49.5}},
                                                                                                          color={35,82,132}));
    connect(cableATRU1.p, HVAC1.pBar)
      annotation (Line(points={{-40,40},{-40,44},{-40,48.5},{-7.10543e-15,48.5}}, color={35,82,132}));
    connect(cableESS.p, HVAC1.pBar) annotation (Line(points={{42,40},{42,48.5},{-7.10543e-15,48.5}}, color={35,82,132}));
    connect(S1_WIPS.control, controlS1_WIPS.y) annotation (Line(points={{-12,37},{-0.7,37}}, color={255,0,255}));
    connect(cableWIPS.p, S1_WIPS.n) annotation (Line(points={{-20,0},{-20,20}}, color={35,82,132}));
    connect(S1_WIPS.p, HVAC1.pBar) annotation (Line(points={{-20,40},{-20,48.5},{-7.10543e-15,48.5}}, color={35,82,132}));
    annotation (
      Icon(coordinateSystem(extent={{-100,-100},{100,100}}, initialScale=0.1)),
           Diagram(
     coordinateSystem(extent={{-120,-180},{120,180}}, initialScale=0.1),
     graphics={Rectangle(
         extent={{-48,-30},{-114,-32}},
         lineColor={35,82,132},
         fillColor={35,82,132},
         fillPattern=FillPattern.Solid), Text(
         extent={{-120,-26},{-102,-28}},
         lineColor={35,82,132},
         fillColor={35,82,132},
         fillPattern=FillPattern.None,
            textString="HVDC1")}),
   Documentation(info="<html>
<hr>
<h4>Tutorials</h4>
<p>Find additional information about this example in the following tutorial: <a
        href=\"ElectricPowerSystems.UsersGuide.Tutorials.MOET\">MOET - Single Generator Architecture</a></p>

<hr>
<h4>MOET architecture</h4>
<p>The following example is based on a generic architecture, proposed and studied within the MOET european project.<br>
    This architecture considers a large aircraft with four engines.<br>
    The power generation is realized thanks to the aircraft engines connected directly to Variable Frequency Starter
    Generators (VFSGs). The VFSGs convert the mechanical power into electrical power.<br>
    The electrical power is then distributed to the consumers through High Voltage AC bus bar and DC bus bars.</p>
<p>Typical loads on the system are, for example: </p>
<ul>
    <li>Environmental Control System (ECS),</li>
    <li>Electro-Mechanical Actuators (EMA),</li>
    <li>Wing Icing Protection System (WIPS).</li>
</ul>

<p>The architecture proposed in MOET project is symmetric. This enables considering only half of it for the illustration
    of a twin-generator aircraft <a href=\"ElectricPowerSystems.UsersGuide.Literature\">[WU10a]</a>.<br>
    The following picture represents the power system of such twin-generator aircraft.</p>
<p><img src=\"modelica://ElectricPowerSystems/Resources/Images/MOETarchitectureHalf.png\"/></p>

<p>In this example, the first circuit is modelled with the components available in the EPS library.</p>
</html>"),
      __Dymola_Commands(file="modelica://ElectricPowerSystems/Resources/Scripts/plot/Example_MOET_SingleGenerator.mos" "plot"));
  end SingleGenerator;
annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Polygon(
          origin={8.0,14.0},
          lineColor={78,138,73},
          fillColor={78,138,73},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
end MOET;
