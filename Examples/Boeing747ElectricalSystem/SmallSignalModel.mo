within CHEETA.Examples.Boeing747ElectricalSystem;
model SmallSignalModel
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.0001,11900;
        0.5,12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={306,208})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={204,208})));
  Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
    AC_Hydraulic_Pump(
    p=8,
    fsNominal=400,
    TsOperational=298.15,
    Rs=0.2761,
    TsRef=298.15,
    alpha20s(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    Lssigma=0.0002191,
    Jr=0.01,
    frictionParameters(
      PRef=0.000001,
      wRef=1.0471975511966e-7,
      power_w=0.000001),
    phiMechanical(fixed=true, start=0),
    wMechanical(start=0, fixed=true),
    statorCoreParameters(VRef=115),
    strayLoadParameters(power_w=0.00001),
    Lm=0.07614,
    Lrsigma=0.0002191,
    Rr=0.16,
    TrRef=298.15,
    alpha20r(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    TrOperational=298.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={14,198})));

  Aircraft.Electrical.Machines.Examples.Boeing747.SG_small_signal
                                               SG_with_AVR annotation (
      Placement(transformation(
        rotation=0,
        extent={{-17,-17},{17,17}},
        origin={145,205})));
  Modelica.Electrical.MultiPhase.Basic.Star star(final m=3) annotation (
      Placement(transformation(extent={{-2,198},{-22,218}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
                                                 annotation (Placement(
        transformation(
        origin={-56,208},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    HydraulicPumpLoad(
    useSupport=false,
    tau_nominal=-28/500,
    w_nominal=1) annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=0,
        origin={49,187})));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_ElectricalExcited
    Fuel_Pump(
    VaNominal=28,
    IaNominal=300/28,
    wNominal=198.96753472735,
    TaNominal=298.15,
    Ra=DataDCMotor.Ra,
    alpha20a(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    La=DataDCMotor.La,
    Jr=DataDCMotor.J,
    phiMechanical(start=0, fixed=false),
    wMechanical(start=0, fixed=false),
    ia(start=0, fixed=false),
    Re=DataDCMotor.Rf,
    alpha20e(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    Le=DataDCMotor.Lf,
    sigmae=DataDCMotor.Laf/DataDCMotor.Lf,
    ie(start=0, fixed=false)) annotation (Placement(transformation(
        extent={{-13,-13},{13,13}},
        rotation=0,
        origin={263,167})));

  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    FuelPumpLoad(
    useSupport=false,
    tau_nominal=-1.5/200,
    w_nominal=1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={304,166})));
  Modelica.Electrical.Analog.Basic.Resistor DC_Load_Lamp(R=1)
    annotation (Placement(transformation(extent={{204,144},{224,164}})));
  Modelica.Electrical.Analog.Basic.Resistor DC_Load_Heater(R=1)
    annotation (Placement(transformation(extent={{204,128},{224,148}})));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=10, startValue=
        false)
    annotation (Placement(transformation(extent={{-38,244},{-18,264}})));
  Modelica.Blocks.Routing.BooleanReplicator booleanReplicator(nout=3)
    annotation (Placement(transformation(extent={{-8,244},{12,264}})));
  Aircraft.Electrical.PowerElectronics.Converters.ACDC.Simulink_Averaged_Rectifier
    simulink_Averaged_Rectifier(P_fixed=1, V_rated=200)
    annotation (Placement(transformation(extent={{182,158},{198,174}})));
  Modelica.Electrical.MultiPhase.Ideal.IdealCommutingSwitch switch1(Goff={
        1e-5,1e-5,1e-5}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={46,208})));
  Modelica.Electrical.MultiPhase.Basic.Star star2 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={170,122})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{160,84},{180,104}})));
  Modelica.Electrical.MultiPhase.Basic.Capacitor capacitor(C={1e-6,1e-6,
        1e-6}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={170,148})));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={134,82})));
  Aircraft.Electrical.PowerElectronics.Converters.ACDC.Simulink_Averaged_Rectifier
    simulink_Averaged_Rectifier2(P_fixed=1, V_rated=200)
    annotation (Placement(transformation(extent={{96,32},{112,48}})));
  Aircraft.Electrical.PowerElectronics.Converters.ACDC.Simulink_Averaged_Rectifier
    simulink_Averaged_Rectifier1(P_fixed=1, V_rated=200)
    annotation (Placement(transformation(extent={{96,74},{112,90}})));
  Aircraft.Electrical.Transformers.Yd yd(
    L2=1e-6,
    L1=1e-6,
    N=200/(sqrt(2)*28))
    annotation (Placement(transformation(extent={{132,156},{158,176}})));
  Modelica.Electrical.MultiPhase.Basic.Star star4 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,-10})));
  Modelica.Electrical.Analog.Basic.Ground ground4
    annotation (Placement(transformation(extent={{30,-50},{50,-30}})));
  Modelica.Electrical.MultiPhase.Basic.Capacitor capacitor1(C={1e-6,1e-6,
        1e-6}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,22})));
  parameter Records.Boeing747electricalModel.PMSM.PMSM_2kVA smpmData(
    Js=0,
    frictionParameters(PRef=10),
    VsOpenCircuit=20)
    annotation (Placement(transformation(extent={{316,236},{336,256}})));
  Records.Boeing747electricalModel.DCMotor.FuelPumps.FuelPump_1 DataDCMotor
    annotation (Placement(transformation(extent={{286,234},{306,254}})));
  Modelica.Blocks.Math.Add add annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={254,208})));
  Modelica.Blocks.Interfaces.RealInput dw annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={268,280})));
  Modelica.Blocks.Interfaces.RealOutput dV annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={112,292})));
  Modelica.Blocks.Interfaces.RealOutput dEfd annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={220,292})));
  Modelica.Blocks.Interfaces.RealOutput dpm annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={180,294})));
  Modelica.Blocks.Interfaces.RealOutput dd annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={144,292})));
  Aircraft.Electrical.PowerElectronics.Converters.DCAC.Simulink_AverageInverter
                                                             simulink_AverageInverter
    annotation (Placement(transformation(extent={{180,32},{200,52}})));
  Aircraft.Controls.SpeedDrives.SpeedController
                                speedController(
    K_ref=1,
    p=smpm.p,
    fsNominal=smpm.fsNominal,
    VsOpenCircuit=smpm.VsOpenCircuit,
    Rs=Modelica.Electrical.Machines.Thermal.convertResistance(
        smpm.Rs,
        smpm.TsRef,
        smpm.alpha20s,
        smpm.TsOperational),
    Ld=smpm.Lssigma + smpm.Lmd,
    Lq=smpm.Lssigma + smpm.Lmq,
    decoupling=true,
    Iqmax=200) annotation (Placement(transformation(
        extent={{-25,-16},{25,16}},
        rotation=0,
        origin={319,54})));
  Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
    smpm(
    phiMechanical(start=0, fixed=false),
    wMechanical(start=0, fixed=false),
    useSupport=false,
    useThermalPort=false,
    p=smpmData.p,
    fsNominal=smpmData.fsNominal,
    Rs=smpmData.Rs,
    TsRef=smpmData.TsRef,
    Lszero=smpmData.Lszero,
    Lssigma=smpmData.Lssigma,
    Jr=smpmData.Jr,
    Js=smpmData.Js,
    frictionParameters=smpmData.frictionParameters,
    statorCoreParameters=smpmData.statorCoreParameters,
    strayLoadParameters=smpmData.strayLoadParameters,
    VsOpenCircuit=smpmData.VsOpenCircuit,
    Lmd=smpmData.Lmd,
    Lmq=smpmData.Lmq,
    useDamperCage=smpmData.useDamperCage,
    Lrsigmad=smpmData.Lrsigmad,
    Lrsigmaq=smpmData.Lrsigmaq,
    Rrd=smpmData.Rrd,
    Rrq=smpmData.Rrq,
    TrRef=smpmData.TrRef,
    permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
    TsOperational=293.15,
    alpha20s=smpmData.alpha20s,
    TrOperational=293.15,
    alpha20r=smpmData.alpha20r) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
        rotation=180,
        origin={282,-82})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    linearSpeedDependentTorque(tau_nominal=-0.2, w_nominal=1)
    annotation (Placement(transformation(extent={{298,-34},{278,-14}})));
  Modelica.Electrical.MultiPhase.Sensors.CurrentSensor currentSensor(m=3)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={236,20})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={302,-52})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={254,42})));
  Modelica.Electrical.MultiPhase.Basic.Star star3(final m=3)
                                                            annotation (
      Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={318,-114})));
  Modelica.Electrical.Analog.Basic.Ground ground3
                                                 annotation (Placement(
        transformation(
        origin={318,-140},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Modelica.Blocks.Sources.TimeTable BallScrewSpeedCommands1(table=[0.0,0; 0.4,0;
        0.5,40; 1,60; 2,0; 3,50; 4.5,-50; 8,0; 9,0.0; 10,0.0], timeScale=100)
                     annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={256,86})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=speedController.a)
    annotation (Placement(transformation(extent={{160,56},{180,76}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=speedController.b)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={190,82})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=speedController.c)
    annotation (Placement(transformation(extent={{230,58},{210,78}})));
equation
  connect(RPMtoRPS.y,SG_with_AVR. w_ref) annotation (Line(points={{193,208},
          {193,208.4},{163.7,208.4}}, color={0,0,127}));
  connect(star.pin_n,ground1. p)
    annotation (Line(points={{-22,208},{-46,208}},
                                                 color={0,0,255}));
  connect(AC_Hydraulic_Pump.plug_sn,star. plug_p)
    annotation (Line(points={{8,208},{-2,208}},       color={0,0,255}));
  connect(HydraulicPumpLoad.flange,AC_Hydraulic_Pump. flange) annotation (
      Line(points={{44,187},{36,187},{36,198},{24,198}},
                                                     color={0,0,0}));
  connect(DC_Load_Lamp.p,DC_Load_Heater. p)
    annotation (Line(points={{204,154},{204,138}},
                                                 color={0,0,255}));
  connect(DC_Load_Lamp.n,DC_Load_Heater. n)
    annotation (Line(points={{224,154},{224,138}},
                                                 color={0,0,255}));
  connect(FuelPumpLoad.flange, Fuel_Pump.flange) annotation (Line(points={{294,166},
          {294,167},{276,167}},         color={0,0,0}));
  connect(booleanStep.y, booleanReplicator.u) annotation (Line(points={{-17,
          254},{-16,254},{-16,254},{-10,254}}, color={255,0,255}));
  connect(DC_Load_Lamp.p, simulink_Averaged_Rectifier.DC_n) annotation (Line(
        points={{204,154},{202,154},{202,160},{199,160}}, color={0,0,255}));
  connect(Fuel_Pump.pin_an, Fuel_Pump.pin_en) annotation (Line(points={{255.2,
          180},{250,180},{250,159.2}},      color={0,0,255}));
  connect(booleanReplicator.y, switch1.control) annotation (Line(points={{
          13,254},{46,254},{46,220}}, color={255,0,255}));
  connect(SG_with_AVR.plugSupply, switch1.plug_n2) annotation (Line(points={{127.66,
          208.74},{93.83,208.74},{93.83,208},{56,208}},          color={0,0,
          255}));
  connect(switch1.plug_p, AC_Hydraulic_Pump.plug_sp)
    annotation (Line(points={{36,208},{20,208}},  color={0,0,255}));
  connect(capacitor.plug_n, star2.plug_p)
    annotation (Line(points={{170,138},{170,132}},
                                                 color={0,0,255}));
  connect(DC_Load_Lamp.n, simulink_Averaged_Rectifier.v_ref) annotation (Line(
        points={{224,154},{226,154},{226,166},{199,166}}, color={0,0,255}));
  connect(Fuel_Pump.pin_en, simulink_Averaged_Rectifier.v_ref) annotation (Line(
        points={{250,159.2},{250,166},{199,166}}, color={0,0,255}));
  connect(ground2.p, star2.pin_n)
    annotation (Line(points={{170,104},{170,112}},
                                                 color={0,0,255}));
  connect(yd.Secondary1, simulink_Averaged_Rectifier.AC)
    annotation (Line(points={{158.4,166},{181,166}}, color={0,0,255}));
  connect(capacitor.plug_p, simulink_Averaged_Rectifier.AC)
    annotation (Line(points={{170,158},{170,166},{181,166}}, color={0,0,255}));
  connect(voltageSensor.n, simulink_Averaged_Rectifier1.v_ref)
    annotation (Line(points={{124,82},{113,82}}, color={0,0,255}));
  connect(ground4.p,star4. pin_n)
    annotation (Line(points={{40,-30},{40,-20}}, color={0,0,255}));
  connect(capacitor1.plug_n, star4.plug_p) annotation (Line(points={{40,12},{40,
          0}},                                  color={0,0,255}));
  connect(capacitor1.plug_p, switch1.plug_n2) annotation (Line(points={{40,32},
          {40,166},{78,166},{78,208},{56,208}},      color={0,0,255}));
  connect(add.y, RPMtoRPS.u)
    annotation (Line(points={{243,208},{216,208}}, color={0,0,127}));
  connect(add.u1, timeTable.y) annotation (Line(points={{266,202},{280,202},
          {280,208},{295,208}}, color={0,0,127}));
  connect(dw, add.u2) annotation (Line(points={{268,280},{268,248},{268,214},
          {266,214}}, color={0,0,127}));
  connect(dV, SG_with_AVR.dV) annotation (Line(points={{112,292},{112,257},{
          131.4,257},{131.4,223.7}},    color={0,0,127}));
  connect(dd, SG_with_AVR.ddelta) annotation (Line(points={{144,292},{144,223.7},
          {140.92,223.7}},        color={0,0,127}));
  connect(dpm, SG_with_AVR.dpm) annotation (Line(points={{180,294},{180,274},{
          152,274},{152,223.7},{149.42,223.7}},
                                  color={0,0,127}));
  connect(dEfd, SG_with_AVR.dEfd) annotation (Line(points={{220,292},{220,257},
          {158.6,257},{158.6,223.7}},        color={0,0,127}));
  connect(simulink_Averaged_Rectifier.DC_p, Fuel_Pump.pin_ep) annotation (Line(
        points={{199,172},{224,172},{224,174.8},{250,174.8}}, color={0,0,255}));
  connect(Fuel_Pump.pin_ap, Fuel_Pump.pin_ep) annotation (Line(points={{270.8,
          180},{270.8,184},{224,184},{224,174.8},{250,174.8}}, color={0,0,255}));
  connect(yd.Primary, switch1.plug_n2) annotation (Line(points={{131.8,166},{
          116,166},{116,208.74},{93.83,208.74},{93.83,208},{56,208}}, color={0,
          0,255}));
  connect(simulink_Averaged_Rectifier1.AC, switch1.plug_n2) annotation (Line(
        points={{95,82},{40,82},{40,166},{78,166},{78,208},{56,208}}, color={0,
          0,255}));
  connect(simulink_Averaged_Rectifier2.AC, switch1.plug_n2) annotation (Line(
        points={{95,40},{40,40},{40,166},{78,166},{78,208},{56,208}}, color={0,
          0,255}));
  connect(linearSpeedDependentTorque.flange,smpm. flange)
    annotation (Line(points={{278,-24},{254,-24},{254,-82},{272,-82}},
                                                   color={0,0,0}));
  connect(currentSensor.plug_p, simulink_AverageInverter.positivePlug)
    annotation (Line(points={{236,30},{236,41.6667},{200.769,41.6667}}, color={
          0,0,255}));
  connect(currentSensor.plug_n,smpm. plug_sp) annotation (Line(points={{236,10},
          {236,-92},{276,-92}},        color={0,0,255}));
  connect(speedSensor.flange,smpm. flange) annotation (Line(points={{254,32},{
          254,-82},{272,-82}},                 color={0,0,0}));
  connect(speedSensor.w,speedController. Speed) annotation (Line(points={{254,53},
          {254,55.4545},{296.083,55.4545}},                      color={0,0,
          127}));
  connect(star3.pin_n,ground3. p)
    annotation (Line(points={{318,-124},{318,-130}}, color={0,0,255}));
  connect(star3.plug_p,smpm. plug_sn) annotation (Line(points={{318,-104},{318,
          -92},{288,-92}},                             color={0,0,255}));
  connect(voltageSensor.p, simulink_AverageInverter.pin_p) annotation (Line(
        points={{144,82},{166,82},{166,48.5},{179.231,48.5}}, color={0,0,255}));
  connect(simulink_Averaged_Rectifier1.DC_p, simulink_AverageInverter.pin_p)
    annotation (Line(points={{113,88},{160,88},{160,82},{166,82},{166,48.5},{
          179.231,48.5}}, color={0,0,255}));
  connect(angleSensor.flange, smpm.flange) annotation (Line(points={{292,-52},{
          254,-52},{254,-82},{272,-82}}, color={0,0,0}));
  connect(currentSensor.i, speedController.iActual) annotation (Line(points={{247,20},
          {306.708,20},{306.708,40.9091}},         color={0,0,127}));
  connect(speedController.phi, angleSensor.phi) annotation (Line(points={{319.208,
          40.9091},{319.208,-52},{313,-52}},         color={0,0,127}));
  connect(voltageSensor.v, speedController.Vdc1) annotation (Line(points={{134,
          93},{132,93},{132,104},{352,104},{352,34},{330.875,34},{330.875,
          40.9091}}, color={0,0,127}));
  connect(BallScrewSpeedCommands1.y, speedController.Ref) annotation (Line(
        points={{256,75},{256,65.9273},{296.083,65.9273}}, color={0,0,127}));
  connect(simulink_AverageInverter.pin_n, simulink_Averaged_Rectifier2.DC_n)
    annotation (Line(points={{179.231,37},{142,37},{142,34},{113,34}}, color={0,
          0,255}));
  connect(simulink_AverageInverter.ma, realExpression.y) annotation (Line(
        points={{183.385,53.8333},{182,53.8333},{182,66},{181,66}}, color={0,0,
          127}));
  connect(simulink_AverageInverter.mb, realExpression1.y) annotation (Line(
        points={{190.769,53.8333},{190,53.8333},{190,71}}, color={0,0,127}));
  connect(realExpression2.y, simulink_AverageInverter.mc) annotation (Line(
        points={{209,68},{202,68},{202,53.8333},{198.462,53.8333}}, color={0,0,
          127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-60},
            {360,280}})),            Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-80,-60},{360,280}})),
    experiment(
      StopTime=30,
      __Dymola_NumberOfIntervals=100000,
      Tolerance=1e-08,
      __Dymola_fixedstepsize=1e-07,
      __Dymola_Algorithm="Dassl"));
end SmallSignalModel;
