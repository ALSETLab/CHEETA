within CHEETA.Aircraft.Electrical.Loads.Boeing747;
model PMSM

  parameter Real L = -0.2 "Slope of Linear Speed to Torque Load";
  extends CHEETA.Aircraft.Electrical.Loads.Boeing747.DC_Load(
      simulink_Averaged_Rectifier(P_fixed=200, V_rated=1),           yd(N=N));
  Modelica.Electrical.MultiPhase.Basic.Star star2 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-38,-52})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-48,-86},{-28,-66}})));
  Modelica.Electrical.MultiPhase.Basic.Capacitor capacitor2(C={1e-6,1e-6,1e-6})
               annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-38,-22})));
  PowerElectronics.Converters.DCAC.Simulink_AvgInverter_grounded
                                                             avgInverter
    annotation (Placement(transformation(extent={{196,-44},{222,-12}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={312,30})));
  Modelica.Electrical.MultiPhase.Sensors.CurrentSensor currentSensor(m=3)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={258,0})));
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
    ir(fixed=false),
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
          extent={{292,-44},{312,-24}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{292,-28},{312,-8}})));
  CHEETA.Aircraft.Controls.SpeedDrives.SpeedController
                                                    speedController(
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
        extent={{-23,-11},{23,11}},
        rotation=0,
        origin={89,101})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{26,92},{46,112}})));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={58,24})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    linearSpeedDependentTorque(tau_nominal=L,    w_nominal=1)
    annotation (Placement(transformation(extent={{370,-44},{350,-24}})));
  replaceable parameter Records.Boeing747electricalModel.PMSM.PMSM_2kVA
                                                                   smpmData
    constrainedby CHEETA.Records.Boeing747electricalModel.PMSM.PSM
    annotation (Placement(transformation(extent={{252,78},{272,98}})));
  Modelica.Blocks.Interfaces.RealInput Ref annotation (Placement(transformation(
        origin={-100,120},
        extent={{20,-20},{-20,20}},
        rotation=180), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-80,82})));
equation
  connect(ground1.p,star2. pin_n)
    annotation (Line(points={{-38,-66},{-38,-62}},
                                                 color={0,0,255}));
  connect(capacitor2.plug_n,star2. plug_p) annotation (Line(points={{-38,-32},{
          -38,-42}},                   color={0,0,255}));
  connect(simulink_Averaged_Rectifier.AC, yd.Secondary1) annotation (Line(points={{-27,0},
          {-30,0},{-30,0},{-45.6,0}},  color={0,0,255}));
  connect(capacitor2.plug_p, yd.Secondary1)
    annotation (Line(points={{-38,-12},{-38,0},{-45.6,0}}, color={0,0,255}));
  connect(terminalBox.plug_sn,smpm. plug_sn) annotation (Line(
      points={{296,-24},{296,-24}},
      color={0,0,255}));
  connect(terminalBox.plug_sp,smpm. plug_sp) annotation (Line(
      points={{308,-24},{308,-24}},
      color={0,0,255}));
  connect(smpm.flange,angleSensor. flange) annotation (Line(
      points={{312,-34},{312,20}}));
  connect(currentSensor.plug_n,terminalBox. plugSupply) annotation (Line(
      points={{258,10},{258,20},{302,20},{302,-22}},
      color={0,0,255}));
  connect(speedController.a,avgInverter. ma) annotation (Line(points={{112.958,
          110.8},{112.958,110.454},{200,110.454},{200,-6.76364}}, color={0,0,127}));
  connect(speedController.b,avgInverter. mb) annotation (Line(points={{112.958,
          102},{112.958,102.273},{209.2,102.273},{209.2,-6.76364}},
                                                                color={0,0,127}));
  connect(speedController.c,avgInverter. mc) annotation (Line(points={{112.958,
          91},{112.958,91.273},{218,91.273},{218,-6.76364}},      color={0,0,127}));
  connect(currentSensor.i,speedController. iActual) annotation (Line(points={{247,
          2.22045e-15},{77.6917,2.22045e-15},{77.6917,92}},
                                                     color={0,0,127}));
  connect(currentSensor.plug_p,avgInverter. positivePlug) annotation (Line(
        points={{258,-10},{258,-24.2182},{223,-24.2182}},       color={0,0,
          255}));
  connect(speedSensor.w,speedController. Speed) annotation (Line(points={{47,102},
          {67.9167,102}},                                color={0,0,127}));
  connect(linearSpeedDependentTorque.flange,smpm. flange) annotation (Line(
        points={{350,-34},{312,-34}},
        color={0,0,0}));
  connect(speedSensor.flange, smpm.flange) annotation (Line(points={{26,102},{
          26,-54},{340,-54},{340,-34},{312,-34}},
        color={0,0,0}));
  connect(voltageSensor.p, simulink_Averaged_Rectifier.DC_p) annotation (Line(points={{58,34},
          {44,34},{44,6},{-9,6}},         color={0,0,255}));
  connect(voltageSensor.n, simulink_Averaged_Rectifier.v_ref) annotation (Line(points={{58,14},
          {58,0},{-9,0}},                   color={0,0,255}));
  connect(avgInverter.pin_p, simulink_Averaged_Rectifier.DC_p) annotation (Line(points={{195,
          -14.6182},{44,-14.6182},{44,6},{-9,6}},
                                              color={0,0,255}));
  connect(speedController.Ref, Ref) annotation (Line(points={{67.9167,109.2},{
          55.0415,109.2},{55.0415,120},{-100,120}}, color={0,0,127}));
  connect(voltageSensor.v, speedController.Vdc1) annotation (Line(points={{69,
          24},{99.925,24},{99.925,92}}, color={0,0,127}));
  connect(speedController.phi, speedController.Vdc1) annotation (Line(points={{89.1917,
          92},{89.1917,24},{99.925,24},{99.925,92}},         color={0,0,127}));
  connect(avgInverter.pin_n1, simulink_Averaged_Rectifier.DC_n) annotation (
      Line(points={{195,-40.5091},{93.5,-40.5091},{93.5,-6},{-9,-6}}, color={0,
          0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{380,140}})), Icon(
        coordinateSystem(extent={{-100,-100},{380,140}})),
    Documentation(info="<html>
<p>need fix</p>
</html>"));
end PMSM;
