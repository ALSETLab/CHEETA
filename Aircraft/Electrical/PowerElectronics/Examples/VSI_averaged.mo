within CHEETA.Aircraft.Electrical.PowerElectronics.Examples;
model VSI_averaged
              ElectrifiedPowertrains.PowerElectronics.Inverters.Switched.Ideal
                                                         inverter(redeclare
      CHEETA.Aircraft.Electrical.PowerElectronics.Examples.Records.VSI_PowerElectronics
                                   data)
    annotation (Placement(transformation(extent={{-6,30},{14,50}})));
  ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Linear machine(
      redeclare CHEETA.Aircraft.Electrical.PowerElectronics.Examples.Records.VSI_20kW
                                 data)
    annotation (Placement(transformation(extent={{34,30},{54,50}})));
  ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Torque controller(
      redeclare
      ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Records.Base.Torque
      data(redeclare
        CHEETA.Aircraft.Electrical.PowerElectronics.Examples.Records.VSI_20kW
        machineData))
    annotation (Placement(transformation(extent={{-86,30},{-66,50}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.SpaceVectorModulation
                                                             modulationMethod(fs=5e3, deadTime=
       1e-6)
    annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-10,-28})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(
    J=0.1,
    phi(fixed=true, start=0),
    w(start=0))
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={54,-20})));
  Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque load(w_nominal(
        displayUnit="rpm") = 104.71975511966, tau_nominal=-26)
                       annotation (Placement(transformation(extent={{14,-30},{
            34,-10}})));
  Modelica.Blocks.Sources.Constant desiredTorque(k=16)
                                                      annotation (Placement(transformation(extent={{-110,36},
            {-102,44}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{64,34},
            {76,46}})));
  Modelica.Electrical.Polyphase.Basic.Star star annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-56})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-70,-98},{-50,-78}})));
  parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
                                                              smpmData(
    fsNominal=60,
    Rs=0.011176,
    Lmd=4*164.571e-6,
    Lmq=4*125.772e-6,
    VsOpenCircuit=400)
    annotation (Placement(transformation(extent={{-76,58},{-56,78}})));
  ElectrifiedPowertrains.PowerElectronics.Rectifiers.Averaged.IdealThreePhase
                                                              threePhaseRectifier(
      enableSmoothing=true)
    annotation (Placement(transformation(extent={{-46,-20},{-26,0}})));
  Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(V=fill(400*sqrt(
        2), 3), f=fill(60, 3)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-30})));
  Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet
    smpm1(
    p=smpmData.p,
    fsNominal=smpmData.fsNominal,
    Rs=smpmData.Rs,
    TsRef=smpmData.TsRef,
    Lszero=smpmData.Lszero,
    Lssigma=smpmData.Lssigma,
    Jr=smpmData.Jr,
    Js=smpmData.Js,
    frictionParameters=smpmData.frictionParameters,
    wMechanical(start=10.471975511966, fixed=true),
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
    alpha20r=smpmData.alpha20r)
    annotation (Placement(transformation(extent={{-108,-48},{-128,-28}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-108,-32},{-128,-12}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-96,-50},{-76,-30}})));
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=2*
        3.14*smpmData.fsNominal/smpmData.p)
    annotation (Placement(transformation(extent={{-164,-48},{-144,-28}})));
  Modelica.Electrical.Polyphase.Basic.Resistor resistor(R={20,20,20})
    annotation (Placement(transformation(extent={{-96,-28},{-76,-8}})));
equation
  connect(machine.plug_p,inverter. plug)
    annotation (Line(
      points={{34,40},{14,40}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(controller.electricDriveBus,machine. electricDriveBus) annotation (Line(
      points={{-76,30},{-76,20},{44,20},{44,30}},
      color={0,86,166},
      thickness=0.5,
      smooth=Smooth.None));
  connect(modulationMethod.electricDriveBus,machine. electricDriveBus) annotation (Line(
      points={{-36,30},{-36,20},{44,20},{44,30}},
      color={0,86,166},
      thickness=0.5,
      smooth=Smooth.None));
  connect(inverter.electricDriveBus,machine. electricDriveBus) annotation (Line(
      points={{4,30},{4,20},{44,20},{44,30}},
      color={0,86,166},
      thickness=0.5,
      smooth=Smooth.None));
  connect(ground.p,inverter. pin_n) annotation (Line(
      points={{-10,-24},{-10,34},{-6,34}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(desiredTorque.y,controller. desiredTorque)
    annotation (Line(
      points={{-101.6,40},{-88,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(inverter.gateSignals,modulationMethod. gateSignals)
    annotation (Line(
      points={{-8,40},{-25,40}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(controller.actuatingVoltages,modulationMethod. phaseVoltages)
    annotation (Line(points={{-65,40},{-48,40}},
                                               color={0,0,127}));
  connect(multiSensor.flange_a,machine. flange) annotation (Line(points={{64,40},
          {54,40}},                                                                             color={0,0,0}));
  connect(multiSensor.flange_b,inertia. flange_a) annotation (Line(points={{76,40},
          {82,40},{82,-20},{64,-20}},                                                                        color={0,0,0}));
  connect(inertia.flange_b, load.flange)
    annotation (Line(points={{44,-20},{34,-20}}, color={0,0,0}));
  connect(star.pin_n, ground1.p)
    annotation (Line(points={{-60,-66},{-60,-78}},   color={0,0,255}));
  connect(star.plug_p, sineVoltage.plug_n)
    annotation (Line(points={{-60,-46},{-60,-40}}, color={0,0,255}));
  connect(threePhaseRectifier.plug, sineVoltage.plug_p)
    annotation (Line(points={{-46,-10},{-60,-10},{-60,-20}}, color={0,0,255}));
  connect(inverter.pin_p, threePhaseRectifier.p) annotation (Line(points={{-6,
          46},{-20,46},{-20,-2},{-26,-2}}, color={0,0,255}));
  connect(inverter.pin_n, threePhaseRectifier.n) annotation (Line(points={{-6,
          34},{-10,34},{-10,-18},{-26,-18}}, color={0,0,255}));
  connect(terminalBox.plug_sp,smpm1. plug_sp)
    annotation (Line(points={{-124,-28},{-124,-28}},
                                                   color={0,0,255}));
  connect(terminalBox.plug_sn,smpm1. plug_sn)
    annotation (Line(points={{-112,-28},{-112,-28}},
                                                 color={0,0,255}));
  connect(terminalBox.starpoint,ground2. p)
    annotation (Line(points={{-108,-26},{-86,-26},{-86,-30}},
                                                          color={0,0,255}));
  connect(constantSpeed.flange,smpm1. flange)
    annotation (Line(points={{-144,-38},{-128,-38}},
                                                   color={0,0,0}));
  connect(resistor.plug_p, terminalBox.plugSupply) annotation (Line(points={{
          -96,-18},{-118,-18},{-118,-26}}, color={0,0,255}));
  connect(resistor.plug_n, sineVoltage.plug_p) annotation (Line(points={{-76,
          -18},{-68,-18},{-68,-10},{-60,-10},{-60,-20}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,
            -100},{100,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{100,
            100}})));
end VSI_averaged;
