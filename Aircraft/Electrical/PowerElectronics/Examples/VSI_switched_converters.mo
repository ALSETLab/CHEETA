within CHEETA.Aircraft.Electrical.PowerElectronics.Examples;
model VSI_switched_converters
  "VSI model using a switched rectifier converter so it is very slow"
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
    annotation (Placement(transformation(extent={{-84,24},{-104,44}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.Switched.Ideal
                                                         inverter(redeclare
      Records.VSI_PowerElectronics data)
    annotation (Placement(transformation(extent={{8,-10},{28,10}})));
  ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Linear machine(
      redeclare Records.VSI_20kW data)
    annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Torque controller(
      redeclare
      ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Records.Base.Torque
      data(redeclare
        CHEETA.Aircraft.Electrical.PowerElectronics.Examples.Records.VSI_20kW
        machineData))
    annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.SpaceVectorModulation
                                                             modulationMethod(fs=5e3, deadTime=
       1e-6)
    annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-2,-12})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(
    J=0.1,
    phi(fixed=true, start=0),
    w(start=0))
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={68,-60})));
  Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque load(w_nominal(
        displayUnit="rpm") = 104.71975511966, tau_nominal=-26)
                       annotation (Placement(transformation(extent={{16,-70},{
            36,-50}})));
  Modelica.Blocks.Sources.Constant desiredTorque(k=16)
                                                      annotation (Placement(transformation(extent={{-96,-4},
            {-88,4}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{78,-6},
            {90,6}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-84,40},{-104,60}})));
  Modelica.Electrical.Polyphase.Basic.Star star annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-132,-16})));
  Modelica.Electrical.Polyphase.Basic.Resistor resistor(R={20,20,20})
    annotation (Placement(transformation(extent={{-130,52},{-110,72}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-142,-54},{-122,-34}})));
  parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
                                                              smpmData(
    fsNominal=60,
    Rs=0.011176,
    Lmd=4*164.571e-6,
    Lmq=4*125.772e-6,
    VsOpenCircuit=400)
    annotation (Placement(transformation(extent={{-76,58},{-56,78}})));
  Modelica.Electrical.PowerConverters.ACDC.DiodeBridge2mPulse rectifier
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(v(start=1, fixed=true),
      C=1e-6)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-24,-46})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-72,22},{-52,42}})));
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=2*
        3.14*smpmData.fsNominal/smpmData.p)
    annotation (Placement(transformation(extent={{-140,24},{-120,44}})));
  Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(V=fill(400*sqrt(
        2), 3), f=fill(60, 3)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-102,-50})));
equation
  connect(machine.plug_p,inverter. plug)
    annotation (Line(
      points={{48,0},{28,0}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(controller.electricDriveBus,machine. electricDriveBus) annotation (Line(
      points={{-62,-10},{-62,-20},{58,-20},{58,-10}},
      color={0,86,166},
      thickness=0.5,
      smooth=Smooth.None));
  connect(modulationMethod.electricDriveBus,machine. electricDriveBus) annotation (Line(
      points={{-22,-10},{-22,-20},{58,-20},{58,-10}},
      color={0,86,166},
      thickness=0.5,
      smooth=Smooth.None));
  connect(inverter.electricDriveBus,machine. electricDriveBus) annotation (Line(
      points={{18,-10},{18,-20},{58,-20},{58,-10}},
      color={0,86,166},
      thickness=0.5,
      smooth=Smooth.None));
  connect(ground.p,inverter. pin_n) annotation (Line(
      points={{-2,-8},{4,-8},{4,-6},{8,-6}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(desiredTorque.y,controller. desiredTorque)
    annotation (Line(
      points={{-87.6,0},{-74,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(inverter.gateSignals,modulationMethod. gateSignals)
    annotation (Line(
      points={{6,0},{-11,0}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(controller.actuatingVoltages,modulationMethod. phaseVoltages)
    annotation (Line(points={{-51,0},{-34,0}}, color={0,0,127}));
  connect(multiSensor.flange_a,machine. flange) annotation (Line(points={{78,0},{
          68,0}},                                                                               color={0,0,0}));
  connect(multiSensor.flange_b,inertia. flange_a) annotation (Line(points={{90,0},{
          96,0},{96,-60},{78,-60}},                                                                          color={0,0,0}));
  connect(inertia.flange_b, load.flange)
    annotation (Line(points={{58,-60},{36,-60}}, color={0,0,0}));
  connect(star.plug_p, resistor.plug_p)
    annotation (Line(points={{-132,-6},{-132,62},{-130,62}}, color={0,0,255}));
  connect(resistor.plug_n, terminalBox.plugSupply) annotation (Line(points={{-110,62},
          {-94,62},{-94,46}},                                       color={0,0,
          255}));
  connect(star.pin_n, ground1.p)
    annotation (Line(points={{-132,-26},{-132,-34}}, color={0,0,255}));
  connect(terminalBox.plug_sp, smpm1.plug_sp)
    annotation (Line(points={{-100,44},{-100,44}}, color={0,0,255}));
  connect(terminalBox.plug_sn, smpm1.plug_sn)
    annotation (Line(points={{-88,44},{-88,44}}, color={0,0,255}));
  connect(inverter.pin_n, rectifier.dc_n) annotation (Line(points={{8,-6},{4,-6},
          {4,-56},{-60,-56}}, color={0,0,255}));
  connect(capacitor.p, rectifier.dc_p)
    annotation (Line(points={{-24,-36},{-54,-36},{-54,-44},{-60,-44}},
                                                   color={0,0,255}));
  connect(terminalBox.starpoint, ground2.p)
    annotation (Line(points={{-84,46},{-62,46},{-62,42}}, color={0,0,255}));
  connect(constantSpeed.flange, smpm1.flange)
    annotation (Line(points={{-120,34},{-104,34}}, color={0,0,0}));
  connect(inverter.pin_n, capacitor.n) annotation (Line(points={{8,-6},{4,-6},{
          4,-56},{-24,-56}}, color={0,0,255}));
  connect(capacitor.p, inverter.pin_p) annotation (Line(points={{-24,-36},{-8,
          -36},{-8,6},{8,6}}, color={0,0,255}));
  connect(rectifier.ac, sineVoltage.plug_p)
    annotation (Line(points={{-80,-50},{-92,-50}}, color={0,0,255}));
  connect(sineVoltage.plug_n, resistor.plug_p) annotation (Line(points={{-112,
          -50},{-122,-50},{-122,0},{-132,0},{-132,62},{-130,62}}, color={0,0,
          255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,
            -100},{100,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{100,
            100}})));
end VSI_switched_converters;
