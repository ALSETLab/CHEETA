within CHEETA.Aircraft.Electrical.PowerElectronics.Examples;
model VSI_2
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
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation (Placement(transformation(extent={{-10,24},
            {-22,36}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage batteryVoltage(V=500)
    annotation (Placement(transformation(extent={{-10,42},{-24,56}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{78,-6},
            {90,6}})));
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
  connect(voltageSensor.p,inverter. pin_p)
    annotation (Line(
      points={{-10,30},{2,30},{2,6},{8,6}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(voltageSensor.n,ground. p)
    annotation (Line(
      points={{-22,30},{-34,30},{-34,20},{-2,20},{-2,-8}},
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
  connect(batteryVoltage.n, ground.p) annotation (Line(points={{-24,49},{-34,49},
          {-34,20},{-2,20},{-2,-8}}, color={0,0,255}));
  connect(batteryVoltage.p, inverter.pin_p)
    annotation (Line(points={{-10,49},{2,49},{2,6},{8,6}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end VSI_2;
