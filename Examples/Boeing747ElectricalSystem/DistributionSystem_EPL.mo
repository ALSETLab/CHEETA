within CHEETA.Examples.Boeing747ElectricalSystem;
model DistributionSystem_EPL
  Modelica.Blocks.Sources.CombiTimeTable
                                    combiTimeTable(table=[0,0,0,0; 0.1,11900,
        11900,11900; 0.5,12000,12000,12000; 1,12000,12000,12000; 2,12000,12000,
        12000; 3,10000,10000,10000; 4.5,18000,18000,18000; 6,1,1,1; 6,0,0,0; 10,
        0,0,0],
              timeScale=3600)
                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-64,2})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-26,2})));
  Modelica.Blocks.Sources.Ramp maxLineVoltage(
    duration(displayUnit="s") = 1,
    offset=machine.data.u_s_nom,
    height=0,
    startTime(displayUnit="s") = 1.5) "maximum line to line voltage (rms)"
    annotation (Placement(transformation(extent={{-96,-54},{-76,-34}})));
  ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.MaxLineVoltage toBus_maxStatorVoltage
    annotation (Placement(transformation(extent={{-68,-50},{-56,-38}})));
  replaceable Aircraft.Electrical.Machines.Examples.Boeing747.SynGenwAVR generation(
      useDamperCage=false) constrainedby
    Aircraft.Electrical.Machines.Examples.Boeing747.SynGenwAVR
                                                     annotation (Placement(
        transformation(
        extent={{11,-7},{-11,7}},
        rotation=0,
        origin={21,47})));
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.1,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-64,40})));
  Modelica.Blocks.Math.Gain RPMtoRPS1(k=3.14/30)
                                                annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-26,40})));
  replaceable Aircraft.Electrical.Loads.Boeing747.Fuel_Pump PUMP(
    N=200/(sqrt(2)*28),
    V_rated=28,
    L=-1.5/200,
    P_fixed=0.0001) constrainedby Aircraft.Electrical.Loads.Boeing747.Fuel_Pump
    annotation (Placement(transformation(extent={{62,52},{82,72}})));
  ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed controller(
      redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Records.Base.Speed
      data(
      redeclare
        CHEETA.Aircraft.Electrical.Machines.Records.Boeing.Boeing747_ESM
        machineData,
      Ve_max=1000,
      i_s_max=1000,
      Ie_max=10,
      tau_max=1000))
    annotation (Placement(transformation(extent={{-4,-8},{16,12}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Models.IdealESM_Supply idealESM_Supply
    annotation (Placement(transformation(extent={{26,-8},{46,12}})));
  ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear machine(
      redeclare Aircraft.Electrical.Machines.Records.Boeing.Boeing747_ESM data)
    annotation (Placement(transformation(extent={{56,-8},{76,12}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.EnergyAnalyser machineAnalyser(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{98,-70},{78,-50}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.ElectricQuantities signals(
      terminalConnection=machine.data.terminalConnection)
    annotation (Placement(transformation(extent={{98,-38},{78,-18}})));
  replaceable Aircraft.Electrical.Loads.Boeing747.Fuel_Pump PUMP1(
    N=200/(sqrt(2)*28),
    V_rated=28,
    L=-1.5/200,
    P_fixed=0.0001) constrainedby Aircraft.Electrical.Loads.Boeing747.Fuel_Pump
    annotation (Placement(transformation(extent={{62,20},{82,40}})));
equation
  connect(RPMtoRPS.u, combiTimeTable.y[3])
    annotation (Line(points={{-38,2},{-53,2}}, color={0,0,127}));
  connect(maxLineVoltage.y,toBus_maxStatorVoltage. u)
    annotation (Line(
      points={{-75,-44},{-69.2,-44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(generation.w_ref, RPMtoRPS1.y) annotation (Line(points={{9.3125,47},{
          -8,47},{-8,40},{-15,40}}, color={0,0,127}));
  connect(timeTable.y, RPMtoRPS1.u)
    annotation (Line(points={{-53,40},{-38,40}}, color={0,0,127}));
  connect(generation.AC_out, PUMP.AC_in) annotation (Line(points={{32.1375,
          47.14},{46,47.14},{46,62},{62,62}}, color={0,0,255}));
  connect(idealESM_Supply.statorVoltages, controller.actuatingStatorVoltages)
    annotation (Line(points={{24,8},{17,8}}, color={0,0,127}));
  connect(idealESM_Supply.excitationVoltage, controller.actuatingExcitationVoltage)
    annotation (Line(points={{24,-4},{20,-4},{20,-4},{17,-4}}, color={0,0,127}));
  connect(controller.desiredSpeed, RPMtoRPS.y)
    annotation (Line(points={{-6,2},{-15,2}}, color={0,0,127}));
  connect(machine.plug_p, idealESM_Supply.plug_p1)
    annotation (Line(points={{56,2},{46,2}}, color={0,0,255}));
  connect(idealESM_Supply.p, machine.pin_ep)
    annotation (Line(points={{45.8,-2},{56,-2}}, color={0,0,255}));
  connect(idealESM_Supply.n, machine.pin_en)
    annotation (Line(points={{46,-6},{56,-6}}, color={0,0,255}));
  connect(toBus_maxStatorVoltage.electricDriveBus, controller.electricDriveBus)
    annotation (Line(
      points={{-56,-44},{6,-44},{6,-8}},
      color={0,86,166},
      thickness=0.5));
  connect(machine.electricDriveBus, controller.electricDriveBus) annotation (
      Line(
      points={{66,-8},{66,-44},{6,-44},{6,-8}},
      color={0,86,166},
      thickness=0.5));
  connect(signals.electricDriveBus, controller.electricDriveBus) annotation (
      Line(
      points={{88,-38},{88,-44},{6,-44},{6,-8}},
      color={0,86,166},
      thickness=0.5));
  connect(machineAnalyser.electricDriveBus, controller.electricDriveBus)
    annotation (Line(
      points={{88,-70},{88,-74},{66,-74},{66,-44},{6,-44},{6,-8}},
      color={0,86,166},
      thickness=0.5));
  connect(PUMP1.AC_in, idealESM_Supply.plug_p1)
    annotation (Line(points={{62,30},{52,30},{52,2},{46,2}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -200},{100,80}})),       Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-200},{100,80}})),
    experiment(
      StopTime=10,
      Interval=0.001,
      Tolerance=0.001));
end DistributionSystem_EPL;
