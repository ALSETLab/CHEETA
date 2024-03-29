within CHEETA.Examples.CHEETAElectricalSystem;
model Wing4FC "Body power configuration with battery"
  import ElectrifiedPowertrains;
  import Modelica;

  extends Modelica.Icons.Example;

  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{80,16},{100,36}})));
  Modelica.Blocks.Sources.Ramp     ramp(height=733.038, duration=5)
    annotation (Placement(transformation(extent={{116,42},{102,56}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{50,-278},{70,-258}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM1(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{80,-24},{100,-4}})));
  Modelica.Blocks.Sources.Ramp     ramp2(height=733.038, duration=5)
    annotation (Placement(transformation(extent={{116,2},{102,16}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{88,66},{68,86}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{146,66},{126,86}})));
  Modelica.Blocks.Sources.Ramp     ramp3(height=733.038, duration=5)
    annotation (Placement(transformation(extent={{118,-42},{104,-28}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1e-6)
    annotation (Placement(transformation(extent={{116,16},{136,36}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1e-6)
    annotation (Placement(transformation(extent={{116,-24},{136,-4}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{170,16},{150,36}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque2(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{170,-24},{150,-4}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM2(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{80,-68},{100,-48}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1e-6)
    annotation (Placement(transformation(extent={{116,-68},{136,-48}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque3(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{170,-68},{150,-48}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-44,-14},{-24,6}})));
  Modelica.Blocks.Sources.Constant const6(k=353.15)
    annotation (Placement(transformation(extent={{-78,-14},{-58,6}})));
  Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage_Hydrogen
                                                HTS(
    l=10,
    n=20,
    I_c0=3700,
    A=0.1,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm"),
    capacitor(v(start=1230)))
    annotation (Placement(transformation(extent={{22,46},{38,38}})));
  Aircraft.Electrical.BusExt busExt(np=1, nn=1)
    annotation (Placement(transformation(extent={{50,24},{44,-16}})));
  Aircraft.Electrical.FuelCell.LumpedFuelCells lumpedFuelCells
    annotation (Placement(transformation(extent={{-10,32},{10,52}})));
  Modelica.Blocks.Sources.Ramp     ramp1(height=733.038, duration=5)
    annotation (Placement(transformation(extent={{124,-88},{110,-74}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM3(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{86,-114},{106,-94}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia3(J=1e-6)
    annotation (Placement(transformation(extent={{122,-114},{142,-94}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque1(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{176,-114},{156,-94}})));
  Modelica.Blocks.Sources.Ramp     ramp4(height=733.038, duration=5)
    annotation (Placement(transformation(extent={{122,-138},{108,-124}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM4(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{84,-164},{104,-144}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia4(J=1e-6)
    annotation (Placement(transformation(extent={{120,-164},{140,-144}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque4(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{174,-164},{154,-144}})));
  Modelica.Blocks.Sources.Ramp     ramp5(height=733.038, duration=5)
    annotation (Placement(transformation(extent={{126,-186},{112,-172}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM5(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{88,-212},{108,-192}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia5(J=1e-6)
    annotation (Placement(transformation(extent={{124,-212},{144,-192}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque5(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{178,-212},{158,-192}})));
  Modelica.Blocks.Sources.Ramp     ramp6(height=733.038, duration=5)
    annotation (Placement(transformation(extent={{128,-232},{114,-218}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM6(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{90,-258},{110,-238}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia6(J=1e-6)
    annotation (Placement(transformation(extent={{126,-258},{146,-238}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque6(
      tau_constant=-2000)
    annotation (Placement(transformation(extent={{180,-258},{160,-238}})));
equation
  connect(ramp.y, speedFOC_ESM.desiredSpeed)
    annotation (Line(points={{101.3,49},{90,49},{90,38}}, color={0,0,127}));
  connect(speedFOC_ESM.pin_n,ground. p)
    annotation (Line(points={{80,20},{64,20},{64,-20},{60,-20},{60,-258}},
                                                           color={0,0,255}));
  connect(speedFOC_ESM1.pin_n,ground. p)
    annotation (Line(points={{80,-20},{60,-20},{60,-258}}, color={0,0,255}));
  connect(speedFOC_ESM1.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{80,-8},
          {70,-8},{70,32},{80,32}},            color={0,0,255}));
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{125,76},{90,76}},color={0,0,127}));
  connect(inertia.flange_a, speedFOC_ESM.flange)
    annotation (Line(points={{116,26},{100,26}}, color={0,0,0}));
  connect(inertia1.flange_a, speedFOC_ESM1.flange)
    annotation (Line(points={{116,-14},{100,-14}}, color={0,0,0}));
  connect(constantTorque.flange, inertia.flange_b)
    annotation (Line(points={{150,26},{136,26}}, color={0,0,0}));
  connect(inertia1.flange_b, constantTorque2.flange)
    annotation (Line(points={{136,-14},{150,-14}}, color={0,0,0}));
  connect(inertia2.flange_a, speedFOC_ESM2.flange)
    annotation (Line(points={{116,-58},{100,-58}}, color={0,0,0}));
  connect(inertia2.flange_b, constantTorque3.flange)
    annotation (Line(points={{136,-58},{150,-58}}, color={0,0,0}));
  connect(speedFOC_ESM2.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{80,-52},
          {70,-52},{70,32},{80,32}},      color={0,0,255}));
  connect(speedFOC_ESM2.pin_n, ground.p)
    annotation (Line(points={{80,-64},{60,-64},{60,-258}}, color={0,0,255}));
  connect(const6.y,prescribedTemperature4. T)
    annotation (Line(points={{-57,-4},{-46,-4}},
                                               color={0,0,127}));
  connect(prescribedTemperature3.port, HTS.port_a)
    annotation (Line(points={{68,76},{30.2,76},{30.2,46}}, color={191,0,0}));
  connect(ramp3.y, speedFOC_ESM2.desiredSpeed)
    annotation (Line(points={{103.3,-35},{90,-35},{90,-46}}, color={0,0,127}));
  connect(ramp2.y, speedFOC_ESM1.desiredSpeed)
    annotation (Line(points={{101.3,9},{90,9},{90,-2}}, color={0,0,127}));
  connect(busExt.p[1], HTS.pin_n) annotation (Line(points={{44,4},{44,14},{26,
          14},{26,32},{52,32},{52,42},{39,42}}, color={0,0,255}));
  connect(busExt.n[1], speedFOC_ESM.pin_p)
    annotation (Line(points={{50,4},{72,32},{80,32}}, color={0,0,255}));
  connect(HTS.pin_p, lumpedFuelCells.p1)
    annotation (Line(points={{21,42},{10.2,42}}, color={0,0,255}));
  connect(lumpedFuelCells.port_a1, prescribedTemperature4.port) annotation (
      Line(points={{-10.2,42},{-18,42},{-18,-4},{-24,-4}}, color={191,0,0}));
  connect(inertia3.flange_a,speedFOC_ESM3. flange)
    annotation (Line(points={{122,-104},{106,-104}},
                                                   color={0,0,0}));
  connect(inertia3.flange_b,constantTorque1. flange)
    annotation (Line(points={{142,-104},{156,-104}},
                                                   color={0,0,0}));
  connect(ramp1.y, speedFOC_ESM3.desiredSpeed)
    annotation (Line(points={{109.3,-81},{96,-81},{96,-92}}, color={0,0,127}));
  connect(speedFOC_ESM3.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{86,
          -98},{70,-98},{70,32},{80,32}}, color={0,0,255}));
  connect(speedFOC_ESM3.pin_n, ground.p)
    annotation (Line(points={{86,-110},{60,-110},{60,-258}}, color={0,0,255}));
  connect(inertia4.flange_a,speedFOC_ESM4. flange)
    annotation (Line(points={{120,-154},{104,-154}},
                                                   color={0,0,0}));
  connect(inertia4.flange_b,constantTorque4. flange)
    annotation (Line(points={{140,-154},{154,-154}},
                                                   color={0,0,0}));
  connect(ramp4.y, speedFOC_ESM4.desiredSpeed) annotation (Line(points={{107.3,
          -131},{94,-131},{94,-142}}, color={0,0,127}));
  connect(inertia5.flange_a,speedFOC_ESM5. flange)
    annotation (Line(points={{124,-202},{108,-202}},
                                                   color={0,0,0}));
  connect(inertia5.flange_b,constantTorque5. flange)
    annotation (Line(points={{144,-202},{158,-202}},
                                                   color={0,0,0}));
  connect(ramp5.y, speedFOC_ESM5.desiredSpeed) annotation (Line(points={{111.3,
          -179},{98,-179},{98,-190}}, color={0,0,127}));
  connect(inertia6.flange_a,speedFOC_ESM6. flange)
    annotation (Line(points={{126,-248},{110,-248}},
                                                   color={0,0,0}));
  connect(inertia6.flange_b,constantTorque6. flange)
    annotation (Line(points={{146,-248},{160,-248}},
                                                   color={0,0,0}));
  connect(ramp6.y, speedFOC_ESM6.desiredSpeed) annotation (Line(points={{113.3,
          -225},{100,-225},{100,-236}}, color={0,0,127}));
  connect(speedFOC_ESM4.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{84,
          -148},{70,-148},{70,32},{80,32}}, color={0,0,255}));
  connect(speedFOC_ESM4.pin_n, ground.p)
    annotation (Line(points={{84,-160},{60,-160},{60,-258}}, color={0,0,255}));
  connect(speedFOC_ESM5.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{88,
          -196},{70,-196},{70,32},{80,32}}, color={0,0,255}));
  connect(speedFOC_ESM6.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{90,
          -242},{70,-242},{70,32},{80,32}}, color={0,0,255}));
  connect(speedFOC_ESM5.pin_n, ground.p)
    annotation (Line(points={{88,-208},{60,-208},{60,-258}}, color={0,0,255}));
  connect(speedFOC_ESM6.pin_n, ground.p)
    annotation (Line(points={{90,-254},{60,-254},{60,-258}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-80,-280},{180,
            100}})),
    Icon(coordinateSystem(extent={{-80,-280},{180,100}},  preserveAspectRatio=false), graphics),
    experiment(Interval=0.1, __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Wing4FC;
