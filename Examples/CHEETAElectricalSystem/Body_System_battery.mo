within CHEETA.Examples.CHEETAElectricalSystem;
model Body_System_battery "Body power configuration with battery"
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
  Modelica.Blocks.Sources.Constant const2(k=733.038285)
    annotation (Placement(transformation(extent={{116,42},{102,56}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{56,-92},{76,-72}})));
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
  Modelica.Blocks.Sources.Constant const1(k=733.038285)
    annotation (Placement(transformation(extent={{116,2},{102,16}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{88,66},{68,86}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{146,66},{126,86}})));
  Modelica.Blocks.Sources.Constant const4(k=733.038285)
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
  Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(
    n=10,
    R_ohm_current=0.0075,
    C_dl(v(start=0, fixed=true)))
    annotation (Placement(transformation(extent={{-6,34},{8,48}})));
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
equation
  connect(const2.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{101.3,49},{90,49},{90,38}},
                                                       color={0,0,127}));
  connect(speedFOC_ESM.pin_n,ground. p)
    annotation (Line(points={{80,20},{66,20},{66,-72}},    color={0,0,255}));
  connect(speedFOC_ESM1.pin_n,ground. p)
    annotation (Line(points={{80,-20},{66,-20},{66,-72}},  color={0,0,255}));
  connect(speedFOC_ESM1.pin_p,speedFOC_ESM. pin_p) annotation (Line(points={{80,-8},
          {72,-8},{72,32},{80,32}},            color={0,0,255}));
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
  connect(speedFOC_ESM2.pin_p, speedFOC_ESM.pin_p) annotation (Line(points={{80,
          -52},{72,-52},{72,32},{80,32}}, color={0,0,255}));
  connect(speedFOC_ESM2.pin_n, ground.p)
    annotation (Line(points={{80,-64},{66,-64},{66,-72}},  color={0,0,255}));
  connect(const6.y,prescribedTemperature4. T)
    annotation (Line(points={{-57,-4},{-46,-4}},
                                               color={0,0,127}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a,prescribedTemperature4. port)
    annotation (Line(points={{1,34},{2,34},{2,-4},{-24,-4}},
                           color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1, HTS.pin_p) annotation (Line(
        points={{8,41.7},{14.5,41.7},{14.5,42},{21,42}}, color={0,0,255}));
  connect(prescribedTemperature3.port, HTS.port_a)
    annotation (Line(points={{68,76},{30.2,76},{30.2,46}}, color={191,0,0}));
  connect(const4.y, speedFOC_ESM2.desiredSpeed)
    annotation (Line(points={{103.3,-35},{90,-35},{90,-46}}, color={0,0,127}));
  connect(const1.y, speedFOC_ESM1.desiredSpeed)
    annotation (Line(points={{101.3,9},{90,9},{90,-2}}, color={0,0,127}));
  connect(busExt.p[1], HTS.pin_n) annotation (Line(points={{44,4},{44,14},{26,
          14},{26,32},{52,32},{52,42},{39,42}}, color={0,0,255}));
  connect(busExt.n[1], speedFOC_ESM.pin_p)
    annotation (Line(points={{50,4},{72,32},{80,32}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-80,-140},{180,
            100}})),
    Icon(coordinateSystem(extent={{-80,-140},{180,100}},  preserveAspectRatio=false), graphics),
    experiment(Interval=0.1, __Dymola_Algorithm="Dassl"),
    __Dymola_experimentSetupOutput,
    Documentation(info="<html>
<p>The example shows an induction machine with a forced air-based cooling system driven by an external fan.</p>
</html>"),
    __Dymola_Commands(file="modelica://ElectrifiedPowertrains/Resources/Scripts/plot/Example_ForcedCoolingAIM.mos" "plot"));
end Body_System_battery;
