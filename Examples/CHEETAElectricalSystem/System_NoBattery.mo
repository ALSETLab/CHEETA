within CHEETA.Examples.CHEETAElectricalSystem;
model System_NoBattery

  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-4,-62},{16,-42}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{6,34},{-14,54}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{68,34},{48,54}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-106,-30},{-86,-10}})));
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
        data)) annotation (Placement(transformation(extent={{30,-14},{50,6}})));
  Modelica.Blocks.Sources.Constant const1(k=353.15)
    annotation (Placement(transformation(extent={{-140,-30},{-120,-10}})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
    TorqueDirection=false,
    tau_nominal=-1000,
    w_nominal(displayUnit="rpm") = 314.15926535898)
                                          annotation (Placement(transformation(extent={{78,-14},
            {58,6}})));
  Modelica.Blocks.Sources.Constant const2(k=733.038285)
    annotation (Placement(transformation(extent={{18,22},{32,36}})));
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
    b(displayUnit="mm"))
    annotation (Placement(transformation(extent={{-48,26},{-32,18}})));
  Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(n=2, R_ohm_current=0.075)
    annotation (Placement(transformation(extent={{-94,12},{-80,26}})));
equation
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{47,44},{8,44}},  color={0,0,127}));
  connect(const1.y,prescribedTemperature4. T)
    annotation (Line(points={{-119,-20},{-108,-20}},
                                               color={0,0,127}));
  connect(speedFOC_ESM.pin_n,ground1. p)
    annotation (Line(points={{30,-10},{6,-10},{6,-42}},   color={0,0,255}));
  connect(speedFOC_ESM.flange,load. flange)
    annotation (Line(points={{50,-4},{58,-4}},   color={0,0,0}));
  connect(const2.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{32.7,29},{40,29},{40,8}},color={0,0,127}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1,HTS. pin_p) annotation (Line(
        points={{-80,19.7},{-66,19.7},{-66,22},{-49,22}},
                                                        color={0,0,255}));
  connect(prescribedTemperature3.port, HTS.port_a) annotation (Line(points={{
          -14,44},{-40,44},{-40,30},{-39.8,30},{-39.8,26}}, color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a, prescribedTemperature4.port)
    annotation (Line(points={{-87,12},{-84,12},{-84,-2},{-70,-2},{-70,-20},{-86,
          -20}}, color={191,0,0}));
  connect(speedFOC_ESM.pin_p, HTS.pin_n) annotation (Line(points={{30,2},{-24,2},
          {-24,22},{-31,22}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-140,-60},{100,60}})),  Icon(
        coordinateSystem(extent={{-140,-60},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end System_NoBattery;
