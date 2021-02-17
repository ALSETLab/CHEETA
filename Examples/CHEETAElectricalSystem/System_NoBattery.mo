within CHEETA.Examples.CHEETAElectricalSystem;
model System_NoBattery

  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{18,-56},{38,-36}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{26,28},{6,48}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{72,28},{52,48}})));
  CHEETA.Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(n=2, R_ohm_current=0.09)
    annotation (Placement(transformation(extent={{-72,-8},{-58,6}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-98,-40},{-78,-20}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    speedFOC_ESM(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
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
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data)) annotation (Placement(transformation(extent={{38,-24},{58,-4}})));
  Modelica.Blocks.Sources.Constant const4(k=353.15)
    annotation (Placement(transformation(extent={{-132,-40},{-112,-20}})));
  Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage_Hydrogen HTS(
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
    annotation (Placement(transformation(extent={{-28,24},{-12,16}})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
    TorqueDirection=false,
    tau_nominal=-1000,
    w_nominal(displayUnit="rpm") = 314.15926535898)
                                          annotation (Placement(transformation(extent={{86,-24},
            {66,-4}})));
  Modelica.Blocks.Sources.Constant const1(k=733.038285)
    annotation (Placement(transformation(extent={{4,0},{24,20}})));
equation
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{51,38},{28,38}}, color={0,0,127}));
  connect(const4.y,prescribedTemperature4. T)
    annotation (Line(points={{-111,-30},{-100,-30}},
                                               color={0,0,127}));
  connect(prescribedTemperature4.port, fuelCell_EquationBased_DetailedRohm1.port_a)
    annotation (Line(points={{-78,-30},{-65,-30},{-65,-8}},     color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1, HTS.pin_p) annotation (Line(
        points={{-58,-0.3},{-58,0},{-40,0},{-40,20},{-29,20}}, color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature3.port)
    annotation (Line(points={{-19.8,24},{-19.8,38},{6,38}}, color={191,0,0}));
  connect(speedFOC_ESM.pin_p, HTS.pin_n) annotation (Line(points={{38,-8},{-4,
          -8},{-4,20},{-11,20}}, color={0,0,255}));
  connect(speedFOC_ESM.pin_n, ground1.p)
    annotation (Line(points={{38,-20},{28,-20},{28,-36}}, color={0,0,255}));
  connect(speedFOC_ESM.flange, load.flange)
    annotation (Line(points={{58,-14},{66,-14}}, color={0,0,0}));
  connect(const1.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{25,10},{48,10},{48,-2}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-140,-60},{100,60}})),  Icon(
        coordinateSystem(extent={{-140,-60},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end System_NoBattery;
