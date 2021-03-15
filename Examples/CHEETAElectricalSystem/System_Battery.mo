within CHEETA.Examples.CHEETAElectricalSystem;
model System_Battery

  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{18,-56},{38,-36}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{26,28},{6,48}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{72,28},{52,48}})));
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
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
    TorqueDirection=false,
    tau_nominal=-1000,
    w_nominal(displayUnit="rpm") = 314.15926535898)
                                          annotation (Placement(transformation(extent={{86,-24},
            {66,-4}})));
  Modelica.Blocks.Sources.Constant const1(k=733.038285)
    annotation (Placement(transformation(extent={{26,12},{40,26}})));
  Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-12,-8},{12,8}},
        rotation=270,
        origin={-6,-28})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression
    annotation (Placement(transformation(extent={{-46,-58},{-26,-38}})));
  Aircraft.Electrical.BusExt busExt(nn=2, np=1)
    annotation (Placement(transformation(extent={{14,10},{16,-10}})));
  Modelica.Blocks.Sources.Ramp     ramp(
    height=-9999.99,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-46,-22},{-26,-2}})));
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
    annotation (Placement(transformation(extent={{-40,16},{-24,8}})));
  Aircraft.Electrical.FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(n=2, R_ohm_current=0.075)
    annotation (Placement(transformation(extent={{-86,2},{-72,16}})));
equation
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{51,38},{28,38}}, color={0,0,127}));
  connect(const4.y,prescribedTemperature4. T)
    annotation (Line(points={{-111,-30},{-100,-30}},
                                               color={0,0,127}));
  connect(speedFOC_ESM.pin_n, ground1.p)
    annotation (Line(points={{38,-20},{28,-20},{28,-36}}, color={0,0,255}));
  connect(speedFOC_ESM.flange, load.flange)
    annotation (Line(points={{58,-14},{66,-14}}, color={0,0,0}));
  connect(const1.y,speedFOC_ESM. desiredSpeed)
    annotation (Line(points={{40.7,19},{48,19},{48,-2}},
                                                       color={0,0,127}));
  connect(battery_FC_Charging.u1, booleanExpression.y) annotation (Line(points={{
          -5.27273,-37.1765},{-5.27273,-48},{-25,-48}},   color={255,0,255}));
  connect(busExt.p[1], speedFOC_ESM.pin_p)
    annotation (Line(points={{16,0},{32,0},{32,-8},{38,-8}}, color={0,0,255}));
  connect(battery_FC_Charging.p1, busExt.n[1]) annotation (Line(points={{
          0.545455,-23.7647},{6,-23.7647},{6,3},{14,3}},   color={0,0,255}));
  connect(battery_FC_Charging.n1, ground1.p) annotation (Line(points={{0.545455,
          -30.8235},{28,-30.8235},{28,-36}}, color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1,HTS. pin_p) annotation (Line(
        points={{-72,9.7},{-58,9.7},{-58,12},{-41,12}}, color={0,0,255}));
  connect(HTS.pin_n, busExt.n[2])
    annotation (Line(points={{-23,12},{14,12},{14,-3}}, color={0,0,255}));
  connect(prescribedTemperature3.port, HTS.port_a) annotation (Line(points={{6,
          38},{-30,38},{-30,20},{-31.8,20},{-31.8,16}}, color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a, prescribedTemperature4.port)
    annotation (Line(points={{-79,2},{-76,2},{-76,-12},{-62,-12},{-62,-30},{-78,
          -30},{-78,-30}}, color={191,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-140,-60},{100,60}})),  Icon(
        coordinateSystem(extent={{-140,-60},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end System_Battery;
