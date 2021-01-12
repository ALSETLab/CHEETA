within CHEETA.Aircraft.Electrical.Battery.Examples;
model Battery_Motor "Battery powering the CHEETA powertrain system"
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                              coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Water_10degC
                                                annotation (Placement(transformation(extent={{110,40},
            {122,52}})));
  Battery_FC_Charging                             battery_FC_Charging(
      batteryPack(N_serialCells=5500, N_parallelCells=10))
    annotation (Placement(transformation(
        extent={{-16.5,-11.5},{16.5,11.5}},
        rotation=270,
        origin={-70.5,1.5})));
  Modelica.Electrical.Analog.Basic.Ground ground9
    annotation (Placement(transformation(extent={{-68,-28},{-48,-8}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-96,-34},{-76,-14}})));
  ElectrifiedPowertrains.ElectricMachines.Common.Blocks.EnergyAnalyser machineAnalyser(
      useBusConnector=true)
    annotation (Placement(transformation(extent={{56,-42},{36,-22}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{80,-4},
            {92,8}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{78,-40},{98,-20}})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque load(
      tau_nominal=-7000, w_nominal(displayUnit="rpm") = 733.03828583762)
                                          annotation (Placement(transformation(extent={{128,-40},
            {108,-20}})));
  FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(
    E_0=1550,
    n=2,
    R_ohm_current=0.4425)
    annotation (Placement(transformation(extent={{-48,44},{-34,58}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-74,10},{-54,30}})));
  CHEETA.Aircraft.Electrical.HTS.LiquidCooled.HTS_filmboiling_Voltage_Hydrogen
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
    annotation (Placement(transformation(extent={{-4,74},{12,66}})));
equation
  connect(battery_FC_Charging.n1,ground9. p)
    annotation (Line(points={{-61.0909,-2.38235},{-58,-2.38235},{-58,-8}},
                                                             color={0,0,255}));
  connect(booleanExpression.y,battery_FC_Charging. u1) annotation (Line(points={{-75,-24},
          {-69.4545,-24},{-69.4545,-11.1176}},       color={255,0,255}));
  connect(inertia.flange_a,multiSensor. flange_b) annotation (Line(points={{78,-30},
          {70,-30},{70,-10},{106,-10},{106,2},{92,2}},         color={0,0,0}));
  connect(load.flange, inertia.flange_b)
    annotation (Line(points={{108,-30},{98,-30}}, color={0,0,0}));
  connect(prescribedTemperature4.port,fuelCell_EquationBased_DetailedRohm1. port_a)
    annotation (Line(points={{-54,20},{-41,20},{-41,44}},       color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1,HTS. pin_p) annotation (Line(
        points={{-34,51.7},{-34,50},{-16,50},{-16,70},{-5,70}},color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature3.port)
    annotation (Line(points={{4.2,74},{4.2,88},{30,88}},    color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Battery_Motor;
