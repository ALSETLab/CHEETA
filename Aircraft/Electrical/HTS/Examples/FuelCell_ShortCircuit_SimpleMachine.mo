within CHEETA.Aircraft.Electrical.HTS.Examples;
model FuelCell_ShortCircuit_SimpleMachine

  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{8,-60},{28,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{28,24},{8,44}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{74,24},{54,44}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor3
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={18,-22})));
  Modelica.Blocks.Sources.Ramp     ramp(
    height=-9999.99,
    duration=0.5,
    offset=10000,
    startTime=10)
    annotation (Placement(transformation(extent={{-34,-32},{-14,-12}})));
  PowerElectronics.Converters.DCAC.DCAC_SinglePhase   inverter
    annotation (Placement(transformation(extent={{42,-32},{62,-12}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature2
    annotation (Placement(transformation(extent={{-96,-44},{-76,-24}})));
  Modelica.Blocks.Sources.Constant const2(k=353.15)
    annotation (Placement(transformation(extent={{-130,-44},{-110,-24}})));
  Machines.Motors.SimpleMotor simpleMotor
    annotation (Placement(transformation(extent={{78,-32},{98,-12}})));
  LiquidCooled.HTS_filmboiling_Voltage_Hydrogen HTS(
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
    annotation (Placement(transformation(extent={{-28,6},{-12,-2}})));
  FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased_DetailedRohm1(n=2, R_ohm_current=0.075)
    annotation (Placement(transformation(extent={{-74,-8},{-60,6}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM(each
      useConstantDutyCycle=true, each f=60) annotation (Placement(
        transformation(extent={{-10,-10},{10,10}}, origin={52,-58})));
  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    linearSpeedDependentTorque(tau_nominal=500, w_nominal=4500)
    annotation (Placement(transformation(extent={{130,-32},{110,-12}})));
equation
  connect(const1.y, prescribedTemperature1.T)
    annotation (Line(points={{53,34},{30,34}}, color={0,0,127}));
  connect(resistor3.n, ground2.p)
    annotation (Line(points={{18,-32},{18,-40}},            color={0,0,255}));
  connect(resistor3.R,ramp. y)
    annotation (Line(points={{6,-22},{-13,-22}}, color={0,0,127}));
  connect(inverter.dc_n, ground2.p) annotation (Line(points={{42,-28},{30,-28},
          {30,-40},{18,-40}},   color={0,0,255}));
  connect(const2.y,prescribedTemperature2. T)
    annotation (Line(points={{-109,-34},{-98,-34}},
                                               color={0,0,127}));
  connect(inverter.ac, simpleMotor.p1)
    annotation (Line(points={{62,-22},{77.6,-22}}, color={0,0,255}));
  connect(HTS.pin_n, resistor3.p)
    annotation (Line(points={{-11,2},{18,2},{18,-12}}, color={0,0,255}));
  connect(prescribedTemperature1.port, HTS.port_a)
    annotation (Line(points={{8,34},{-19.8,34},{-19.8,6}}, color={191,0,0}));
  connect(fuelCell_EquationBased_DetailedRohm1.p1, HTS.pin_p) annotation (Line(
        points={{-60,-0.3},{-46,-0.3},{-46,2},{-29,2}}, color={0,0,255}));
  connect(fuelCell_EquationBased_DetailedRohm1.port_a, prescribedTemperature2.port)
    annotation (Line(points={{-67,-8},{-68,-8},{-68,-34},{-76,-34}}, color={191,
          0,0}));
  connect(inverter.dc_p, resistor3.p) annotation (Line(points={{42,-16},{30,-16},
          {30,2},{18,2},{18,-12}}, color={0,0,255}));
  connect(inverter.fire_p, signalPWM.fire)
    annotation (Line(points={{46,-34},{46,-47}}, color={255,0,255}));
  connect(inverter.fire_n, signalPWM.notFire)
    annotation (Line(points={{58,-34},{58,-47}}, color={255,0,255}));
  connect(simpleMotor.flange1, linearSpeedDependentTorque.flange)
    annotation (Line(points={{98.4,-22},{110,-22}}, color={0,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-140,-100},{100,60}})), Icon(
        coordinateSystem(extent={{-140,-100},{100,60}})),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "../../../image/HTS/Fault Test - One Line/PlotFaults.mos"
        "PlotFaults"));
end FuelCell_ShortCircuit_SimpleMachine;
