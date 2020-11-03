within CHEETA.Aircraft.Electrical.HTS.Examples;
model Inverter_Fault_resistor

  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-84,-14})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-94,-50},{-74,-30}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-18,-90},{-38,-70}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{16,-90},{-4,-70}})));
  LiquidCooled.HTS_filmboiling_Voltage_Hydrogen hTS_filmboiling3_2(
    l=10,
    n=20,
    I_c0=37000,
    A=0.1,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0) annotation (Placement(transformation(extent={{-60,-4},{-44,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-44,-60},{-24,-40}})));
  Modelica.Electrical.PowerConverters.DCAC.SinglePhase2Level inverter
    annotation (Placement(transformation(extent={{-24,-16},{-4,4}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM(
    useConstantDutyCycle=true,
    constantDutyCycle=0.5,
    f=1)                               annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={-14,-34})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=100)
    annotation (Placement(transformation(extent={{28,-16},{48,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{44,-44},{64,-24}})));
equation
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-16,-80},{-5,-80}},color={0,0,127}));
  connect(constantVoltage.p, hTS_filmboiling3_2.pin_p)
    annotation (Line(points={{-84,-4},{-84,0},{-61,0}}, color={0,0,255}));
  connect(hTS_filmboiling3_2.port_a, prescribedTemperature.port) annotation (
      Line(points={{-51.8,-4},{-52,-4},{-52,-80},{-38,-80}}, color={191,0,0}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-84,-24},{-84,-30}}, color={0,0,255}));
  connect(hTS_filmboiling3_2.pin_n, inverter.dc_p)
    annotation (Line(points={{-43,0},{-24,0}}, color={0,0,255}));
  connect(inverter.dc_n, ground1.p)
    annotation (Line(points={{-24,-12},{-34,-12},{-34,-40}}, color={0,0,255}));
  connect(inverter.fire_p, signalPWM.fire)
    annotation (Line(points={{-20,-18},{-20,-23}}, color={255,0,255}));
  connect(inverter.fire_n, signalPWM.notFire)
    annotation (Line(points={{-8,-18},{-8,-23}}, color={255,0,255}));
  connect(inverter.ac, resistor.p)
    annotation (Line(points={{-4,-6},{28,-6}}, color={0,0,255}));
  connect(resistor.n, ground2.p)
    annotation (Line(points={{48,-6},{54,-6},{54,-24}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,60}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,60}})));
end Inverter_Fault_resistor;
