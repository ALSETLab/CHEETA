within CHEETA.Aircraft.Electrical.HTS;
model DCPM_CHEETA

  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{30,-74},{10,-54}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{76,-74},{56,-54}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-32,-20})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-42,-54},{-22,-34}})));
  LiquidCooled.HTS_filmboiling_Voltage hTS_filmboiling3_1(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=1,
    I_crit=10000,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0,
    a=0.1,
    b=0.5,
    P=10) annotation (Placement(transformation(extent={{-4,-4},{12,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{16,-44},{36,-24}})));
  Modelica.Blocks.Sources.Constant const1(k=733)
    annotation (Placement(transformation(extent={{42,50},{62,70}})));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={48,-20})));
equation
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{32,-64},{55,-64}}, color={0,0,127}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-32,-30},{-32,-34}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.pin_p, constantVoltage.p)
    annotation (Line(points={{-5,0},{-32,0},{-32,-10}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.port_a, prescribedTemperature.port)
    annotation (Line(points={{4.2,-4},{4.2,-64},{10,-64}}, color={191,0,0}));
  connect(dcpm.pin_an, ground1.p)
    annotation (Line(points={{42,-10},{26,-10},{26,-24}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.pin_n, dcpm.pin_ap)
    annotation (Line(points={{13,0},{54,0},{54,-10}}, color={0,0,255}));
end DCPM_CHEETA;
