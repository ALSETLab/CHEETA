within CHEETA.Aircraft.Electrical.HTS;
model EPTL_CHEETA

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
              ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Speed
                                                                          controller(
      redeclare
      ElectrifiedPowertrains.ElectricMachines.PSM.Controllers.Records.Base.Speed
      data(redeclare
        CHEETA.Aircraft.Electrical.PowerElectronics.Examples.Records.VSI_20kW
        machineData))
    annotation (Placement(transformation(extent={{90,-22},{110,-2}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
                                                             modulationMethod(k3=0)
    annotation (Placement(transformation(extent={{124,-22},{144,-2}})));
              ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.LossyLinearized
                                                         inverter(
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Losses.Records.Data.LossyLinearized.IGBT_ModuleSemikron_600V_6A
      data,
    fs=20e3)
    annotation (Placement(transformation(extent={{164,-22},{184,-2}})));
equation
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{32,-64},{55,-64}}, color={0,0,127}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{-32,-30},{-32,-34}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.pin_p, constantVoltage.p)
    annotation (Line(points={{-5,0},{-32,0},{-32,-10}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.port_a, prescribedTemperature.port)
    annotation (Line(points={{4.2,-4},{4.2,-64},{10,-64}}, color={191,0,0}));
  connect(controller.actuatingVoltages,modulationMethod. phaseVoltages)
    annotation (Line(points={{111,-12},{122,-12}},
                                                 color={0,0,127}));
  connect(inverter.normalizedPhaseVoltages,modulationMethod. normalizedPhaseVoltages)
    annotation (Line(points={{162,-12},{145,-12}},color={0,0,127}));
  connect(controller.desiredSpeed, const1.y) annotation (Line(points={{88,-12},
          {70,-12},{70,60},{63,60}}, color={0,0,127}));
  connect(hTS_filmboiling3_1.pin_n, inverter.pin_p) annotation (Line(points={{
          13,0},{158,0},{158,-6},{164,-6}}, color={0,0,255}));
  connect(inverter.pin_n, ground1.p) annotation (Line(points={{164,-18},{96,-18},
          {96,-24},{26,-24}}, color={0,0,255}));
end EPTL_CHEETA;
