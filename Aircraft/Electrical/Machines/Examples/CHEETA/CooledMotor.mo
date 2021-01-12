within CHEETA.Aircraft.Electrical.Machines.Examples.CHEETA;
model CooledMotor
  import CHEETA;
  parameter Real pi = Modelica.Constants.pi;
  parameter Integer m = 3 "Number of phases";
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{58,-60},{78,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature3
    annotation (Placement(transformation(extent={{64,24},{44,44}})));
  Modelica.Blocks.Sources.Constant const3(k=20)
    annotation (Placement(transformation(extent={{110,24},{90,44}})));
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
    annotation (Placement(transformation(extent={{-46,4},{-30,-4}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=-1500)
    annotation (Placement(transformation(extent={{154,-30},{134,-10}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    electricDrive1(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Records.Base.Speed
        data(redeclare
          CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM2
          machineData, tau_max=10000)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod,
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=false,
    redeclare
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Linear
      machine(redeclare
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM2
        data))
    annotation (Placement(transformation(extent={{86,-30},{106,-10}})));
  Modelica.Blocks.Sources.Constant const1(k=733.038285)
    annotation (Placement(transformation(extent={{124,-6},{112,6}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,-20})));
  CHEETA.Aircraft.Electrical.Battery.Battery_FC_Charging battery_FC_Charging
    annotation (Placement(transformation(
        extent={{-17,-11},{17,11}},
        rotation=270,
        origin={-21,-23})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=false)
    annotation (Placement(transformation(extent={{-50,-62},{-30,-42}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor
    annotation (Placement(transformation(extent={{114,-26},{126,-14}})));
equation
  connect(const3.y,prescribedTemperature3. T)
    annotation (Line(points={{89,34},{66,34}}, color={0,0,127}));
  connect(HTS.port_a,prescribedTemperature3. port)
    annotation (Line(points={{-37.8,4},{-37.8,34},{44,34}}, color={191,0,0}));
  connect(electricDrive1.pin_n, ground1.p) annotation (Line(points={{86,-26},{
          84,-26},{84,-40},{68,-40}}, color={0,0,255}));
  connect(const1.y, electricDrive1.desiredSpeed)
    annotation (Line(points={{111.4,0},{96,0},{96,-8}}, color={0,0,127}));
  connect(constantVoltage.n, ground1.p)
    annotation (Line(points={{-80,-30},{-80,-40},{68,-40}}, color={0,0,255}));
  connect(HTS.pin_p, constantVoltage.p)
    annotation (Line(points={{-47,0},{-80,0},{-80,-10}}, color={0,0,255}));
  connect(battery_FC_Charging.n1, ground1.p)
    annotation (Line(points={{-12,-27},{-12,-40},{68,-40}}, color={0,0,255}));
  connect(booleanExpression.y, battery_FC_Charging.u1) annotation (Line(points=
          {{-29,-52},{-22,-52},{-22,-46},{-20,-46},{-20,-36}}, color={255,0,255}));
  connect(HTS.pin_n, electricDrive1.pin_p) annotation (Line(points={{-29,0},{28,
          0},{28,-14},{86,-14}}, color={0,0,255}));
  connect(battery_FC_Charging.p1, electricDrive1.pin_p) annotation (Line(points
        ={{-12,-17},{-12,0},{28,0},{28,-14},{86,-14}}, color={0,0,255}));
  connect(constantTorque.flange, multiSensor.flange_b)
    annotation (Line(points={{134,-20},{126,-20}}, color={0,0,0}));
  connect(multiSensor.flange_a, electricDrive1.flange)
    annotation (Line(points={{114,-20},{106,-20}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{160,60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{160,
            60}})),
    Documentation(info="<html>
</html>"),
    experiment(StopTime=10));
end CooledMotor;
