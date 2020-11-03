within CHEETA.Aircraft.Electrical.HTS.Examples;
model EPTL_CHEETA

  Modelica.Blocks.Sources.Constant
                               tauRef(k=733.038285)
               annotation (Placement(transformation(extent={{-28,12},{-8,32}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-20,-64},{-40,-44}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{26,-64},{6,-44}})));
  LiquidCooled.HTS_filmboiling_Voltage_Hydrogen HTS(
    l=10,
    n=20,
    I_c0=3700,
    A=0.1,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0,
    epsilon_r=2.2)
           annotation (Placement(transformation(extent={{-60,-4},{-44,4}})));
  ElectrifiedPowertrains.ElectricDrives.ESM.ElectroMechanical.SpeedFOC
    electricDrive(
    redeclare ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Speed
      controller(redeclare
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM_Controller
        data),
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
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_2_5MW_ESM
        data))
    annotation (Placement(transformation(extent={{8,-16},{28,4}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-16,-40},{4,-20}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
    annotation (Placement(transformation(extent={{40,-48},{60,-28}})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{42,-12},
            {54,0}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque             load(
      tau_constant=-8000)                 annotation (Placement(transformation(extent={{96,-48},
            {76,-28}})));
  FuelCell.FuelCell_EquationBased
    fuelCell_EquationBased1(R_ohm_current=0.01)
    annotation (Placement(transformation(extent={{-88,-8},{-74,6}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature4
    annotation (Placement(transformation(extent={{-106,-50},{-86,-30}})));
  Modelica.Blocks.Sources.Constant const4(k=353.15)
    annotation (Placement(transformation(extent={{-138,-50},{-118,-30}})));
equation
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-18,-54},{5,-54}}, color={0,0,127}));
  connect(ground1.p,electricDrive. pin_n)
    annotation (Line(points={{-6,-20},{-6,-12},{8,-12}},     color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature.port) annotation (Line(points={{-51.8,
          -4},{-52,-4},{-52,-54},{-40,-54}}, color={191,0,0}));
  connect(HTS.pin_n, electricDrive.pin_p)
    annotation (Line(points={{-43,0},{8,0}}, color={0,0,255}));
  connect(tauRef.y, electricDrive.desiredSpeed)
    annotation (Line(points={{-7,22},{18,22},{18,6}}, color={0,0,127}));
  connect(multiSensor.flange_a, electricDrive.flange)
    annotation (Line(points={{42,-6},{28,-6}}, color={0,0,0}));
  connect(inertia.flange_a, multiSensor.flange_b) annotation (Line(points={{40,
          -38},{34,-38},{34,-20},{70,-20},{70,-6},{54,-6}}, color={0,0,0}));
  connect(inertia.flange_b, load.flange)
    annotation (Line(points={{60,-38},{76,-38}}, color={0,0,0}));
  connect(prescribedTemperature4.port,fuelCell_EquationBased1. port_a)
    annotation (Line(points={{-86,-40},{-81,-40},{-81,-8}},     color={191,0,0}));
  connect(fuelCell_EquationBased1.p1, HTS.pin_p)
    annotation (Line(points={{-74,-0.3},{-74,0},{-61,0}}, color={0,0,255}));
  connect(const4.y, prescribedTemperature4.T)
    annotation (Line(points={{-117,-40},{-108,-40}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-140,-80},{100,40}})),  Icon(
        coordinateSystem(extent={{-140,-80},{100,40}})));
end EPTL_CHEETA;
