within CHEETA.Aircraft.Electrical.Battery;
model Battery "Battery with SOC output"
  extends Modelon.Electrical.EnergyStorage.Interfaces.Base;
  extends Modelon.Thermal.HeatTransfer.Interfaces.ConditionalHeatPort;

  replaceable
    Modelon.Electrical.EnergyStorage.Components.BatteryPackEMFModShepherd
    stackVoltage(final SOC_start=SOC_start) constrainedby
    Modelon.Electrical.EnergyStorage.Components.EMFInterface(SOC_start=SOC_start) annotation (choicesAllMatching=true, Placement(transformation(
        extent={{30,30},{-10,-10}},
        origin={30,-10})));
  Modelica.Electrical.Analog.Basic.Resistor internalResistance(R=R_int,
    useHeatPort=enable_heatport,
    T_ref=T0,
    alpha=0,
    T=T0)
    annotation (Placement(transformation(extent={{0,-20},{-40,20}})));
  Modelica.Blocks.Interfaces.RealOutput SOC "State of Charge" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          origin={110,0})));

  parameter Real V_min=0
    "Minimum allowable terminal voltage (simulation terminated if below)";

  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
    annotation (Placement(transformation(extent={{-20,-60},{-40,-40}})));
  Modelon.Blocks.Terminate terminate(message=
        "Battery voltage to low, this may be due to a too high current")
    annotation (Placement(transformation(extent={{10,-80},{30,-60}})));
  Modelica.Blocks.Logical.LessThreshold voltageLimit(threshold=V_min)
    annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));

  final parameter Modelica.SIunits.Mass m_battery = stackVoltage.ns*stackVoltage.np*stackVoltage.cellInfo.m_cell
    "Battery mass, calculated from cell mass";

  final parameter Modelica.SIunits.Resistance R_int = (stackVoltage.ns/stackVoltage.np)*stackVoltage.cellInfo.R_cell
    "Battery internal resistance, calculated from cell resistance and number of series/parallel cells";

  parameter Real SOC_start=0.6 "Initial SOC";
equation
  connect(internalResistance.p, stackVoltage.n)
                                      annotation (Line(
      points={{0,0},{20,0}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(voltageSensor.p, stackVoltage.p) annotation (Line(
      points={{-20,-50},{60,-50},{60,0}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(voltageSensor.n, internalResistance.n) annotation (Line(
      points={{-40,-50},{-60,-50},{-60,0},{-40,0}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(voltageLimit.y,terminate. u) annotation (Line(
      points={{1,-70},{9,-70}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(voltageLimit.u,voltageSensor. v) annotation (Line(
      points={{-22,-70},{-30,-70},{-30,-61}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(stackVoltage.SoC, SOC) annotation (Line(
      points={{40,-14},{40,-20},{80,-20},{80,0},{110,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pin_p, stackVoltage.p) annotation (Line(
      points={{40,100},{40,60},{60,60},{60,0}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(internalResistance.n, pin_n) annotation (Line(
      points={{-40,0},{-60,0},{-60,60},{-40,60},{-40,100}},
      color={0,0,255},
      smooth=Smooth.None));

  connect(internalResistance.heatPort, heatPort) annotation (Line(
      points={{-20,-20},{-20,-30},{100,-30},{100,-100}},
      color={191,0,0},
      smooth=Smooth.None));
 annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}})),                Icon(coordinateSystem(
          preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={
        Rectangle(
          extent={{-80,60},{80,-80}},
          lineColor={95,95,95},
          fillPattern=FillPattern.Solid,
          fillColor={215,215,215}),
        Line(
          points={{-50,90},{-50,60}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{50,90},{50,60}},
          color={95,95,95},
          smooth=Smooth.None),
        Text(
          extent={{90,60},{50,100}},
          lineColor={95,95,95},
          textString="+"),
        Text(
          extent={{-50,60},{-90,100}},
          lineColor={95,95,95},
          textString="-"),
        Line(
          points={{-60,-58},{-60,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-40,-58},{-40,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-20,-58},{-20,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{0,-58},{0,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{20,-58},{20,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{40,-58},{40,22}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-58},{60,22}},
          color={95,95,95},
          smooth=Smooth.None)}),
    Documentation(info="<html>
<p>The Battery model uses BatteryPackEMF to determine SoC-dependent or charge-dependent voltage. The total internal resistance is modeled as a series resistance, the value of which is determined by the cell internal resistance given by the cellInfo parameter, as well as the number and arrangement (series/parallel) of cells in the battery pack. </p>
<p><br>The user can select between three different EMF voltage models, Proportional, Shepherd and Extended Shepherd. The discharge voltage and SoC curves are visible below.</p>
<p><br>Voltage as a function of time (SoC)</p>
<p><br><img src=\"modelica://Modelon/Resources/Images/Electrical/EnergyStorage/VoltagePlot.png\"/></p>
<p><br>StateOf Charge during a discharge cycle.</p>
<p><br><img src=\"modelica://Modelon/Resources/Images/Electrical/EnergyStorage/SoC.png\"/></p>
<p><br>Note that SoC can take values below 0 in the proportional model. This model will be removed in the future.</p>
<p>A voltage sensor is used to monitor terminal voltage. Simulation is terminated if the terminal voltage falls below a given value. </p>
</html>"));
end Battery;
