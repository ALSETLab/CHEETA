within CHEETA.Aircraft.Electrical.HTS;
model HTS_Cooling "Simple cooling circuit"

  parameter Modelica.Thermal.FluidHeatFlow.Media.Medium medium=Modelica.Thermal.FluidHeatFlow.Media.Medium()
    "Cooling medium"
    annotation(choicesAllMatching=true);
  parameter Modelica.SIunits.Temperature TAmb(displayUnit="degC")=293.15
    "Ambient temperature";
  output Modelica.SIunits.TemperatureDifference dTSource=
    prescribedHeatFlow.port.T-TAmb "Source over Ambient";
  output Modelica.SIunits.TemperatureDifference dTtoPipe=prescribedHeatFlow.port.T-pipe.T_q
    "Source over Coolant";
  output Modelica.SIunits.TemperatureDifference dTCoolant=pipe.dT
    "Coolant's temperature increase";
  Modelica.Thermal.FluidHeatFlow.Sources.Ambient ambient1(constantAmbientTemperature=TAmb, medium=medium,
    constantAmbientPressure=0)
    annotation (Placement(transformation(extent={{-28,24},{-48,44}})));
  Modelica.Thermal.FluidHeatFlow.Sources.VolumeFlow pump(
    medium=medium,
    m=0,
    T0=TAmb,
    useVolumeFlowInput=true,
    constantVolumeFlow=1)
    annotation (Placement(transformation(extent={{-10,24},{10,44}})));
  Modelica.Thermal.FluidHeatFlow.Components.Pipe pipe(
    medium=medium,
    m=0.1,
    T0=TAmb,
    V_flowLaminar=0.1,
    dpLaminar(displayUnit="Pa") = 0.1,
    V_flowNominal=1,
    dpNominal(displayUnit="Pa") = 1,
    h_g=0,
    T0fixed=true,
    useHeatPort=true)
    annotation (Placement(transformation(extent={{32,24},{52,44}})));
  Modelica.Thermal.FluidHeatFlow.Sources.Ambient ambient2(constantAmbientTemperature=TAmb, medium=medium,
    constantAmbientPressure=0)
    annotation (Placement(transformation(extent={{68,24},{88,44}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor HTS_Cooler(C=0.1, T(
        start=TAmb, fixed=true)) annotation (Placement(transformation(
        origin={48,-20},
        extent={{-10,10},{10,-10}},
        rotation=90)));
  Modelica.Blocks.Sources.Constant volumeFlow(k=1)
    annotation (Placement(transformation(extent={{-28,44},{-8,64}})));
  Modelica.Thermal.HeatTransfer.Components.Convection convection
    annotation (Placement(transformation(
        origin={30,8},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  Modelica.Blocks.Sources.Constant thermalConductance(k=1)
    annotation (Placement(transformation(extent={{-10,-2},{10,18}})));
  Thermal.PrescribedHeatFlow                           prescribedHeatFlow
    annotation (Placement(transformation(extent={{-2,-30},{18,-10}})));
  Modelica.Blocks.Sources.Trapezoid trapezoid(
    amplitude=77,
    rising=3,
    width=2,
    falling=3,
    period=10,
    nperiod=1)
    annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  parameter Real l "Length of wire";
  HTS LINE annotation (Placement(transformation(extent={{-10,-66},{12,-56}})));
  Modelica.Electrical.Analog.Interfaces.Pin p_in1
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a temperature1
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin p_out1
    annotation (Placement(transformation(extent={{110,-10},{90,10}})));
equation
  connect(ambient1.flowPort, pump.flowPort_a)
    annotation (Line(points={{-28,34},{-10,34}},
                                               color={255,0,0}));
  connect(pump.flowPort_b, pipe.flowPort_a)
    annotation (Line(points={{10,34},{32,34}},
                                             color={255,0,0}));
  connect(pipe.flowPort_b, ambient2.flowPort)
    annotation (Line(points={{52,34},{68,34}},
                                             color={255,0,0}));
  connect(convection.solid, HTS_Cooler.port) annotation (Line(points={{30,
          -2},{30,-20},{38,-20}}, color={191,0,0}));
  connect(pipe.heatPort, convection.fluid) annotation (Line(points={{42,24},
          {30,24},{30,18}},
                          color={191,0,0}));
  connect(thermalConductance.y, convection.Gc)
                               annotation (Line(points={{11,8},{20,8}},    color={0,0,127}));
  connect(volumeFlow.y, pump.volumeFlow) annotation (Line(
      points={{-7,54},{0,54},{0,44}},      color={0,0,127}));
  connect(prescribedHeatFlow.port, HTS_Cooler.port)
    annotation (Line(points={{18,-20},{38,-20}}, color={191,0,0}));
  connect(prescribedHeatFlow.Q_flow, trapezoid.y)
    annotation (Line(points={{-2,-20},{-19,-20}}, color={0,0,127}));
  connect(LINE.p_in, p_in1) annotation (Line(points={{-11.925,-61.25},{-100,
          -61.25},{-100,0}}, color={0,0,255}));
  connect(LINE.temperature, temperature1) annotation (Line(points={{0.8625,
          -67.875},{0.8625,-100},{0,-100}}, color={191,0,0}));
  connect(LINE.p_out, p_out1) annotation (Line(points={{13.925,-61.25},{100,
          -61.25},{100,0}}, color={0,0,255}));
annotation (Documentation(info="<html>
<p>
1st test example: SimpleCooling
</p>
A prescribed heat source dissipates its heat through a thermal conductor to a coolant flow. The coolant flow is taken from an ambient and driven by a pump with prescribed mass flow.<br>
<strong>Results</strong>:<br>
<table>
<tr>
<td><strong>output</strong></td>
<td><strong>explanation</strong></td>
<td><strong>formula</strong></td>
<td><strong>actual steady-state value</strong></td>
</tr>
<tr>
<td>dTSource</td>
<td>Source over Ambient</td>
<td>dtCoolant + dtToPipe</td>
<td>20 K</td>
</tr>
<tr>
<td>dTtoPipe</td>
<td>Source over Coolant</td>
<td>Losses / ThermalConductor.G</td>
<td>10 K</td>
</tr>
<tr>
<td>dTCoolant</td>
<td>Coolant's temperature increase</td>
<td>Losses * cp * massFlow</td>
<td>10 K</td>
</tr>
</table>
</html>"),    experiment(StopTime=1.0, Interval=0.001),
    Icon(graphics={Line(
          points={{-102,0},{78,0}},
          color={0,140,72},
          thickness=1)}));
end HTS_Cooling;
