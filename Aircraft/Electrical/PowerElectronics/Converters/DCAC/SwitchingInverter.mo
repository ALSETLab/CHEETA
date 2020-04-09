within CHEETA.Aircraft.Electrical.PowerElectronics.Converters.DCAC;
model SwitchingInverter "Single phase DC to AC converter with switching"

 import ElectrifiedPowertrains;

  extends ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Interfaces.ThermalBase;
  extends DymolaModels.Icons.Models.Inverter;

  parameter Modelica.SIunits.Resistance RonTransistor=1e-05
    "Transistor closed resistance";
  parameter Modelica.SIunits.Conductance GoffTransistor=1e-05
    "Transistor opened conductance";
  parameter Modelica.SIunits.Voltage VkneeTransistor=0
    "Transistor threshold voltage";
  parameter Modelica.SIunits.Resistance RonDiode=1e-05
    "Diode closed resistance";
  parameter Modelica.SIunits.Conductance GoffDiode=1e-05
    "Diode opened conductance";
  parameter Modelica.SIunits.Voltage VkneeDiode=0 "Diode threshold voltage";
  parameter Integer m = 3 "Number of phases";
  Modelica.Electrical.MultiPhase.Ideal.IdealGTOThyristor transistor_p(
    final m=m,
    final Ron=fill(RonTransistor, m),
    final Goff=fill(GoffTransistor, m),
    final Vknee=fill(VkneeTransistor, m),
    useHeatPort=false)                    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={6,20})));
  Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_p(
    final m=m,
    final Ron=fill(RonDiode, m),
    final Goff=fill(GoffDiode, m),
    final Vknee=fill(VkneeDiode, m),
    useHeatPort=false)               annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={46,20})));
  Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m) annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={26,50})));
  Modelica.Electrical.MultiPhase.Ideal.IdealGTOThyristor transistor_n(
    final m=m,
    final Ron=fill(RonTransistor, m),
    final Goff=fill(GoffTransistor, m),
    final Vknee=fill(VkneeTransistor, m),
    useHeatPort=false)                    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={6,-20})));
  Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
    final m=m,
    final Ron=fill(RonDiode, m),
    final Goff=fill(GoffDiode, m),
    final Vknee=fill(VkneeDiode, m),
    useHeatPort=false)               annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={46,-20})));
  Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m) annotation (
      Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={26,-50})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM3
                                                                      [
    m](each useConstantDutyCycle=true, each f(displayUnit="kHz") = 100000)
                                                  annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        origin={-52,-54})));
  parameter Boolean useHeatPort=true "=true, if all heat ports are enabled";
equation

  connect(transistor_p.plug_p,star_p. plug_p) annotation (Line(
      points={{6,30},{26,30},{26,40}},  color={0,0,255}));
  connect(star_p.plug_p,diode_p. plug_n) annotation (Line(
      points={{26,40},{26,30},{46,30}}, color={0,0,255}));
  connect(transistor_n.plug_n,star_n. plug_p) annotation (Line(
      points={{6,-30},{26,-30},{26,-40}},  color={0,0,255}));
  connect(star_n.plug_p,diode_n. plug_p) annotation (Line(
      points={{26,-40},{26,-30},{46,-30}}, color={0,0,255}));
  connect(transistor_p.plug_n,diode_p. plug_p) annotation (Line(
      points={{6,10},{46,10}},  color={0,0,255}));
  connect(transistor_n.plug_p,diode_n. plug_n) annotation (Line(
      points={{6,-10},{46,-10}},  color={0,0,255}));
  connect(diode_n.plug_n, diode_p.plug_p) annotation (Line(points={{46,-10},{26,
          -10},{26,10},{46,10}}, color={0,0,255}));
  connect(quasiPowerSensor.nc, diode_p.plug_p)
    annotation (Line(points={{70,0},{26,0},{26,10},{46,10}}, color={0,0,255}));
  connect(star_p.pin_n, currentSensor.n)
    annotation (Line(points={{26,60},{26,80},{-82,80}}, color={0,0,255}));
  connect(star_n.pin_n, pin_n)
    annotation (Line(points={{26,-60},{26,-80},{-140,-80}}, color={0,0,255}));
  connect(signalPWM3.fire, transistor_n.fire) annotation (Line(points={{-46,-43},
          {-46,-30},{-6,-30}}, color={255,0,255}));
  connect(signalPWM3.notFire, transistor_p.fire)
    annotation (Line(points={{-58,-43},{-58,10},{-6,10}}, color={255,0,255}));
  connect(transistor_p.heatPort[3], thermalPortInverter.heatPort_switch)
    annotation (Line(points={{16,20},{22,20},{22,122},{-1,122}}, color={191,0,0}));
  connect(transistor_n.heatPort, transistor_p.heatPort) annotation (Line(points=
         {{16,-20},{22,-20},{22,20},{16,20}}, color={191,0,0}));
  connect(diode_p.heatPort[3], thermalPortInverter.heatPort_diode) annotation (
      Line(points={{56,20},{76,20},{76,100},{22,100},{22,122},{-1,122}}, color={
          191,0,0}));
  connect(diode_n.heatPort, diode_p.heatPort) annotation (Line(points={{56,-20},
          {76,-20},{76,20},{56,20}}, color={191,0,0}));
  annotation (defaultComponentName="inverter",
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}}), graphics={
        Line(
          points={{-100,-100},{100,100}},
          color={0,0,127})}),
    Documentation(info="<html>
<p>
This is a single phase two level inverter. The boolean signals <code>fire_p</code> and <code>fire_n</code> shall not be <code>true</code> at the same time to avoid DC bus short circuits. The inverter consists of two transistors and two anti parallel free wheeling diodes.
</p>

<p>
An example of a single phase inverter with PWM voltage control is included in
<a href=\"modelica://Modelica.Electrical.PowerConverters.Examples.DCAC.SinglePhaseTwoLevel\">Examples.DCAC.SinglePhaseTwoLevel</a>.
</p>
</html>"));
end SwitchingInverter;
