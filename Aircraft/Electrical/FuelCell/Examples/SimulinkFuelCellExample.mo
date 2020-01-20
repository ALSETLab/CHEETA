within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model SimulinkFuelCellExample
  "Single branch model with averaged half bridge DCAC converter model"
  SimulinkFuelCell                                simulinkFuelCell(  R=100,
    V=500)   annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-122,0})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-82,0})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(i(fixed=true, start=5000),
                                                     L=1e-6)
    annotation (Placement(transformation(extent={{-58,-4},{-38,16}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f=10)
    annotation (Placement(transformation(extent={{-92,-40},{-72,-20}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm1(
    useConstantDutyCycle=false,
      constantDutyCycle=0.5,
    f=100)
    annotation (Placement(transformation(extent={{26,-44},{46,-24}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(i(fixed=true, start=5000),
                                                      L=1e-6)
    annotation (Placement(transformation(extent={{-58,-16},{-38,4}})));
  Aircraft.Electrical.Machines.SimpleMotor simpleMotor
    annotation (Placement(transformation(extent={{84,-10},{104,10}})));
  Aircraft.Mechanical.Loads.Fan fan(J=0.5)
    annotation (Placement(transformation(extent={{114,-4},{122,4}})));
  Aircraft.Electrical.Controls.ModulatedSignalController
    modulatedSignalController(k=10000)
    annotation (Placement(transformation(extent={{-26,-44},{-6,-24}})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
    annotation (Placement(transformation(extent={{58,6},{70,-6}})));
  Modelica.Blocks.Sources.Constant const(k=plant.Vd/2) annotation (Placement(
        transformation(
        extent={{-7,-7},{7,7}},
        rotation=90,
        origin={-17,-61})));
  Modelica.Blocks.Sources.Constant
                               const1(k=650)
    annotation (Placement(transformation(extent={{-56,-38},{-40,-22}})));
  Records.NotionalPowerSystem.Plant plant(Vd=1000)
    annotation (Placement(transformation(extent={{84,28},{104,48}})));
  Aircraft.Electrical.PowerElectronics.Converters.DCAC.DCAC_HalfBridgeAverage
    dCAC_HalfBridgeAverage
    annotation (Placement(transformation(extent={{26,-10},{46,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{66,-34},{86,-14}})));
equation
  connect(dcdc.dc_p2, inductor.p)
    annotation (Line(points={{-72,6},{-58,6}}, color={0,0,255}));
  connect(dcdc.fire_p, pwm.fire)
    annotation (Line(points={{-88,-12},{-88,-19}}, color={255,0,255}));
  connect(inductor1.p, dcdc.dc_n2)
    annotation (Line(points={{-58,-6},{-72,-6}}, color={0,0,255}));
  connect(simpleMotor.flange1, fan.flange_a1)
    annotation (Line(points={{104.4,0},{113,0}},
                                               color={0,0,0}));
  connect(pwm1.dutyCycle, modulatedSignalController.y) annotation (Line(points={{24,-34},
          {-5,-34}},                             color={0,0,127}));
  connect(simpleMotor.p1, currentSensor.n)
    annotation (Line(points={{83.6,0},{70,0}}, color={0,0,255}));
  connect(modulatedSignalController.Vdc2, const.y) annotation (Line(points={{
          -16,-45.2},{-16,-53.3},{-17,-53.3}}, color={0,0,127}));
  connect(modulatedSignalController.Reference, const1.y)
    annotation (Line(points={{-27,-30},{-39.2,-30}}, color={0,0,127}));
  connect(dcdc.dc_p1, simulinkFuelCell.pin_p) annotation (Line(points={{-92,6},
          {-104,6},{-104,4},{-115,4}}, color={0,0,255}));
  connect(modulatedSignalController.FeedbackSignal, currentSensor.i)
    annotation (Line(points={{-27,-38},{-34,-38},{-34,-78},{74,-78},{74,26},{64,
          26},{64,6.6}}, color={0,0,127}));
  connect(currentSensor.p, dCAC_HalfBridgeAverage.ac)
    annotation (Line(points={{58,0},{47.25,0}}, color={0,0,255}));
  connect(dCAC_HalfBridgeAverage.Q1gate, pwm1.fire) annotation (Line(points={{
          29.75,-11.25},{29.75,-22.55},{30,-22.55},{30,-23}}, color={255,0,255}));
  connect(dCAC_HalfBridgeAverage.Q4gate, pwm1.notFire) annotation (Line(points=
          {{42.25,-11.25},{42.25,-22.55},{42,-22.55},{42,-23}}, color={255,0,
          255}));
  connect(dCAC_HalfBridgeAverage.pin1, inductor.n) annotation (Line(points={{
          24.75,7.5},{-6,7.5},{-6,6},{-38,6}}, color={0,0,255}));
  connect(inductor1.n, dCAC_HalfBridgeAverage.pin_n1) annotation (Line(points={
          {-38,-6},{-6,-6},{-6,-7.5},{24.75,-7.5}}, color={0,0,255}));
  connect(dCAC_HalfBridgeAverage.DutyCycle, modulatedSignalController.y)
    annotation (Line(points={{42.25,11.25},{42.25,24},{18,24},{18,-34},{-5,-34}},
        color={0,0,127}));
  connect(dCAC_HalfBridgeAverage.current, currentSensor.i) annotation (Line(
        points={{29.75,11.25},{29.75,32},{64,32},{64,6.6}}, color={0,0,127}));
  connect(simulinkFuelCell.pin_p1, dcdc.dc_n1) annotation (Line(points={{-115,-4},
          {-104,-4},{-104,-6},{-92,-6}}, color={0,0,255}));
  connect(ground.p, currentSensor.n)
    annotation (Line(points={{76,-14},{76,0},{70,0}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-80},{140,60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-80},{140,
            60}})),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=100));
end SimulinkFuelCellExample;
