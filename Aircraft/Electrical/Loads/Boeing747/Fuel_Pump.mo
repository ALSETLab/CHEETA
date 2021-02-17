within CHEETA.Aircraft.Electrical.Loads.Boeing747;
model Fuel_Pump
  extends CHEETA.Aircraft.Electrical.Loads.Boeing747.DC_Load(
      simulink_Averaged_Rectifier(P_fixed=50, V_rated=28),           yd(N=N));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_ElectricalExcited
    Fuel_Pump(
    VaNominal=28,
    IaNominal=300/28,
    wNominal=198.96753472735,
    TaNominal=298.15,
    Ra=Data.Ra,
    alpha20a(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    La=Data.La,
    Jr=Data.J,
    phiMechanical(start=0, fixed=true),
    wMechanical(start=0, fixed=true),
    ia(start=0, fixed=true),
    Re=Data.Rf,
    alpha20e(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    Le=Data.Lf,
    sigmae=Data.Laf/Data.Lf,
    ie(start=0, fixed=true))  annotation (Placement(transformation(
        extent={{-13,-13},{13,13}},
        rotation=0,
        origin={125,11})));

  Modelica.Mechanics.Rotational.Sources.LinearSpeedDependentTorque
    FuelPumpLoad(
    useSupport=false,
    tau_nominal=L,
    w_nominal=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={158,12})));
  parameter Modelica.Units.SI.Torque L=-L
    "Nominal torque (if negative, torque is acting as load)";
  replaceable CHEETA.Records.Boeing747electricalModel.DCMotor.FuelPumps.FuelPump_2 Data
    constrainedby CHEETA.Records.Boeing747electricalModel.Base.DC_Motor_Data
    annotation (Placement(transformation(extent={{94,-80},{114,-60}})));

  parameter Modelica.Units.SI.Resistance R=0.5
    "Resistance of Auxillary Loads (Ohms)";
  Modelica.Electrical.Analog.Basic.Resistor DC_Load_Lamp(R=1)
    annotation (Placement(transformation(extent={{14,-48},{34,-28}})));
  Modelica.Electrical.Analog.Basic.Resistor DC_Load_Heater(R=1)
    annotation (Placement(transformation(extent={{14,-78},{34,-58}})));
  Modelica.Electrical.Polyphase.Basic.Star star2 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-28,-54})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-38,-94},{-18,-74}})));
  Modelica.Electrical.Polyphase.Basic.Capacitor capacitor(C={1e-6,1e-6,1e-6})
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-28,-30})));
equation
  connect(DC_Load_Lamp.n,DC_Load_Heater. n)
    annotation (Line(points={{34,-38},{34,-68}}, color={0,0,255}));
  connect(DC_Load_Lamp.n, simulink_Averaged_Rectifier.v_ref) annotation (Line(points={{34,-38},
          {36,-38},{36,0},{-9,0}},                color={0,0,255}));
  connect(DC_Load_Lamp.p, simulink_Averaged_Rectifier.DC_n) annotation (Line(points={{14,-38},
          {6,-38},{6,-6},{-9,-6}},             color={0,0,255}));
  connect(DC_Load_Heater.p, simulink_Averaged_Rectifier.DC_n) annotation (Line(
        points={{14,-68},{14,-38},{14,-6},{-9,-6}},   color={0,0,255}));
  connect(capacitor.plug_n,star2. plug_p)
    annotation (Line(points={{-28,-40},{-28,-44}},
                                                 color={0,0,255}));
  connect(ground2.p,star2. pin_n)
    annotation (Line(points={{-28,-74},{-28,-64}},
                                                 color={0,0,255}));
  connect(capacitor.plug_p, yd.Secondary1) annotation (Line(points={{-28,
          -20},{-28,0},{-45.6,0}}, color={0,0,255}));
  connect(FuelPumpLoad.flange, Fuel_Pump.flange) annotation (Line(points=
          {{148,12},{138,12},{138,11}}, color={0,0,0}));
  connect(Fuel_Pump.pin_ep, Fuel_Pump.pin_ap) annotation (Line(points={{
          112,18.8},{112,46},{132.8,46},{132.8,24}}, color={0,0,255}));
  connect(Fuel_Pump.pin_an, Fuel_Pump.pin_en) annotation (Line(points={{
          117.2,24},{106,24},{106,0},{112,0},{112,3.2}}, color={0,0,255}));
  connect(simulink_Averaged_Rectifier.DC_p, Fuel_Pump.pin_ap) annotation (Line(
        points={{-9,6},{132.8,6},{132.8,46},{132.8,24}},color={0,0,255}));
  connect(Fuel_Pump.pin_en, simulink_Averaged_Rectifier.v_ref) annotation (Line(
        points={{112,3.2},{74,3.2},{74,0},{-9,0}},   color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{180,100}})), Icon(
        coordinateSystem(extent={{-100,-100},{180,100}})),
    Documentation(info="<html>
<p>This model is a fuel pump to be used as a load for the 747 electrical model. It consists of a DC machine driving a fixed load.</p>
</html>"));
end Fuel_Pump;
