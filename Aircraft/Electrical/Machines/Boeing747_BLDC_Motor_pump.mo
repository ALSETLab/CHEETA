within CHEETA.Aircraft.Electrical.Machines;
model Boeing747_BLDC_Motor_pump "Boeing 747 BLDC motor pump"
  Modelica.Electrical.Polyphase.Interfaces.PositivePlug Bus
    annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
  PowerElectronics.Converters.ACDC.Snubber_Rectifier aCDC(
    Rcond=1e-3,
    Vt=0.8,
    R1=1e3) annotation (Placement(transformation(extent={{-60,-40},{-38,-20}})));
  PowerElectronics.Converters.ACDC.Snubber_Rectifier aCDC1(
    Rcond=1e-3,
    Vt=0.8,
    R1=1e3) annotation (Placement(transformation(extent={{-60,20},{-38,40}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=2e-3) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,16})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=2e-3) annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-20,-12})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=1e-6)
    annotation (Placement(transformation(extent={{-40,-10},{-60,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-82,-20},{-62,0}})));
  Modelica.Electrical.Analog.Basic.Resistor DC_Load_Lamp(R=1)
    "Load lamp for the distribution network" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,-16})));
  Modelica.Electrical.Analog.Basic.Resistor DC_Load_Heater(R=1)
    "Load heater lamp for the distribution network" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={42,-16})));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_SeriesExcited dcse(
    VaNominal=VaNominal,
    IaNominal=IaNominal,
    TaNominal=TaNominal,
    Ra=Ra,
    TaRef=TaRef,
    alpha20a(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
    La=La,
    Re=Re,
    TeRef=TeRef,
    Le=Le,
    sigmae=sigmae,
    TeNominal=TeNominal)
    annotation (Placement(transformation(extent={{70,-30},{90,-10}})));

  parameter Modelica.Units.SI.Resistance Ra=0.35 "Armature resistance at TRef";
  parameter Modelica.Units.SI.Temperature TaRef=65
    "Reference temperature of armature resistance";
  parameter Modelica.Units.SI.Inductance La=0.0001 "Armature inductance";
  parameter Modelica.Units.SI.Voltage VaNominal=400 "Nominal armature voltage";
  parameter Modelica.Units.SI.Current IaNominal=1
    "Nominal armature current (>0..Motor, <0..Generator)";
  parameter Modelica.Units.SI.Temperature TaNominal=60
    "Nominal armature temperature";
  parameter Modelica.Units.SI.Temperature TeNominal=65
    "Nominal series excitation temperature";
  parameter Modelica.Units.SI.Resistance Re
    "Series excitation resistance at TRef";
  parameter Modelica.Units.SI.Temperature TeRef
    "Reference temperature of excitation resistance";
  parameter Modelica.Units.SI.Inductance Le "Total field excitation inductance";
  parameter Real sigmae "Stray fraction of total excitation inductance";
  Mechanical.Pumps.Boeing747_Pump_Load boeing747_Pump_Load(k=1.5/200)
    annotation (Placement(transformation(extent={{122,-24},{102,-16}})));
  parameter CHEETA.Records.Boeing747electricalModel.SynchronousMachine.SM100kVA
                                                                         Data
    annotation (Placement(transformation(extent={{40,22},{60,42}})));
equation
  connect(aCDC.positivePlug, Bus) annotation (Line(points={{-61.5714,-30},{-92,
          -30},{-92,0},{-104,0}}, color={0,0,255}));
  connect(aCDC1.positivePlug, Bus) annotation (Line(points={{-61.5714,30},{-92,
          30},{-92,0},{-104,0}}, color={0,0,255}));
  connect(aCDC1.pin_p, inductor.p) annotation (Line(points={{-36.4286,35},{-20,
          35},{-20,26}}, color={0,0,255}));
  connect(aCDC.pin_p, inductor1.p) annotation (Line(points={{-36.4286,-25},{-20,
          -25},{-20,-22}}, color={0,0,255}));
  connect(capacitor.p, inductor1.n)
    annotation (Line(points={{-40,0},{-20,0},{-20,-2}}, color={0,0,255}));
  connect(inductor.n, inductor1.n)
    annotation (Line(points={{-20,6},{-20,-2}}, color={0,0,255}));
  connect(capacitor.n, ground.p)
    annotation (Line(points={{-60,0},{-72,0}}, color={0,0,255}));
  connect(aCDC1.pin_n, aCDC.pin_n) annotation (Line(points={{-36.4286,25},{-30,
          25},{-30,-35},{-36.4286,-35}}, color={0,0,255}));
  connect(DC_Load_Lamp.p, inductor1.n) annotation (Line(points={{20,-6},{20,0},
          {-20,0},{-20,-2}}, color={0,0,255}));
  connect(DC_Load_Lamp.n, aCDC.pin_n) annotation (Line(points={{20,-26},{20,-34},
          {-30,-34},{-30,-35},{-36.4286,-35}}, color={0,0,255}));
  connect(DC_Load_Heater.p, inductor1.n) annotation (Line(points={{42,-6},{42,0},
          {-20,0},{-20,-2}}, color={0,0,255}));
  connect(DC_Load_Heater.n, aCDC.pin_n) annotation (Line(points={{42,-26},{42,
          -34},{-30,-34},{-30,-35},{-36.4286,-35}}, color={0,0,255}));
  connect(dcse.pin_ep, inductor1.n) annotation (Line(points={{70,-14},{70,0},{
          -20,0},{-20,-2}}, color={0,0,255}));
  connect(dcse.pin_ap, inductor1.n) annotation (Line(points={{86,-10},{86,0},{
          -20,0},{-20,-2}}, color={0,0,255}));
  connect(dcse.pin_en, aCDC.pin_n) annotation (Line(points={{70,-26},{70,-34},{
          -30,-34},{-30,-35},{-36.4286,-35}}, color={0,0,255}));
  connect(dcse.pin_an, aCDC.pin_n) annotation (Line(points={{74,-10},{74,-34},{
          -30,-34},{-30,-35},{-36.4286,-35}}, color={0,0,255}));
  connect(dcse.flange, boeing747_Pump_Load.flange)
    annotation (Line(points={{90,-20},{101,-20}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -60},{140,60}})), Diagram(coordinateSystem(preserveAspectRatio=
            false, extent={{-100,-60},{140,60}})));
end Boeing747_BLDC_Motor_pump;
