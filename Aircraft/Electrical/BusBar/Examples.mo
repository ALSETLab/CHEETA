within CHEETA.Aircraft.Electrical.BusBar;
package Examples
  model Cu_CurrentLead
    CHEETA.Aircraft.Electrical.BusBar.Cu_CurrentLead cu_CurrentLead(
      l=0.5,
      I_0=3000,
      A=0.1) annotation (Placement(transformation(extent={{24,-14},{36,4}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{20,-48},{40,-28}})));
    Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage(V=10, duration=
          10) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={50,-4})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature3
      annotation (Placement(transformation(extent={{12,26},{-8,46}})));
    Modelica.Blocks.Sources.Constant const3(k=20)
      annotation (Placement(transformation(extent={{48,26},{28,46}})));
  equation
    connect(cu_CurrentLead.n1, ground.p)
      annotation (Line(points={{30,-15.5},{30,-28}}, color={0,0,255}));
    connect(rampVoltage.p, cu_CurrentLead.p1) annotation (Line(points={{50,6},{
            50,16},{30,16},{30,5.5}}, color={0,0,255}));
    connect(rampVoltage.n, ground.p) annotation (Line(points={{50,-14},{50,-22},
            {30,-22},{30,-28}}, color={0,0,255}));
    connect(const3.y, prescribedTemperature3.T)
      annotation (Line(points={{27,36},{14,36}}, color={0,0,127}));
    connect(prescribedTemperature3.port, cu_CurrentLead.port_a) annotation (
        Line(points={{-8,36},{-16,36},{-16,-5},{24,-5}}, color={191,0,0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=10, __Dymola_Algorithm="Rkfix4"));
  end Cu_CurrentLead;

  model BusBar
    Modelica.Electrical.Analog.Basic.Ground ground1
      annotation (Placement(transformation(extent={{40,-58},{60,-38}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature3
      annotation (Placement(transformation(extent={{30,28},{10,48}})));
    Modelica.Blocks.Sources.Ramp     ramp(
      height=150,
      duration=10,
      offset=20)
      annotation (Placement(transformation(extent={{76,28},{56,48}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature4
      annotation (Placement(transformation(extent={{-102,-28},{-82,-8}})));
    Modelica.Blocks.Sources.Constant const4(k=353.15)
      annotation (Placement(transformation(extent={{-136,-28},{-116,-8}})));
    Al_Bar al_Bar(
      l=0.1,
      k_0=0.182,
      h_L=558e3,
      c_p0=14.31e3,
      A=2) annotation (Placement(transformation(extent={{28,-20},{38,10}})));
    Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch switch
      annotation (Placement(transformation(extent={{-24,6},{-4,-14}})));
    Modelica.Blocks.Sources.BooleanConstant booleanConstant
      annotation (Placement(transformation(extent={{-48,-52},{-28,-32}})));
    FuelCell.FuelCell_EquationBased_DetailedRohm
      fuelCell_EquationBased_DetailedRohm(R_ohm0=0.02)
      annotation (Placement(transformation(extent={{-84,10},{-66,28}})));
    Battery.Battery_BMS
                battery_BMS annotation (Placement(transformation(
          extent={{-8,-7},{8,7}},
          rotation=270,
          origin={-60,-15})));
    Modelica.Blocks.Sources.BooleanExpression booleanExpression
      annotation (Placement(transformation(extent={{-102,-52},{-82,-32}})));
    PowerElectronics.Converters.DCAC.TheveninEquivalent inverter1(R=0.1, R_cm=
          0.1)
      annotation (Placement(transformation(extent={{62,-20},{82,0}})));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R=100)
      annotation (Placement(transformation(extent={{90,-20},{110,0}})));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L=0.12086)
      annotation (Placement(transformation(extent={{90,6},{110,26}})));
  equation
    connect(ramp.y, prescribedTemperature3.T)
      annotation (Line(points={{55,38},{32,38}}, color={0,0,127}));
    connect(const4.y,prescribedTemperature4. T)
      annotation (Line(points={{-115,-18},{-104,-18}},
                                                 color={0,0,127}));
    connect(switch.n, al_Bar.Battery) annotation (Line(points={{-4,-4},{-4,-11},
            {26.75,-11}}, color={0,0,255}));
    connect(switch.control, booleanConstant.y) annotation (Line(points={{-14,
            -16},{-14,-42},{-27,-42}}, color={255,0,255}));
    connect(prescribedTemperature4.port, fuelCell_EquationBased_DetailedRohm.port_a)
      annotation (Line(points={{-82,-18},{-75,-18},{-75,10}}, color={191,0,0}));
    connect(booleanExpression.y,battery_BMS. u1)
      annotation (Line(points={{-81,-42},{-59.125,-42},{-59.125,-21.4}},
                                                             color={255,0,255}));
    connect(battery_BMS.p1, switch.p) annotation (Line(points={{-50.55,-9.4},{
            -50.55,-4},{-24,-4}}, color={0,0,255}));
    connect(fuelCell_EquationBased_DetailedRohm.p1, al_Bar.FuelCell)
      annotation (Line(points={{-66,19.9},{12,19.9},{12,1},{26.75,1}}, color={0,
            0,255}));
    connect(inverter1.dc_n, ground1.p) annotation (Line(points={{62,-16},{50,
            -16},{50,-38}},     color={0,0,255}));
    connect(inverter1.ac,resistor4. p)
      annotation (Line(points={{82,-10},{90,-10}}, color={0,0,255}));
    connect(inductor1.p,resistor4. p) annotation (Line(points={{90,16},{86,16},
            {86,-10},{90,-10}},
                            color={0,0,255}));
    connect(inverter1.dc_p, al_Bar.Machine) annotation (Line(points={{62,-4},{
            40,-4},{40,-5},{39.25,-5}}, color={0,0,255}));
    connect(inductor1.n, resistor4.n) annotation (Line(points={{110,16},{114,16},
            {114,-10},{110,-10}}, color={0,0,255}));
    connect(ground1.p, resistor4.n) annotation (Line(points={{50,-38},{114,-38},
            {114,-10},{110,-10}}, color={0,0,255}));
    connect(prescribedTemperature3.port, al_Bar.port_a1) annotation (Line(
          points={{10,38},{0,38},{0,-5},{27.5,-5}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
              -60},{120,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-140,-60},{120,60}})));
  end BusBar;

  model BusBar_Fault
    Modelica.Electrical.Analog.Basic.Ground ground1
      annotation (Placement(transformation(extent={{112,-74},{132,-54}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature3
      annotation (Placement(transformation(extent={{22,22},{2,42}})));
    Modelica.Blocks.Sources.Constant const3(k=20)
      annotation (Placement(transformation(extent={{68,22},{48,42}})));
    Modelica.Electrical.Analog.Basic.VariableResistor
                                              resistor1
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=270,
          origin={94,-36})));
    Modelica.Blocks.Sources.Ramp     ramp1(
      height=-10000,
      duration=0.5,
      offset=10000,
      startTime=10)
      annotation (Placement(transformation(extent={{44,-46},{64,-26}})));
    PowerElectronics.Converters.DCAC.TheveninEquivalent inverter1(R=0.1, R_cm=
          0.1)
      annotation (Placement(transformation(extent={{118,-46},{138,-26}})));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R=100)
      annotation (Placement(transformation(extent={{146,-46},{166,-26}})));
    FuelCell.FuelCell_EquationBased
      fuelCell_EquationBased1(R_ohm_current=0.25)
      annotation (Placement(transformation(extent={{-70,-6},{-56,8}})));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L=0.12086)
      annotation (Placement(transformation(extent={{146,-24},{166,-4}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature4
      annotation (Placement(transformation(extent={{-102,-46},{-82,-26}})));
    Modelica.Blocks.Sources.Constant const4(k=353.15)
      annotation (Placement(transformation(extent={{-136,-46},{-116,-26}})));
    HTS.LiquidCooled.HTS_filmboiling_Voltage_Hydrogen
                                     HTS1(
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
      annotation (Placement(transformation(extent={{-34,18},{-18,10}})));
    Al_Bar al_Bar(
      l=0.1,
      k_0=0.182,
      h_L=558e3,
      c_p0=14.31e3,
      A=2) annotation (Placement(transformation(extent={{70,-24},{80,6}})));
    Battery.Battery_BMS
                battery_BMS annotation (Placement(transformation(
          extent={{-8,-7},{8,7}},
          rotation=270,
          origin={-38,-23})));
    Modelica.Blocks.Sources.BooleanConstant booleanConstant
      annotation (Placement(transformation(extent={{-6,-56},{14,-36}})));
    Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch switch
      annotation (Placement(transformation(extent={{14,-6},{34,-26}})));
    Modelica.Blocks.Sources.BooleanExpression booleanExpression
      annotation (Placement(transformation(extent={{-70,-60},{-50,-40}})));
  equation
    connect(const3.y,prescribedTemperature3. T)
      annotation (Line(points={{47,32},{24,32}}, color={0,0,127}));
    connect(resistor1.n,ground1. p)
      annotation (Line(points={{94,-46},{94,-54},{122,-54}},  color={0,0,255}));
    connect(resistor1.R,ramp1. y)
      annotation (Line(points={{82,-36},{65,-36}},   color={0,0,127}));
    connect(inverter1.dc_n,ground1. p) annotation (Line(points={{118,-42},{106,
            -42},{106,-54},{122,-54}},
                                  color={0,0,255}));
    connect(inverter1.ac,resistor4. p)
      annotation (Line(points={{138,-36},{146,-36}}, color={0,0,255}));
    connect(resistor4.n,ground1. p) annotation (Line(points={{166,-36},{170,-36},
            {170,-54},{122,-54}},color={0,0,255}));
    connect(inductor1.p,resistor4. p) annotation (Line(points={{146,-14},{142,
            -14},{142,-36},{146,-36}},
                                  color={0,0,255}));
    connect(inductor1.n,ground1. p) annotation (Line(points={{166,-14},{170,-14},
            {170,-54},{122,-54}},color={0,0,255}));
    connect(const4.y,prescribedTemperature4. T)
      annotation (Line(points={{-115,-36},{-104,-36}},
                                                 color={0,0,127}));
    connect(prescribedTemperature4.port,fuelCell_EquationBased1. port_a)
      annotation (Line(points={{-82,-36},{-63,-36},{-63,-6}},     color={191,0,0}));
    connect(inverter1.dc_p,resistor1. p) annotation (Line(points={{118,-30},{
            104,-30},{104,-12},{94,-12},{94,-26}},color={0,0,255}));
    connect(HTS1.port_a,prescribedTemperature3. port) annotation (Line(points={{-25.8,
            18},{-26,18},{-26,32},{2,32}},       color={191,0,0}));
    connect(fuelCell_EquationBased1.p1,HTS1. pin_p) annotation (Line(points={{-56,1.7},
            {-46,1.7},{-46,14},{-35,14}},      color={0,0,255}));
    connect(al_Bar.FuelCell, HTS1.pin_n) annotation (Line(points={{68.75,-3},{
            -10,-3},{-10,14},{-17,14}}, color={0,0,255}));
    connect(al_Bar.Machine, resistor1.p)
      annotation (Line(points={{81.25,-9},{94,-9},{94,-26}}, color={0,0,255}));
    connect(al_Bar.Battery, switch.n) annotation (Line(points={{68.75,-15},{
            68.75,-16},{34,-16}}, color={0,0,255}));
    connect(switch.control, booleanConstant.y) annotation (Line(points={{24,-28},
            {24,-46},{15,-46}}, color={255,0,255}));
    connect(switch.p, battery_BMS.p1) annotation (Line(points={{14,-16},{14,
            -17.4},{-28.55,-17.4}}, color={0,0,255}));
    connect(battery_BMS.u1, booleanExpression.y) annotation (Line(points={{
            -37.125,-29.4},{-38,-29.4},{-38,-50},{-49,-50}}, color={255,0,255}));
    connect(al_Bar.port_a1, prescribedTemperature3.port) annotation (Line(
          points={{70,-9},{-4,-9},{-4,32},{2,32}}, color={191,0,0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-80},{180,
              60}})),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-80},{
              180,60}})),
      experiment(StopTime=30, __Dymola_Algorithm="Dassl"));
  end BusBar_Fault;
end Examples;
