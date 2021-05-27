within CHEETA.Aircraft.Electrical.BusBar;
package Examples
  model Cu_CurrentLead
    CHEETA.Aircraft.Electrical.BusBar.Cu_CurrentLead cu_CurrentLead(I_0=3000, A
        =0.1) annotation (Placement(transformation(extent={{24,-8},{36,10}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=10)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,0})));
  equation
    connect(cu_CurrentLead.n1, ground.p)
      annotation (Line(points={{30,-9.5},{30,-26}}, color={0,0,255}));
    connect(constantVoltage.p, cu_CurrentLead.p1) annotation (Line(points={{
            1.77636e-15,10},{0,10},{0,30},{30,30},{30,11.5}}, color={0,0,255}));
    connect(constantVoltage.n, ground.p) annotation (Line(points={{-1.77636e-15,
            -10},{0,-10},{0,-22},{24,-22},{24,-20},{30,-20},{30,-26}}, color={0,
            0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Cu_CurrentLead;

  model BusBar
    Modelica.Electrical.Analog.Basic.Ground ground1
      annotation (Placement(transformation(extent={{38,-56},{58,-36}})));
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
    Al_Bar al_Bar
      annotation (Placement(transformation(extent={{28,-20},{38,10}})));
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
            -16},{50,-36},{48,-36}},
                                color={0,0,255}));
    connect(inverter1.ac,resistor4. p)
      annotation (Line(points={{82,-10},{90,-10}}, color={0,0,255}));
    connect(inductor1.p,resistor4. p) annotation (Line(points={{90,16},{86,16},
            {86,-10},{90,-10}},
                            color={0,0,255}));
    connect(inverter1.dc_p, al_Bar.Machine) annotation (Line(points={{62,-4},{
            40,-4},{40,-5},{39.25,-5}}, color={0,0,255}));
    connect(inductor1.n, resistor4.n) annotation (Line(points={{110,16},{114,16},
            {114,-10},{110,-10}}, color={0,0,255}));
    connect(ground1.p, resistor4.n) annotation (Line(points={{48,-36},{64,-36},
            {64,-38},{114,-38},{114,-10},{110,-10}}, color={0,0,255}));
    connect(prescribedTemperature3.port, al_Bar.port_a1) annotation (Line(
          points={{10,38},{-4,38},{-4,-5},{27.5,-5}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
              -80},{100,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-140,-80},{100,60}})));
  end BusBar;
end Examples;
