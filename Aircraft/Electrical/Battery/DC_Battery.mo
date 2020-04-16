within CHEETA.Aircraft.Electrical.Battery;
model DC_Battery
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=0,
        origin={-11,27})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1 annotation (Placement(
        transformation(extent={{-62,90},{-42,110}}),
                                                   iconTransformation(extent={{-62,90},
            {-42,110}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 annotation (Placement(
        transformation(extent={{38,90},{58,110}}),   iconTransformation(extent={{38,90},
            {58,110}})));
  parameter DymolaModels.Types.StateOfCharge startSOC=1
    "State of Charge at initialization of the simulation";
  ElectrifiedPowertrains.SupplySystem.Batteries.Packs.DC batteryPack(redeclare
      ElectrifiedPowertrains.SupplySystem.Batteries.Packs.Records.Data.DC.Enerdel15Ah_96s5p
      data, startSOC = 1.0)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,34})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Interfaces.Bus batteryBus1
    annotation (Placement(transformation(extent={{-52,-90},{-32,-70}})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Packs.DC batteryPack1(
      redeclare
      ElectrifiedPowertrains.SupplySystem.Batteries.Packs.Records.Data.DC.Enerdel15Ah_96s5p
      data, startSOC = 1.0)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={42,34})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Interfaces.Bus batteryBus2
    annotation (Placement(transformation(extent={{30,-90},{50,-70}})));
equation
  connect(batteryPack.p, p1) annotation (Line(points={{-46,44},{-50,44},{-50,100},
          {-52,100}}, color={0,0,255}));
  connect(batteryPack.n, ground.p)
    annotation (Line(points={{-34,44},{-11,44},{-11,36}}, color={0,0,255}));
  connect(batteryPack.batteryBus, batteryBus1) annotation (Line(
      points={{-30,34},{-28,34},{-28,-80},{-42,-80}},
      color={0,255,0},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(batteryPack1.n, n1)
    annotation (Line(points={{48,44},{48,100}}, color={0,0,255}));
  connect(batteryPack1.p, ground.p)
    annotation (Line(points={{36,44},{-11,44},{-11,36}}, color={0,0,255}));
  connect(batteryPack1.batteryBus, batteryBus2) annotation (Line(
      points={{52,34},{60,34},{60,-80},{40,-80}},
      color={0,255,0},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,100}}), graphics={
        Rectangle(
          extent={{-82,70},{78,-70}},
          lineColor={95,95,95},
          fillPattern=FillPattern.Solid,
          fillColor={215,215,215}),
        Line(
          points={{-52,100},{-52,70}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{48,100},{48,70}},
          color={95,95,95},
          smooth=Smooth.None),
        Text(
          extent={{-52,64},{-92,104}},
          lineColor={95,95,95},
          textString="+"),
        Text(
          extent={{90,66},{50,106}},
          lineColor={95,95,95},
          textString="-"),
        Line(
          points={{-62,-48},{-62,32}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-42,-48},{-42,32}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-22,-48},{-22,32}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-2,-48},{-2,32}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{18,-48},{18,32}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{38,-48},{38,32}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{58,-48},{58,32}},
          color={95,95,95},
          smooth=Smooth.None)}), Diagram(coordinateSystem(preserveAspectRatio=
            false, extent={{-100,-80},{100,100}})));
end DC_Battery;
