within CHEETA.Aircraft.Electrical.Battery;
model DC_Battery
  replaceable
  ElectrifiedPowertrains.SupplySystem.Batteries.Packs.RC1 packRC1(startSOC=
        startSOC, redeclare
      ElectrifiedPowertrains.SupplySystem.Batteries.Packs.Records.Data.RC1.EIG20Ah_96s2p
      data) constrainedby
    ElectrifiedPowertrains.SupplySystem.Batteries.Packs.DC(startSOC=startSOC)
            annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-44,30})), __Dymola_choicesAllMatching=true);
  replaceable ElectrifiedPowertrains.SupplySystem.Batteries.Packs.RC1 packRC2(
      startSOC=startSOC, redeclare
      ElectrifiedPowertrains.SupplySystem.Batteries.Packs.Records.Data.RC1.EIG20Ah_96s2p
      data) constrainedby
    ElectrifiedPowertrains.SupplySystem.Batteries.Packs.DC(startSOC=startSOC)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={26,30})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        extent={{-9,-9},{9,9}},
        rotation=0,
        origin={-11,27})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1 annotation (Placement(
        transformation(extent={{38,92},{58,112}}), iconTransformation(extent={{
            38,92},{58,112}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 annotation (Placement(
        transformation(extent={{-62,92},{-42,112}}), iconTransformation(extent=
            {{-62,92},{-42,112}})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Interfaces.Bus batteryBus1
    annotation (Placement(transformation(extent={{-46,-80},{-26,-60}}),
        iconTransformation(extent={{-46,-80},{-26,-60}})));
  ElectrifiedPowertrains.SupplySystem.Batteries.Interfaces.Bus batteryBus2
    annotation (Placement(transformation(extent={{30,-80},{50,-60}}),
        iconTransformation(extent={{30,-80},{50,-60}})));
  parameter DymolaModels.Types.StateOfCharge startSOC=1
    "State of Charge at initialization of the simulation";
equation
  connect(packRC1.n, packRC2.p)
    annotation (Line(points={{-38,40},{20,40}}, color={0,0,255}));
  connect(packRC1.n, ground.p)
    annotation (Line(points={{-38,40},{-11,40},{-11,36}}, color={0,0,255}));
  connect(packRC1.batteryBus, batteryBus1) annotation (Line(
      points={{-34,30},{-24,30},{-24,-50},{-36,-50},{-36,-70}},
      color={0,255,0},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(packRC2.batteryBus, batteryBus2) annotation (Line(
      points={{36,30},{44,30},{44,-70},{40,-70}},
      color={0,255,0},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(packRC1.p, p1) annotation (Line(points={{-50,40},{-46,40},{-46,84},{
          48,84},{48,102}}, color={0,0,255}));
  connect(packRC2.n, n1) annotation (Line(points={{32,40},{32,92},{-52,92},{-52,
          102}}, color={0,0,255}));
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
          extent={{88,70},{48,110}},
          lineColor={95,95,95},
          textString="+"),
        Text(
          extent={{-52,70},{-92,110}},
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
