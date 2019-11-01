within CHEETA.Aircraft.Airframes;
package Interfaces

  partial model Frames
    "Frame connectors to connector for fixed wing aircraft components to each other (airframe, power, systems). "



    Modelica.Mechanics.MultiBody.Interfaces.Frame_b noseTip
                                                           "Nose tip frame" annotation (Placement(
          transformation(
          extent={{-16,-16},{16,16}},
          rotation=180,
          origin={-100,0}), iconTransformation(
          extent={{-8,-8},{8,8}},
          rotation=180,
          origin={-100,0})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine5Frame
                                                                "Left aft-fuselage engine frame"
      annotation (Placement(transformation(
          extent={{-16,-16},{16,16}},
          rotation=90,
          origin={30,-100}), iconTransformation(
          extent={{-8,-8},{8,8}},
          rotation=90,
          origin={50,-100})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine6Frame
                                                                "Right aft-fuselage engine frame"
      annotation (Placement(transformation(
          extent={{-16,-16},{16,16}},
          rotation=90,
          origin={30,100}), iconTransformation(
          extent={{-8,-8},{8,8}},
          rotation=270,
          origin={50,100})));

    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine4Frame
                                                                "Engine 4 frame" annotation (
        Placement(transformation(extent={{-116,64},{-84,96}}), iconTransformation(extent={{-108,72},{-92,
              88}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine3Frame
                                                                "Engine 3 frame" annotation (
        Placement(transformation(extent={{-116,24},{-84,56}}), iconTransformation(extent={{-108,32},{
              -92,48}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine2Frame
                                                                "Engine 2 frame" annotation (
        Placement(transformation(extent={{-116,-56},{-84,-24}}), iconTransformation(extent={{-108,-48},
              {-92,-32}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine1Frame
                                                                "Engine 1 frame" annotation (
        Placement(transformation(extent={{-116,-96},{-84,-64}}), iconTransformation(extent={{-108,-88},
              {-92,-72}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b fwdBay
                                                          "Forward equipment bay" annotation (
        Placement(transformation(
          extent={{-16,-16},{16,16}},
          rotation=90,
          origin={-70,-100}), iconTransformation(
          extent={{-8,-8},{8,8}},
          rotation=90,
          origin={-70,-100})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b tailConeTip
                                                               "Tip of tail cone" annotation (
        Placement(transformation(
          extent={{-16,-16},{16,16}},
          rotation=180,
          origin={100,0}), iconTransformation(
          extent={{-8,-8},{8,8}},
          rotation=180,
          origin={100,0})));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}})),
                Diagram(coordinateSystem(
            preserveAspectRatio=false)),
      Documentation(revisions="<html>
<hr><p><font class=\"copyright_bold\">Copyright &copy; 2004-2019, MODELON AB</font> <font class=\"copyright_base\">The use of this software component is regulated by the licensing conditions for the Aircraft Dynamics Library. This copyright notice must, unaltered, accompany all components that are derived from, copied from, or by other means have their origin from the Aircraft Dynamics Library. </font></p>
</html>", info="<html>
<p>This is the frame for physically coupling all the power system, airframe, and other components together.</p>
</html>"));
  end Frames;

annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Polygon(origin={20.0,0.0},
          lineColor={64,64,64},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-10.0,70.0},{10.0,70.0},{40.0,20.0},{80.0,20.0},{80.0,-20.0},{40.0,-20.0},{10.0,-70.0},{-10.0,-70.0}}),
        Polygon(fillColor={102,102,102},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-100.0,20.0},{-60.0,20.0},{-30.0,70.0},{-10.0,70.0},{-10.0,-70.0},{-30.0,-70.0},{-60.0,-20.0},{-100.0,-20.0}})}));
end Interfaces;
