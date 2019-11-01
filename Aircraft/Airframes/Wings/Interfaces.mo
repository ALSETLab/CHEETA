within CHEETA.Aircraft.Airframes.Wings;
package Interfaces

  partial model Wing


      parameter Integer nMlgFrames(start = 2)
      "Number of individual, basic wheeled units (single-wheel units or bogies composed of multiple wheels)";

    Modelica.Mechanics.MultiBody.Interfaces.Frame_b wingBoxLe "Wing box leading edge" annotation (
        Placement(transformation(extent={{-116,-16},{-84,16}}), iconTransformation(extent={{-57,-8},{
              -41,8}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine1Frame "Engine 1 frame" annotation (
        Placement(transformation(extent={{-116,-96},{-84,-64}}), iconTransformation(extent={{-57,-88},
              {-41,-72}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine2Frame "Engine 2 frame" annotation (
        Placement(transformation(extent={{-116,-56},{-84,-24}}), iconTransformation(extent={{-57,-48},
              {-41,-32}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine3Frame "Engine 3 frame" annotation (
        Placement(transformation(extent={{-116,24},{-84,56}}), iconTransformation(extent={{-57,32},{-41,
              48}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b engine4Frame "Engine 4 frame" annotation (
        Placement(transformation(extent={{-116,64},{-84,96}}), iconTransformation(extent={{-57,72},{
              -41,88}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b wingBoxTe "Wing box trainling edge" annotation (
        Placement(transformation(extent={{84,-16},{116,16}}), iconTransformation(extent={{41,-8},{57,
              8}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b mlgFrame[nMlgFrames]
      "Main landing gear frames (ordered left to right)"                                                                   annotation (
        Placement(transformation(extent={{84,14},{116,46}}), iconTransformation(extent={{41,14},{57,
              30}})));
    outer Modelica.Mechanics.MultiBody.World world;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>This is the framework for developing an airplane wing.</p>
</html>"));
  end Wing;
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
