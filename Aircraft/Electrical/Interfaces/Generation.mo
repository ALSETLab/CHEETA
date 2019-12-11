within CHEETA.Aircraft.Electrical.Interfaces;
partial model Generation
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug AC_out annotation (
      Placement(transformation(rotation=0, extent={{-232,-8},{-212,12}})));
  Modelica.Blocks.Interfaces.RealInput w_ref(unit="rad/s") annotation (
      Placement(transformation(rotation=0, extent={{120,-10},{100,10}}),
        iconTransformation(extent={{120,-10},{100,10}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -220,-100},{100,100}})),                             Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-220,-100},{
            100,100}})));
end Generation;
