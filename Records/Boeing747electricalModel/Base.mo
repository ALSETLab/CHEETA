within CHEETA.Records.Boeing747electricalModel;
package Base
  record DC_Motor_Data
    extends Modelica.Icons.Record;
    parameter Modelica.SIunits.Resistance  Ra "Armature Resistance";
    parameter Modelica.SIunits.Inductance  La "Armature Inductance";
    parameter Modelica.SIunits.Resistance  Rf "Field Resistance";
    parameter Modelica.SIunits.Inductance  Lf "Field Inductance";
    parameter Modelica.SIunits.Inductance  Laf "Field- armature Mutual Inductance";
    parameter Modelica.SIunits.Inertia J "Inertia";
  end DC_Motor_Data;

  record Synch
    extends Modelica.Icons.Record;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Synch;

  record AVR
    extends Modelica.Icons.Record
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end AVR;

  record IM
    extends Modelica.Icons.Record;
    parameter Integer p "Number of Poles";
    parameter Real fsNominal "Nominal Operating Frequency";
    parameter Real J "Rotor Inertia";
    parameter Real Rs "Stator Resistance";
    parameter Real Lss "Stray Inductance per phase";
    parameter Real Lm "Stator main field inductance per pahse";
    parameter Real Lr "Rotor stray inductance";
    parameter Real Rr "Rotor resistance per phase";
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));

  end IM;
  annotation (Documentation(info="<html>
<p>Base Classes to Make Easy Replaceable Records</p>
</html>"));
end Base;
