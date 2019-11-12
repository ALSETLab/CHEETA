within CHEETA.Aircraft.Electrical.Machines;
model Boeing747_MotorDrive
  "Permanent magnet synchronous motor drive for Boeing 747"
  Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
    smpm annotation (Placement(transformation(extent={{-10,-12},{10,8}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-10,4},{10,24}})));
equation
  connect(terminalBox.plug_sn, smpm.plug_sn)
    annotation (Line(points={{-6,8},{-6,8}}, color={0,0,255}));
  connect(terminalBox.plug_sp, smpm.plug_sp)
    annotation (Line(points={{6,8},{6,8}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Boeing747_MotorDrive;
