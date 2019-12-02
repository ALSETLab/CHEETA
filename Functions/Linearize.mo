within CHEETA.Functions;
function Linearize
  "Linearize a model and return the linearized model as StateSpace object"
  extends Modelica_LinearSystems2.Internal.PartialAnalyzeFunction;
public
  output Modelica_LinearSystems2.StateSpace ss = ssLin
    "Linearized system as StateSpace object";
algorithm
    // PRINT linear system
  Modelica.Utilities.Streams.print(String(ss));
  // SAVE the data in a mat file
  DataFiles.writeMATmatrix(
    "MyData.mat",
    "ABCD",
    [ss.A, ss.B; ss.C, ss.D],
    append=false);

  annotation(__Dymola_interactive=true, Icon(graphics={
          Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={255,127,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-80,80},{80,-80}},
          lineColor={255,127,0},
          textString="L")}));
end Linearize;
