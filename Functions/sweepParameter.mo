within CHEETA.Functions;
function sweepParameter

  input String pathToModel=
    "FinalProject.Experiments.FlightManeuver";
  input String parameterToSweep = "aircraft.span";
  input Integer numSimulations = 3 "Number of simulations";
  input Real startValue = Modelon.Units.Conversions.from_ft(90);
  input Real endValue= Modelon.Units.Conversions.from_ft(110);

protected
  Real[:] parameterValues;

algorithm
  parameterValues := linspace(startValue, endValue, numSimulations);
  translateModel(pathToModel);
  //DymolaCommands.Plot.createPlot();
  for i in 1:numSimulations loop
    simulateExtendedModel(pathToModel, stopTime=100, method="dassl",
                          resultFile= parameterToSweep,
                          initialNames={parameterToSweep},
                          initialValues={parameterValues[i]},
                          finalNames={parameterToSweep});

  end for;

end sweepParameter;
