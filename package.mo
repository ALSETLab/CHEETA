within ;






































































































package CHEETA
annotation (uses(
    OpenIPSL(version="2.0.0-dev"),
    ElectricPower(version="2.7"),
    FuelCell(version="1.10"),
    Electrification(version="1.1"),
    AircraftDynamics(version="1.1"),
    Modelica(version="4.0.0"),
    Complex(version="4.0.0"),
    Modelica_LinearSystems2(version="2.4.0"),
    ElectrifiedPowertrains(version="1.3.3"),
    VeSyMA(version="2020.3"),
    DymolaModels(version="1.2"),
    Battery(version="2.2.0"),
    Hydrogen(version="1.3.3"),
    DassaultSystemes(version="1.5"),
    Modelon(version="3.3"),
    BrushlessDCDrives(version="1.1.2")),
  version="5",
  conversion(from(
      version="3",
      script="ConvertFromCHEETA_3.mos",
      version="",
      version="2")));
end CHEETA;

