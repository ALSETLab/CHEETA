within CHEETA;
package Types "Collection of special types used in OpenIPSL"
extends Modelica.Icons.TypesPackage;

  type ActivePowerMega = Real (final quantity="Power", final unit="MW")
    annotation (Documentation);

  type ApparentPowerMega = Real (final quantity="Power", final unit="MVA")
    annotation (Documentation);

  type ReactivePowerMega = Real (final quantity="Power", final unit="Mvar")
    annotation (Documentation);

  type VoltageKilo = Real (final quantity="ElectricPotential", final unit="kV")
    annotation (Documentation);

annotation (Documentation);
end Types;
