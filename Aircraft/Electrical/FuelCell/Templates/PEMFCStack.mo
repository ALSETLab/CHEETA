within CHEETA.Aircraft.Electrical.FuelCell.Templates;
model PEMFCStack
  import FuelCell;
  extends FuelCell.Stacks.Templates.CoolStack(subStack(
      redeclare model Membrane = FuelCell.Membranes.PEMFC.PEMFCSimplified,
      addProxToAnode=true,
      L_anode=0.15,
      D_anode=0.08,
      L_cathode=0.15,
      D_cathode=0.08,
      d0_prox=500,
      m_flow_nom_prox=0.001));
end PEMFCStack;
