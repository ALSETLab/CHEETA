within CHEETA.Aircraft.Electrical.FuelCell;
model SOFCStack
  "Full SOFC stack model with manifolds and endplates (no instulation)"
  import FuelCell;
  extends FuelCell.Stacks.Templates.FullStack(
    redeclare model SubStack = FuelCell.Stacks.SOFC.SOFCSubStack,
    A_cell=361e-4,
    Cp_cell=500,
    kc_cell=250,
    redeclare model Feed_Flow_an = FuelCell.Manifolds.Feed.External,
    redeclare model Feed_Flow_cath = FuelCell.Manifolds.Feed.External,
    redeclare model Drain_Flow_an = FuelCell.Manifolds.Drain.External,
    redeclare model Drain_Flow_cath = FuelCell.Manifolds.Drain.External,
    feed_Anode(redeclare model Friction =
          Modelon.ThermoFluid.FlowResistances.FrictionModels.LinearOperatingPointLoss
          (
          d0=0.5,
          m_flow0=m_flow_nom_an/nbrSubStacks,
          dp0(displayUnit="kPa") = 500)),
    drain_Anode(redeclare model Friction =
          Modelon.ThermoFluid.FlowResistances.FrictionModels.LinearOperatingPointLoss
          (
          d0=0.5,
          m_flow0=m_flow_nom_an,
          dp0(displayUnit="kPa") = 100)),
    feed_Cathode(redeclare model Friction =
          Modelon.ThermoFluid.FlowResistances.FrictionModels.LinearOperatingPointLoss
          (
          d0=0.5,
          m_flow0=m_flow_nom_cath/nbrSubStacks,
          dp0(displayUnit="kPa") = 500)),
    drain_Cathode(redeclare model Friction =
          Modelon.ThermoFluid.FlowResistances.FrictionModels.LinearOperatingPointLoss
          (
          d0=0.5,
          m_flow0=m_flow_nom_cath,
          dp0(displayUnit="kPa") = 100)),
    redeclare model StackHeatLoss = FuelCell.HeatTransfer.SHLWithoutInsulation);

  annotation (Documentation(revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2019, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>", info="<html>
<h4>SOFCStack</h4>
<p>Predefined solid oxide fuel cell (SOFC) stack. Extends from <a href=\"modelica://FuelCell.Stacks.Templates.FullStack\">FuelCell.Stacks.Templates.FullStack</a>.
The model contains the following components:</p>
<p><ul>
<li>External inlet manifolds for both anode and cathode side</li>
<li>End plates</li>
<li>Feed and drain connectors of reaction gas type (ideal gas) for both anode and cathode</li>
<li>Electrical pins (positive and negative)</li>
<li>Solid oxide fuel cell substack model</li>
<li>Mixing volumes and linear flow resistance before the fluid flow outlets</li>
</ul></p>
<p>The model supports both co-flow and counter-flow configurations, this is determined by the <code>counterFlow</code> parameter.</p>
<h4>Parametrization</h4>
The discretization of the stack is defined using the following parameters:
<p><ul>
<li>The parameter <code>nbrSubStacks</code> describes the number of substacks in parallel</li>
<li>The parameter (array) <code>n_cell</code> describes the number of cells in each substack</li>
<li>The parameter <code>N</code> describes the number of discretizations in the flow direction (membrane and flow channels in each substack divided in <code>N</code> segments) </li>
</ul></p>
<h4>External Inlet Manifolds</h4>
<p>The external inlet manifolds are used to split the flow between the substacks. The pressure drop in each outlet from the manifold is calculated from the nominal mass flow rate through each manifold outlet. The nominal mass flow rate is in turn calculated from the overall nominal mass flow rate and the splitratios defined by the parameters <code>splitRatio_an</code> and <code>splitRatio_cath</code>. Please see <a href=\"modelica://FuelCell.Manifolds\">the manifold models</a> for more information about the manifolds.</p>
<h4>Substack Model</h4>
<p>The stack is parameterized to use a (<a href=\"modelica://FuelCell.Stacks.SOFC.SOFCSubStack\">FuelCell.Stacks.SOFC.SOFCSubStack</a>) substack.
Some parameters are still left unparameterized and needs to be defined in the experimental setup; the parameter <code>nbrSubStacks</code> describes the number of parallel substacks, connected electrically and thermally in series. The number of cells modelled by each substack is described by the array parameter <code>n_cell</code> and the number discrete segments in the fluid flow direction of each substack is described by the parameter <code>N</code>.</p>
<p></p>
</html>"),
     Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics),
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}), graphics={Text(
          extent={{-100,-82},{100,-94}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="%name")}));
end SOFCStack;
