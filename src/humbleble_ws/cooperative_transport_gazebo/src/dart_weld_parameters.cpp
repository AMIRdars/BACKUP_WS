#include <cmath>
#include <iostream>
#include <stdexcept>
#include <dart/constraint/DynamicJointConstraint.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

namespace cooperative_transport
{
// Applies to DART dynamic constraints (including detachable welds),
// not contacts or servo joint constraints. Forces remain solver reactions.
class DartWeldParameters final : public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    if (!sdf->HasElement("constraint_force_mixing")) {
      throw std::invalid_argument("Explicit weld CFM is required");
    }
    const auto value = sdf->Get<double>("constraint_force_mixing");
    if (!std::isfinite(value) || value < 1e-9 || value > 1.) {
      throw std::invalid_argument("Weld CFM outside DART documented range");
    }
    std::clog << "[DartWeldParameters] previous CFM="
      << dart::constraint::DynamicJointConstraint::getConstraintForceMixing()
      << ", requested CFM=" << value << "\n";
    dart::constraint::DynamicJointConstraint::setConstraintForceMixing(value);
  }
};
}
IGNITION_ADD_PLUGIN(cooperative_transport::DartWeldParameters, gz::sim::System,
  gz::sim::ISystemConfigure)
