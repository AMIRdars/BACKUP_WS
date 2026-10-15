#include <cmath>
#include <iostream>
#include <stdexcept>

#include <dart/constraint/ContactConstraint.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

namespace cooperative_transport
{
// Fortress's DART backend does not consume the ODE pad kp/kd settings.
// Configure its native contact compliance and penetration recovery instead.
// These are solver parameters; no payload wrench or attachment is added.
class DartContactParameters final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &config,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    const double cfm = config->Get<double>("constraint_force_mixing", 0.05).first;
    const double erp = config->Get<double>("error_reduction_parameter", 0.2).first;
    const double max_velocity = config->Get<double>("max_error_velocity", 0.2).first;
    if (!std::isfinite(cfm) || cfm < 1e-9 || cfm > 1.0 ||
      !std::isfinite(erp) || erp < 0.0 || erp > 1.0 ||
      !std::isfinite(max_velocity) || max_velocity < 0.0)
    {
      throw std::invalid_argument("Invalid DART contact solver parameters");
    }
    dart::constraint::ContactConstraint::setConstraintForceMixing(cfm);
    dart::constraint::ContactConstraint::setErrorReductionParameter(erp);
    dart::constraint::ContactConstraint::setMaxErrorReductionVelocity(max_velocity);
    std::clog << "[DartContactParameters] CFM=" << cfm << ", ERP=" << erp
              << ", max recovery velocity=" << max_velocity << " m/s\n";
  }
};
}  // namespace cooperative_transport

IGNITION_ADD_PLUGIN(
  cooperative_transport::DartContactParameters,
  gz::sim::System,
  gz::sim::ISystemConfigure)
