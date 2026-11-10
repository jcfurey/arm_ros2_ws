// Weld a grasped part to the hand on request, as grasp-fix plugins do.
//
// Bullet collides a box pad with a curved part through one contact point per
// step, so a part held only by friction can tip in the hand and wedge the
// pads open over a carry. Once the fingers have closed and stalled on a part,
// a fixed joint holds it where it was gripped until release; the grasp itself
// stays physical. gz-sim's DetachableJoint system cannot start detached in
// this release, so this world system creates the same DetachableJoint
// components on demand.
//
// Requests are gz.msgs.StringMsg on <topic> (default /grasp_weld):
//   "attach <part model> <model>::<link>"   weld the part's canonical link
//   "detach <part model>"                   remove that weld
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace arm_bringup {

class GraspWeld : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const auto topic = sdf->Get<std::string>("topic", "/grasp_weld").first;
    if (!node_.Subscribe(topic, &GraspWeld::OnRequest, this)) {
      gzerr << "GraspWeld cannot subscribe to " << topic << "\n";
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    std::vector<std::string> requests;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      requests.swap(requests_);
    }
    for (const auto & request : requests) {
      std::istringstream words(request);
      std::string verb, part, parent;
      words >> verb >> part >> parent;
      const auto weld = welds_.find(part);
      if (verb == "detach") {
        if (weld != welds_.end()) {
          ecm.RequestRemoveEntity(weld->second);
          welds_.erase(weld);
        }
        continue;
      }
      const auto separator = parent.find("::");
      if (verb != "attach" || separator == std::string::npos) {
        gzerr << "GraspWeld ignores request [" << request << "]\n";
        continue;
      }
      if (weld != welds_.end()) {
        continue;  // already held
      }
      const auto child_link = FindModel(ecm, part).CanonicalLink(ecm);
      const auto parent_link =
        FindModel(ecm, parent.substr(0, separator)).LinkByName(ecm, parent.substr(separator + 2));
      if (child_link == gz::sim::kNullEntity || parent_link == gz::sim::kNullEntity) {
        gzerr << "GraspWeld cannot find the links for [" << request << "]\n";
        continue;
      }
      const auto joint = ecm.CreateEntity();
      ecm.CreateComponent(joint, gz::sim::components::DetachableJoint({parent_link, child_link, "fixed"}));
      welds_[part] = joint;
    }
  }

private:
  static gz::sim::Model FindModel(const gz::sim::EntityComponentManager & ecm, const std::string & name)
  {
    return gz::sim::Model(ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name(name)));
  }

  void OnRequest(const gz::msgs::StringMsg & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    requests_.push_back(message.data());
  }

  gz::transport::Node node_;
  std::mutex mutex_;
  std::vector<std::string> requests_;
  std::map<std::string, gz::sim::Entity> welds_;
};

}  // namespace arm_bringup

GZ_ADD_PLUGIN(arm_bringup::GraspWeld, gz::sim::System,
              arm_bringup::GraspWeld::ISystemConfigure,
              arm_bringup::GraspWeld::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(arm_bringup::GraspWeld, "arm_bringup::GraspWeld")
