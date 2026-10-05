// Contact-gated fixed constraint: never writes an object pose or velocity.
#include <atomic>
#include <chrono>
#include <string>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

namespace sim = ignition::gazebo;
class ContactGrasp : public sim::System, public sim::ISystemConfigure,
                     public sim::ISystemPreUpdate {
  sim::Entity parent{sim::kNullEntity}, child{sim::kNullEntity}, joint{sim::kNullEntity};
  std::string object, state{"detached"};
  ignition::transport::Node node;
  ignition::transport::Node::Publisher output;
  std::atomic<bool> attach{false}, detach{false};
  double requested{-1}, lastPublish{-1};
 public:
  void Configure(const sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager &ecm, sim::EventManager &) override {
    parent = sim::Model(entity).LinkByName(ecm, "wrist_3_link");
    object = sdf->Get<std::string>("child_model");
    const std::string prefix = "/ur3_llm/grasp/" + object;
    output = node.Advertise<ignition::msgs::StringMsg>(prefix + "/state");
    node.Subscribe(prefix + "/attach", &ContactGrasp::Attach, this);
    node.Subscribe(prefix + "/detach", &ContactGrasp::Detach, this);
  }
  void Attach(const ignition::msgs::Empty &) { attach = true; }
  void Detach(const ignition::msgs::Empty &) { detach = true; }
  void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override {
    if (info.paused || parent == sim::kNullEntity) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (detach.exchange(false)) {
      attach = false;
      requested = -1;
      if (joint != sim::kNullEntity) ecm.RequestRemoveEntity(joint);
      joint = sim::kNullEntity;
      state = "detached";
      Publish();
    }
    if (attach.load() && joint == sim::kNullEntity) {
      if (requested < 0) requested = now;
      const auto model = ecm.EntityByComponents(sim::components::Model(), sim::components::Name(object));
      child = sim::Model(model).LinkByName(ecm, "link");
      bool left = false, right = false;
      ecm.Each<sim::components::ContactSensorData>(
        [&](const sim::Entity &, const sim::components::ContactSensorData *data) {
          for (const auto &contact : data->Data().contact()) {
            const auto a = sim::scopedName(contact.collision1().id(), ecm, "::", false);
            const auto b = sim::scopedName(contact.collision2().id(), ecm, "::", false);
            if (a.find(object + "::") == std::string::npos &&
                b.find(object + "::") == std::string::npos) continue;
            const std::string combined = a + " " + b;
            left |= combined.find("left_finger_link") != std::string::npos;
            right |= combined.find("right_finger_link") != std::string::npos;
          }
          return true;
        });
      if (left && right && child != sim::kNullEntity) {
        joint = ecm.CreateEntity();
        ecm.CreateComponent(joint, sim::components::DetachableJoint({parent, child, "fixed"}));
        state = "attached";
        attach = false;
        requested = -1;
        Publish();
      } else if (now - requested > 2.0) {
        state = "rejected_no_bilateral_contact_left=" + std::to_string(left) +
                "_right=" + std::to_string(right);
        attach = false;
        requested = -1;
        Publish();
      }
    }
    if (now - lastPublish >= 0.1) { Publish(); lastPublish = now; }
  }
  void Publish() {
    ignition::msgs::StringMsg msg;
    msg.set_data(state);
    output.Publish(msg);
  }
};
IGNITION_ADD_PLUGIN(ContactGrasp, sim::System, ContactGrasp::ISystemConfigure, ContactGrasp::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(ContactGrasp, "ur3::ContactGrasp")
