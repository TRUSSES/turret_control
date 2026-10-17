#ifndef CUBEMARS_REPLY_ROUTER_H
#define CUBEMARS_REPLY_ROUTER_H

#include <functional>
#include <map>
#include <stdexcept>
#include <tuple>

namespace turret_control {

// Pi3Hat receives every motor's reply on a shared bus, irrespective of which
// motor sent the current command. Ownership follows transport, bus and the
// reply's payload motor ID. The caller serializes registration and dispatch.
template <typename Receiver>
class CubemarsReplyRouter {
 public:
  void add(const void* transport, int bus, int motor_id, Receiver* receiver) {
    const auto inserted = receivers_.emplace(Key{transport, bus, motor_id}, receiver);
    if (!inserted.second) {
      throw std::invalid_argument("Duplicate CubeMars motor ID on the same Pi3Hat CAN bus");
    }
  }

  void remove(const void* transport, int bus, int motor_id, Receiver* receiver) {
    const auto found = receivers_.find(Key{transport, bus, motor_id});
    if (found != receivers_.end() && found->second == receiver) receivers_.erase(found);
  }

  template <typename Frame, typename Observer, typename Decoder>
  void dispatch(const void* transport, const Frame& frame,
                Observer observe, Decoder decode) const {
    for (const auto& entry : receivers_) {
      if (std::get<0>(entry.first) == transport && std::get<1>(entry.first) == frame.bus) {
        observe(*entry.second, frame);
      }
    }
    // There is no payload motor ID in an empty frame. A CAN arbitration ID
    // alone is insufficient: CubeMars replies may use a common host ID.
    if (frame.size == 0) return;
    const auto found = receivers_.find(Key{transport, frame.bus, frame.data[0]});
    if (found != receivers_.end()) decode(*found->second, frame);
  }

 private:
  using Key = std::tuple<const void*, int, int>;
  struct KeyLess {
    bool operator()(const Key& left, const Key& right) const {
      if (std::get<0>(left) != std::get<0>(right)) {
        return std::less<const void*>{}(std::get<0>(left), std::get<0>(right));
      }
      return std::tie(std::get<1>(left), std::get<2>(left)) <
             std::tie(std::get<1>(right), std::get<2>(right));
    }
  };
  std::map<Key, Receiver*, KeyLess> receivers_;
};

}  // namespace turret_control

#endif  // CUBEMARS_REPLY_ROUTER_H
