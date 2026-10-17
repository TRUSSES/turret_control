#include "cubemars_reply_router.h"
#include "pi3hat.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {
struct Receiver {
  int observed = 0;
  int addressed = 0;
  int accepted = 0;
  int rejected_size = 0;
  int last_position_byte = -1;
};
using Router = turret_control::CubemarsReplyRouter<Receiver>;
using Frame = mjbots::pi3hat::CanFrame;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
  }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

Frame reply(int motor_id, int position_byte, int bus = 5, int size = 8) {
  Frame frame;
  frame.bus = bus;
  frame.id = 0;  // Motors can share a host arbitration ID.
  frame.size = size;
  frame.data[0] = motor_id;
  frame.data[1] = position_byte;
  return frame;
}

void dispatch(Router& router, const void* transport, const Frame& frame) {
  router.dispatch(transport, frame,
      [](Receiver& owner, const Frame&) { ++owner.observed; },
      [](Receiver& owner, const Frame& addressed) {
        ++owner.addressed;
        if (addressed.size != 8) {
          ++owner.rejected_size;
          return;
        }
        ++owner.accepted;
        owner.last_position_byte = addressed.data[1];
      });
}

void testMixedRepliesBelongToTheirMotorsRegardlessOfCommandSender() {
  Router router;
  int transport = 0;
  Receiver pitch, yaw;
  router.add(&transport, 5, 10, &pitch);
  router.add(&transport, 5, 11, &yaw);

  // A pitch command can receive the preceding yaw reply first. Both replies
  // must be decoded, even though their arbitration IDs are identical.
  dispatch(router, &transport, reply(11, 24));
  dispatch(router, &transport, reply(10, 42));
  CHECK(pitch.accepted == 1 && pitch.last_position_byte == 42);
  CHECK(yaw.accepted == 1 && yaw.last_position_byte == 24);
  CHECK(pitch.observed == 2 && yaw.observed == 2);

  // Mode and zero transactions use this same dispatch operation; they must
  // not discard a motor's reply just because no own reply was expected.
  dispatch(router, &transport, reply(11, 25));
  CHECK(yaw.accepted == 2 && yaw.last_position_byte == 25);
  CHECK(pitch.accepted == 1);
}

void testTransportBusAndPayloadMotorIdAllSelectTheOwner() {
  Router router;
  int first_transport = 0, second_transport = 0;
  Receiver first_bus_yaw, other_bus_yaw, other_transport_yaw;
  router.add(&first_transport, 5, 11, &first_bus_yaw);
  router.add(&first_transport, 4, 11, &other_bus_yaw);
  router.add(&second_transport, 5, 11, &other_transport_yaw);

  dispatch(router, &first_transport, reply(11, 1));
  CHECK(first_bus_yaw.accepted == 1);
  CHECK(other_bus_yaw.observed == 0 && other_transport_yaw.observed == 0);
  dispatch(router, &first_transport, reply(11, 2, 4));
  dispatch(router, &second_transport, reply(11, 3));
  CHECK(other_bus_yaw.accepted == 1 && other_bus_yaw.last_position_byte == 2);
  CHECK(other_transport_yaw.accepted == 1 && other_transport_yaw.last_position_byte == 3);
  CHECK(first_bus_yaw.accepted == 1);

  Frame wrong_payload = reply(12, 99);
  wrong_payload.id = 11;  // Arbitration ID alone must not identify the motor.
  dispatch(router, &first_transport, wrong_payload);
  CHECK(first_bus_yaw.accepted == 1 && first_bus_yaw.observed == 2);
}

void testMalformedFramesDoNotBecomeAcceptedFeedback() {
  Router router;
  int transport = 0;
  Receiver yaw;
  router.add(&transport, 5, 11, &yaw);
  dispatch(router, &transport, reply(11, 10));
  dispatch(router, &transport, reply(11, 99, 5, 6));
  dispatch(router, &transport, reply(11, 99, 5, 0));
  CHECK(yaw.observed == 3);
  CHECK(yaw.addressed == 2 && yaw.rejected_size == 1);
  CHECK(yaw.accepted == 1 && yaw.last_position_byte == 10);
}

void testRemovalAndReRegistrationDoNotLeaveAnOldOwner() {
  Router router;
  int transport = 0;
  Receiver original, replacement;
  router.add(&transport, 5, 11, &original);
  bool duplicate_rejected = false;
  try {
    router.add(&transport, 5, 11, &replacement);
  } catch (const std::invalid_argument&) {
    duplicate_rejected = true;
  }
  CHECK(duplicate_rejected);
  router.remove(&transport, 5, 11, &replacement);
  dispatch(router, &transport, reply(11, 1));
  CHECK(original.accepted == 1 && replacement.accepted == 0);
  router.remove(&transport, 5, 11, &original);
  dispatch(router, &transport, reply(11, 2));
  CHECK(original.accepted == 1);
  router.add(&transport, 5, 11, &replacement);
  dispatch(router, &transport, reply(11, 3));
  CHECK(original.accepted == 1 && replacement.accepted == 1);
}
}  // namespace

int main() {
  try {
    testMixedRepliesBelongToTheirMotorsRegardlessOfCommandSender();
    testTransportBusAndPayloadMotorIdAllSelectTheOwner();
    testMalformedFramesDoNotBecomeAcceptedFeedback();
    testRemovalAndReRegistrationDoNotLeaveAnOldOwner();
    std::cout << "CubeMars reply router tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
