#ifndef ___DROPBOT__SIGNAL__H___
#define ___DROPBOT__SIGNAL__H___


namespace dropbot {

/**
 * @brief Simple broadcast signalling inspired by [Python Blinker
 * signals][blinker].
 *
 * [blinker]: https://pythonhosted.org/blinker/
 *
 * Example
 * -------
 *
 * ```c++
 * #include <functional>
 * #include <iostream>
 * #include <Signal.h>
 *
 * int main() {
 *   Signal<std::function<void(bool, int)> > foo;
 *
 *   bool b;
 *   int i;
 *
 *   // Connect lambda function.  `A` is a reference to the connected function.
 *   auto A = foo.connect([&] (bool some_b, int some_i) {
 *       b = some_b;
 *       i = some_i;
 *   });
 *   auto B = foo.connect([] (bool some_b, int some_i) {
 *       std::cout << "bool: " << some_b << ", int: " << some_i << std::endl;
 *   });
 *
 *   foo.send(true, 42);  // Will call both `A` and `B` functions.
 *
 *   foo.disconnect(B);  // Disconnect `B` function from signal.
 *
 *   foo.send(true, 42);  // Only `A` will be called.
 *
 *   return 0;
 * }
 * ```
 *
 * @tparam Receiver
 */
template<typename Receiver>
class Signal {
public:
  typedef Receiver receiver_t;

  /**
   * @brief Send notification to all connected receivers.
   *
   * @tparam Targs
   * @param Fargs  Arguments to pass to receivers.
   */
  template <typename... Targs>
  void send(Targs... Fargs) {
    for (auto &receiver : receivers_) {
      receiver(Fargs...);
    }
  }
  /**
   * @brief Connect a receiver (i.e., callback function).
   *
   * @param receiver  A callback function matching `Receiver` signature.
   *
   * @return Reference to stored copy of `Receiver`, which can, e.g., be used
   * to disconnect the receiver.
   */
  Receiver *connect(Receiver receiver) {
    receivers_.push_back(receiver);
    return &receivers_[receivers_.size() - 1];
  }
  /**
   * @brief Disconnect a receiver from the signal.
   *
   * @param receiver_ref  Receiver reference returned by `connect()`.
   *
   * @return  `true` if the receiver was found/disconnected, `false` otherwise.
   */
  bool disconnect(Receiver *receiver_ref) {
    for (auto it = receivers_.begin(); it != receivers_.end(); it++) {
      auto &receiver = *it;
      if (&receiver == receiver_ref) {
        receivers_.erase(it, it + 1);
        return true;
      }
    }
    return false;
  }
  /**
   * @brief Disconnect all receivers.
   */
  void clear() { receivers_.clear(); }
protected:
  std::vector<Receiver> receivers_;
};

}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__SIGNAL__H___
