#ifndef ___DROPBOT__SIGNAL_TIMER__H___
#define ___DROPBOT__SIGNAL_TIMER__H___


namespace dropbot {

/**
 * @brief Wrapper around `Signal` class to provide time-based receivers.
 *
 * Each receiver is connected along with an interval at which the receiver
 * should be called.  By default, receivers are called once every interval.
 * Optionally, receivers can be called only after the first interval has passed
 * by specifying `repeat=false` when connecting the receiver.
 */
class SignalTimer {
public:
    typedef std::function<void(uint32_t)> Receiver;
    /**
     * @brief Must be called periodically to update the timer counter.
     *
     * @param now  Current timer counter value, e.g., result of `millis()`.
     */
    void update(uint32_t now) {
      timer_context_.send(now);
      now_ = now;
    }
    /**
     * @brief Connect a receiver function to be called at the specified
     * interval.
     *
     * @param receiver  Receiver function with signature `void(uint32_t now)`.
     * @param interval  Interval after which the receiver should be called.
     *     Units are the same as the units supplied to the `update()` method.
     * @param repeat  Optional.  If `true`, call after every interval of the
     *     specified length has passed.  If `false` only call the receiver
     *     once; after the *first* interval.
     */
    void connect(Receiver receiver, uint32_t interval, bool repeat=true) {
        timer_context_.connect(Adapter(receiver, now_, interval, repeat, &timer_context_));
    };
private:
    uint32_t now_ = 0;

    struct Adapter;

    Signal<Adapter> timer_context_;

    struct Adapter {
        Receiver receiver_;
        uint32_t start_;
        uint32_t interval_;
        bool repeat_;
        Signal<Adapter> *timer_context_;

        Adapter(Receiver receiver, uint32_t start, uint32_t interval, bool repeat, Signal<Adapter> *timer_context) :
        receiver_(receiver),
        start_(start),
        interval_(interval), repeat_(repeat),
        timer_context_(timer_context) {}

        void operator() (uint32_t now) {
            if ((now - start_) >= interval_) {
                receiver_(now);
                start_ = now;
                if (!repeat_) {
                    timer_context_->disconnect(this);
                }
            }
        }
    };
};

}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__SIGNAL_TIMER__H___
