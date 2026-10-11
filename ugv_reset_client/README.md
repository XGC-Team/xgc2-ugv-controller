# ugv_reset_client

The vehicle's side of the UGV Reset protocol with the station's coordinator
(`ugv_reset_safety`): a request/response lease for one vehicle, and its ROS client.

| | |
|---|---|
| `ugv_reset_client/reset_lease.h` | `ResetLease`, ROS-free: the ledger of the requests the vehicle issued under the current session, the freshness of a response on the ROS clock and the wall clock, once-only acceptance, the command the vehicle applied last. Thread safe, because a response arrives on a subscriber thread while requests leave from the control loop. |
| `ugv_reset_client/reset_client.h` | `ResetClient`, the ROS edge of the lease: publishes `reset/request`, takes `reset/response` (both relative to the vehicle's namespace unless the topics are given) and hands every accepted response to a sink. |

The vehicle's controller owns the Reset session: its generation (random at start,
never reused while the clock is paused) and the target it froze when it entered
Reset. The owner of a client reads the session from the controller after each
control update and gives it to `ResetClient::update`; the accepted responses go
back to the controller, which executes them only for its current session and while
the lease holds. There is no other vehicle in this package and no coordination.

Used by `unicycle_ugv_controller`, `mecanum_ugv_controller` and the ROS edge of
`ugv_modules`. The messages stay in `ugv_reset_safety` until the cross-product
messages move to the shared message repository.
