# PVA receipt diagnostic

The receiver publishes non-latched `std_msgs/String` JSON on `<resolved-pva-topic>/receipt` for each finite accepted PVA. The schema is `unicycle.pva-receipt/v1`.

`source_sec/source_nsec/source_sequence` preserve the input header. `received_sec/received_nsec` identify the exact receiver clock reading used by `liftWorldPva`. `received_sequence` counts accepted samples in this process. Integer seconds and nanoseconds avoid epoch-time float rounding. `pva` is the accepted world-frame `[x,y,yaw,vx,vy,ax,ay]`, in metres, radians, metres/second and metres/second². Yaw reflects the receiver's normalization.

The input's source activation stamp is not substituted for the local receipt clock. Reconstruct the consumed reference at controller time `t` with `dt=max(0,t-received_time)`, `p=p0+v0*dt+a0*dt²/2`, `v=v0+a0*dt`. Source-time planning error is a separate measurement. Keep trial clock domains separate and check receipt sequence gaps. The trace does not add a reference timeout, queue, mode switch or command retransmission.
