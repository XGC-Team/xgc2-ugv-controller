# PVA receipt traces

Set private parameter `publish_pva_receipts:=true` to record accepted PVA samples at the actual controller receiver. The default is false. The PVA subscriber, finite-value rejection, local receipt-time lift, event delivery, flatness law and native command publisher are unchanged.

For an input topic `alg/reference/pva`, the receiver publishes `alg/reference/pva/receipt` (`std_msgs/Float64MultiArray`) and a latched `alg/reference/pva/contract` (`std_msgs/String`, JSON). Both are diagnostics, never command acknowledgements. No subscriber count on these topics gates control.

The array has one dimension. Its label names all twelve fields: schema version 1; local ROS receipt time in seconds; source header time in seconds; source header sequence; receiver sequence; world x/y in metres, vx/vy in m/s, ax/ay in m/s²; accepted yaw in radians. The receiver sequence is uint32 and can wrap. Segment analysis at a receiver restart or backward clock step. The source time can be in the future and is not used to defer consumption.

For receipt t_r and the next accepted receipt t_next, evaluate the consumed reference on [t_r,t_next) as q(t)=q_r+v_r*d+.5*a_r*d², v(t)=v_r+a_r*d, d=max(0,t-t_r). A stopped or unhealthy controller does not imply this stored reference was executed. Intersect with recorded controller/mission execution states when scoring q-p.

The contract reports effective controller configuration after parameter loading, including strategy, state source, chassis bounds, flatness gains and filter parameters. It describes the constructor configuration; deployments that subsequently call setConfig must separately record that change. Receipt publication is not proof of hardware delivery, a physical tracking bound or full experiment acceptance.
