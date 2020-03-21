using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.swc_msgs
{
    [RequireComponent(typeof(AckermannController))]
    public class ControlSubscriber : UnitySubscriber<Control>
    {
        private AckermannController car;

        protected override void Start()
        {
            base.Start();
            car = GetComponent<AckermannController>();
        }

        protected override void ReceiveMessage(Control control)
        {
            car.CntrlAngle = control.turn_angle;
            car.CntrlPower = control.speed;
        }
    }
}