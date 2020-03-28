using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.swc_msgs
{
    [RequireComponent(typeof(AckermannController))]
    public class ControlSubscriber : UnitySubscriber<Control>
    {
        private AckermannController car;

        private bool firstMessage = true;
        private bool begingame = false;

        protected override void Start()
        {
            if (ConfigLoader.simulator.ManualControl)
                return;

            base.Start();
            car = GetComponent<AckermannController>();
        }

        private void Update()
        {
            if (begingame)
            {
                GameManager.instance.StartSim();
                begingame = false;
            }
        }

        protected override void ReceiveMessage(Control control)
        {
            car.CntrlAngle = control.turn_angle;
            car.CntrlPower = control.speed;

            if (firstMessage)
            {
                begingame = true;
                firstMessage = false;
            }
        }
    }
}