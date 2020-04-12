using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.swc_msgs
{
    [RequireComponent(typeof(AckermannController))]
    public class ControlSubscriber : UnitySubscriber<Control>
    {
        private AckermannController car;

        private float angleNoiseStdDev = 0.4f;
        private float powerNoiseStdDev = 0.1f;

        private bool firstMessage = true;
        private bool begingame = false;

        protected override void Start()
        {
            if (ConfigLoader.simulator.ManualControl)
                return;

            switch (ConfigLoader.competition.NoiseLevel) {
                case ConfigLoader.CompetitionConfig.NoiseLevels.none:
                    angleNoiseStdDev *= 0;
                    powerNoiseStdDev *= 0;
                    break;
                case ConfigLoader.CompetitionConfig.NoiseLevels.reduced:
                    angleNoiseStdDev *= 0.5f;
                    powerNoiseStdDev *= 0.5f;
                    break;
            }

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
            car.CntrlAngle = control.turn_angle + SimUtils.getRandNormal(0, angleNoiseStdDev);
            car.CntrlPower = control.speed + SimUtils.getRandNormal(0, powerNoiseStdDev);

            if (firstMessage)
            {
                begingame = true;
                firstMessage = false;
            }
        }
    }
}