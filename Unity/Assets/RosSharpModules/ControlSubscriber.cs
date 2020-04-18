using System.Threading;
using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.swc_msgs
{
    [RequireComponent(typeof(AckermannController))]
    public class ControlSubscriber : UnitySubscriber<Control>
    {
        private AckermannController car;

        private Control lastMessage;

        private float angleNoiseStdDev = 0.4f;
        private float powerNoiseStdDev = 0.1f;

        private bool firstMessage = true;
        private bool begingame = false;
        private bool newMessage = false;
        private float startTime = 100000;

        protected override void Start()
        {
            base.Start();
            car = GetComponent<AckermannController>();
            lastMessage = new Control();
            startTime = Time.realtimeSinceStartup;

            if (ConfigLoader.simulator.ManualControl) {
                this.enabled = false;
            }

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

            if (!ConfigLoader.simulator.CompetitionMode) {
                // Don't care about the stopsim stuff
                begingame = false;
                firstMessage = false;
                GameManager.instance.StartSim();
            }
        }

        private void FixedUpdate() {
            if (newMessage) {
                car.SetControl(lastMessage.speed + SimUtils.getRandNormal(0, powerNoiseStdDev), lastMessage.turn_angle + SimUtils.getRandNormal(0, angleNoiseStdDev));
                newMessage = false;
            }

            if (begingame)
            {
                begingame = false;
                GameManager.instance.StartSim();
            }

            if (firstMessage && !begingame && Time.realtimeSinceStartup - startTime >= 30) {
                GameManager.instance.StopSim("Did not start in 30 seconds!");
            }
        }

        protected override void ReceiveMessage(Control control)
        {
            lastMessage = control;
            newMessage = true;

            if (firstMessage)
            {
                begingame = true;
                firstMessage = false;
            }
        }
    }
}