using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.Std
{
    [RequireComponent(typeof(AckermannController))]
    public class VelocityPublisher : UnityPublisher<Float32>
    {
        private Float32 message;

        private float velocityNoiseStdDev = 0.1f;

        private float previousScanTime = 5;
        private float updatePeriod = 0.05f;
        private AckermannController c;

        protected override void Start()
        {
            base.Start();
            c = GetComponent<AckermannController>();
            message = new Float32();

            switch (ConfigLoader.competition.NoiseLevel) {
                case ConfigLoader.CompetitionConfig.NoiseLevels.none:
                    velocityNoiseStdDev *= 0;
                    break;
                case ConfigLoader.CompetitionConfig.NoiseLevels.reduced:
                    velocityNoiseStdDev *= 0.5f;
                    break;
            }
        }

        private void FixedUpdate()
        {
            if (UnityEngine.Time.realtimeSinceStartup >= previousScanTime + updatePeriod)
            {
                WriteMessage();
                previousScanTime = UnityEngine.Time.realtimeSinceStartup;
            }
        }

        private void WriteMessage() {
            message.data = Mathf.Round((c.Power + SimUtils.getRandNormal(0, velocityNoiseStdDev) * c.Power / 4.0f) * 1000f) / 1000f;
            Publish(message);
        }
    }
}