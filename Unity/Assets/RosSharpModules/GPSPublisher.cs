using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.swc_msgs
{
    public class GPSPublisher : UnityPublisher<Gps>
    {
        private Gps message;

        public float latNoiseStdDev = 1.843f;
        public float lonNoiseStdDev = 2.138f;

        public float lat0Pos = 35.205853f;
        public float lon0Pos = -97.442325f;

        private float previousScanTime = 5;
        public float updatePeriod = 0.1f;

        protected override void Start()
        {
            base.Start();
            message = new Gps();

            switch (ConfigLoader.competition.NoiseLevel) {
                case ConfigLoader.CompetitionConfig.NoiseLevels.none:
                    latNoiseStdDev *= 0;
                    lonNoiseStdDev *= 0;
                    break;
                case ConfigLoader.CompetitionConfig.NoiseLevels.reduced:
                    latNoiseStdDev *= 0.5f;
                    lonNoiseStdDev *= 0.5f;
                    break;
            }
        }

        private void FixedUpdate()
        {
            if (Time.realtimeSinceStartup >= previousScanTime + updatePeriod)
            {
                WriteMessage();
                previousScanTime = Time.realtimeSinceStartup;
            }
        }

        private void WriteMessage() {
            Vector3 pos = this.transform.position;
            message.latitude = (pos.z + SimUtils.getRandNormal(0, latNoiseStdDev)) / 110944.33 + lat0Pos;
            message.longitude = (pos.x + SimUtils.getRandNormal(0, lonNoiseStdDev)) / 91058.93 + lon0Pos;
            Publish(message);
        }
    }
}