using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    [RequireComponent(typeof(AckermannController))]
    public class IMUPublisher : UnityPublisher<Imu>
    {
        private Imu message;

        private float accelNoiseStdDev = 0.15f;
        private float orientationNoiseStdDev = 0.017f;
        private float angularVelocityNoiseStdDev = 0.017f;

        private AckermannController c;

        private float previousScanTime = 0;
        private float updatePeriod = 0.1f;

        protected override void Start()
        {
            base.Start();
            c = GetComponent<AckermannController>();
            message = new Imu();

            switch (ConfigLoader.competition.NoiseLevel) {
                case ConfigLoader.CompetitionConfig.NoiseLevels.none:
                    accelNoiseStdDev *= 0;
                    orientationNoiseStdDev *= 0;
                    angularVelocityNoiseStdDev *= 0;
                    break;
                case ConfigLoader.CompetitionConfig.NoiseLevels.reduced:
                    accelNoiseStdDev *= 0.5f;
                    orientationNoiseStdDev *= 0.5f;
                    angularVelocityNoiseStdDev *= 0.5f;
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
            message.header.Update();

            Vector3 eulers = c.transform.rotation.eulerAngles;
            eulers += new Vector3(SimUtils.getRandNormal(0, orientationNoiseStdDev), SimUtils.getRandNormal(0, orientationNoiseStdDev), SimUtils.getRandNormal(0, orientationNoiseStdDev));
            Quaternion orientation = Quaternion.Euler(eulers.x, -eulers.z, -eulers.y);

            message.orientation.w = orientation.w;
            message.orientation.x = orientation.x;
            message.orientation.y = orientation.y;
            message.orientation.z = orientation.z;

            Vector3 accel = c.transform.worldToLocalMatrix * c.accel; // sick maths

            message.linear_acceleration.x = accel.x + SimUtils.getRandNormal(0, accelNoiseStdDev);
            message.linear_acceleration.y = accel.z + SimUtils.getRandNormal(0, accelNoiseStdDev);
            message.linear_acceleration.z = accel.y + SimUtils.getRandNormal(0, accelNoiseStdDev);

            message.angular_velocity.x = c.angular_vel.x + SimUtils.getRandNormal(0, angularVelocityNoiseStdDev);
            message.angular_velocity.y = -c.angular_vel.z + SimUtils.getRandNormal(0, angularVelocityNoiseStdDev);
            message.angular_velocity.z = -c.angular_vel.y + SimUtils.getRandNormal(0, angularVelocityNoiseStdDev);

            Publish(message);
        }
    }
}