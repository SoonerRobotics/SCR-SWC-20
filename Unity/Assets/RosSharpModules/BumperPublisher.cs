using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.Std
{
    public class BumperPublisher : UnityPublisher<Bool>
    {
        private Bool message;
        protected override void Start()
        {
            base.Start();
            message = new Bool();
        }

        private void OnTriggerEnter(Collider other) {
            message.data = true;
            Publish(message);
        }

        private void OnTriggerExit(Collider other) {
            message.data = false;
            Publish(message);
        }
    }
}