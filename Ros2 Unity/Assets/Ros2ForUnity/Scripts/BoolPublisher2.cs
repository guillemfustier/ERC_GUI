using UnityEngine;

namespace ROS2
{
    /// <summary>
    /// Example class to send boolean messages to different topics via ROS2
    /// </summary>
    public class BoolPublisher2 : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;

        private IPublisher<std_msgs.msg.Bool> camBoolPub;
        private IPublisher<std_msgs.msg.Bool> teleopBoolPub;
        private IPublisher<std_msgs.msg.Bool> vueltasBoolPub;
        private IPublisher<std_msgs.msg.Bool> avanzarBoolPub;

        private bool camBoolState = true;
        private bool teleopBoolState = true;
        private bool vueltasBoolState = true;
        private bool avanzarBoolState = true;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }

        void Update()
        {
            if (ros2Unity.Ok() && ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityBoolTalkerNode");
                camBoolPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam");
                teleopBoolPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_teleop");
                vueltasBoolPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_vueltas");
                avanzarBoolPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_avanzar");
            }
        }

        public void SendCamBool()
        {
            if (camBoolPub != null)
            {
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = camBoolState;
                camBoolPub.Publish(msg);

                Debug.Log("Published to /gui_cam: " + msg.Data);

                camBoolState = !camBoolState;
            }
        }

        public void SendTeleopBool()
        {
            if (teleopBoolPub != null)
            {
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = teleopBoolState;
                teleopBoolPub.Publish(msg);

                Debug.Log("Published to /gui_teleop: " + msg.Data);

                teleopBoolState = !teleopBoolState;
            }
        }

        public void SendVueltasBool()
        {
            if (vueltasBoolPub != null)
            {
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = vueltasBoolState;
                vueltasBoolPub.Publish(msg);

                Debug.Log("Published to /gui_vueltas: " + msg.Data);

                vueltasBoolState = !vueltasBoolState;
            }
        }

        public void SendAvanzarBool()
        {
            if (avanzarBoolPub != null)
            {
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = avanzarBoolState;
                avanzarBoolPub.Publish(msg);

                Debug.Log("Published to /gui_avanzar " + msg.Data);

                avanzarBoolState = !avanzarBoolState;
            }
        }
    }
}
