using UnityEngine;
using UnityEngine.UI;

namespace ROS2
{
    public class BoolPublisher2 : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;

        private IPublisher<std_msgs.msg.Bool> camBoolPub;

        private IPublisher<geometry_msgs.msg.Twist> cmdVelPub;

        private bool camBoolState = false;
        private bool teleopBoolState = false;
        private bool vueltasBoolState = false;
        private bool avanzarBoolState = false;

        private float linearSpeed = 0.0f;
        private float angularSpeed = 0.0f;
        private const float maxSpeed = 1.0f;

        [Range(0, 100)]
        public float speedSlider = 50;

        public Button camButton;
        public Button forwardButton;
        public Button backwardButton;
        public Button rotateLeftButton;
        public Button rotateRightButton;

        private Color trueColor = new Color32(40, 167, 69, 255);
        private Color falseColor = new Color32(0, 123, 255, 255);
        private Color activeColor = new Color32(255, 106, 0, 255); // Nuevo color #FF6A00
        private Color inactiveColor = new Color32(241, 196, 15, 255);

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            UpdateButtonColors();
        }

        void Update()
        {
            if (ros2Unity.Ok() && ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityBoolTalkerNode");
                camBoolPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam");

                ros2Node = ros2Unity.CreateNode("ROS2UnityCmdVelTalkerNode");
                cmdVelPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>("/cmd_vel");
            }
        }

        private float CalculateIncrement()
        {
            return 0.05f;
        }

        private void PublishCmdVel()
        {
            if (cmdVelPub != null)
            {
                geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist();
                msg.Linear = new geometry_msgs.msg.Vector3 { X = linearSpeed, Y = 0.0f, Z = 0.0f };
                msg.Angular = new geometry_msgs.msg.Vector3 { X = 0.0f, Y = 0.0f, Z = angularSpeed };

                cmdVelPub.Publish(msg);

                Debug.Log($"Published to /cmd_vel: Linear={linearSpeed}, Angular={angularSpeed}");
                UpdateMovementButtonColors();
            }
        }

        public void MoveForward()
        {
            if (linearSpeed < 0)
            {
                linearSpeed += CalculateIncrement();
                if (linearSpeed > 0) linearSpeed = 0;
            }
            else
            {
                linearSpeed += CalculateIncrement();
                linearSpeed = Mathf.Clamp(linearSpeed, 0.0f, maxSpeed);
            }
            PublishCmdVel();
        }

        public void MoveBackward()
        {
            if (linearSpeed > 0)
            {
                linearSpeed -= CalculateIncrement();
                if (linearSpeed < 0) linearSpeed = 0;
            }
            else
            {
                linearSpeed -= CalculateIncrement();
                linearSpeed = Mathf.Clamp(linearSpeed, -maxSpeed, 0.0f);
            }
            PublishCmdVel();
        }

        public void RotateRight()
        {
            angularSpeed -= CalculateIncrement();
            angularSpeed = Mathf.Clamp(angularSpeed, -maxSpeed, maxSpeed);
            PublishCmdVel();
        }

        public void RotateLeft()
        {
            angularSpeed += CalculateIncrement();
            angularSpeed = Mathf.Clamp(angularSpeed, -maxSpeed, maxSpeed);
            PublishCmdVel();
        }

        public void StopMotion()
        {
            linearSpeed = 0.0f;
            angularSpeed = 0.0f;
            PublishCmdVel();
        }

        public void UpdateSpeedFromSlider(float sliderValue)
        {
            speedSlider = sliderValue;
            Debug.Log($"Slider updated: Increment={CalculateIncrement()} m/s");
        }

        public void SendCamBool()
        {
            if (camBoolPub != null)
            {
                camBoolState = !camBoolState;
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = camBoolState;
                camBoolPub.Publish(msg);

                Debug.Log($"Published to /gui_cam: {msg.Data}");
                camButton.image.color = camBoolState ? trueColor : falseColor;
            }
        }

        private void UpdateMovementButtonColors()
        {
            const float tolerance = 0.01f;

            forwardButton.image.color = (linearSpeed > tolerance) ? activeColor : inactiveColor;
            backwardButton.image.color = (linearSpeed < -tolerance) ? activeColor : inactiveColor;
            rotateLeftButton.image.color = (angularSpeed > tolerance) ? activeColor : inactiveColor;
            rotateRightButton.image.color = (angularSpeed < -tolerance) ? activeColor : inactiveColor;
        }

        private void UpdateButtonColors()
        {
            camButton.image.color = camBoolState ? trueColor : falseColor;
        }
    }
}
