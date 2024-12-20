using UnityEngine;
using UnityEngine.UI;

namespace ROS2
{
    public class BoolPublisher2 : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;

        private IPublisher<std_msgs.msg.Bool> logitech1Pub;
        private IPublisher<std_msgs.msg.Bool> logitech2Pub;
        private IPublisher<std_msgs.msg.Bool> siyiPub;
        private IPublisher<std_msgs.msg.Bool> zedPub;
        private IPublisher<std_msgs.msg.Bool> realsensePub;

        private IPublisher<std_msgs.msg.Int32> nCamsPub;

        private bool camBoolState = false;

        [Range(0, 100)]
        public float speedSlider = 50;

        public Button camButton;

        [SerializeField] private Sprite initialImage;
        [SerializeField] private Sprite toggledImage;

        [Header("UI References for Cameras")]
        public GameObject zedUI;
        public GameObject realsenseUI;
        public GameObject siyiUI;
        public GameObject logitech1UI;
        public GameObject logitech2UI;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            UpdateButtonImage();
        }

        void Update()
        {
            if (ros2Unity.Ok() && ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityBoolTalkerNode");

                logitech1Pub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_logitech_1");
                logitech2Pub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_logitech_2");
                siyiPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_siyi");
                zedPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_zed");
                realsensePub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_realsense");

                nCamsPub = ros2Node.CreatePublisher<std_msgs.msg.Int32>("/n_cams");
            }
        }

        public void SendCamBool()
        {
            camBoolState = !camBoolState;
            UpdateButtonImage();

            ToggleAllCameras(camBoolState);
            PublishIndividualTopics(camBoolState);
            PublishNCams(camBoolState);
        }

        private void ToggleAllCameras(bool state)
        {
            if (zedUI != null) zedUI.SetActive(state);
            if (realsenseUI != null) realsenseUI.SetActive(state);
            if (siyiUI != null) siyiUI.SetActive(state);
            if (logitech1UI != null) logitech1UI.SetActive(state);
            if (logitech2UI != null) logitech2UI.SetActive(state);

            Debug.Log($"All cameras set to: {(state ? "Active" : "Inactive")}");
        }

        private void PublishIndividualTopics(bool state)
        {
            std_msgs.msg.Bool msg = new std_msgs.msg.Bool { Data = state };

            if (logitech1Pub != null)
            {
                logitech1Pub.Publish(msg);
                Debug.Log($"Published to /gui_cam_logitech_1: {state}");
            }

            if (logitech2Pub != null)
            {
                logitech2Pub.Publish(msg);
                Debug.Log($"Published to /gui_cam_logitech_2: {state}");
            }

            if (siyiPub != null)
            {
                siyiPub.Publish(msg);
                Debug.Log($"Published to /gui_cam_siyi: {state}");
            }

            if (zedPub != null)
            {
                zedPub.Publish(msg);
                Debug.Log($"Published to /gui_cam_zed: {state}");
            }

            if (realsensePub != null)
            {
                realsensePub.Publish(msg);
                Debug.Log($"Published to /gui_cam_realsense: {state}");
            }
        }

        private void PublishNCams(bool state)
        {
            if (nCamsPub != null)
            {
                std_msgs.msg.Int32 msg = new std_msgs.msg.Int32();
                msg.Data = state ? 5 : 0;
                nCamsPub.Publish(msg);

                Debug.Log($"Published to /n_cams: {msg.Data}");
            }
        }

        private void UpdateButtonImage()
        {
            if (camButton != null && camButton.image != null)
            {
                camButton.image.sprite = camBoolState ? toggledImage : initialImage;
            }
        }
    }
}
