// Script: Cameras.cs
using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using UnityEngine.UI;
using std_msgs.msg;
using System.Collections.Concurrent;

public class Cameras : MonoBehaviour
{
    [Header("UI Configuration")]
    public GameObject logitech1UI;
    public GameObject logitech2UI;
    public GameObject siyiUI;
    public GameObject zedUI;
    public GameObject lidar2DUI;
    public GameObject lidar4DUI;
    public GameObject realsenseUI;

    public RawImage logitech1Image;
    public RawImage logitech2Image;
    public RawImage siyiImage;
    public RawImage zedImage;
    public RawImage lidar2DImage;
    public RawImage lidar4DImage;
    public RawImage realsenseImage;

    [Header("ROS2 Configuration")]
    public string topicLogitech1 = "/gui_cam_logitech_1";
    public string topicLogitech2 = "/gui_cam_logitech_2";
    public string topicSiyi = "/gui_cam_siyi";
    public string topicZed = "/gui_cam_zed";
    public string topicLidar2D = "/gui_lidar_2d";
    public string topicLidar4D = "/gui_lidar_4d";
    public string topicRealsense = "/gui_cam_realsense";

    public string imageTopicLogitech1 = "/camara_logitech_1/image_raw/compressed";
    public string imageTopicLogitech2 = "/camara_logitech_2/image_raw/compressed";
    public string imageTopicSiyi = "/camara_siyi/image_raw/compressed";
    public string imageTopicZed = "/camara_zed/image_raw/compressed";
    public string imageTopicRealsense = "/camara_realsense/image_raw/compressed";

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    private ISubscription<Bool> subLogitech1, subLogitech2, subSiyi, subZed, subLidar2D, subLidar4D, subRealsense;
    private ISubscription<CompressedImage> imageSubLogitech1, imageSubLogitech2, imageSubSiyi, imageSubZed, imageSubRealsense;

    private ConcurrentQueue<byte[]> imageQueueLogitech1 = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueueLogitech2 = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueueSiyi = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueueZed = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueueRealsense = new ConcurrentQueue<byte[]>();

    private Texture2D textureLogitech1, textureLogitech2, textureSiyi, textureZed, textureRealsense;

    private ConcurrentQueue<System.Action> mainThreadActions = new ConcurrentQueue<System.Action>();

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();

        // Start with all cameras disabled
        SetActiveRecursively(logitech1UI, false);
        SetActiveRecursively(logitech2UI, false);
        SetActiveRecursively(siyiUI, false);
        SetActiveRecursively(zedUI, false);
        SetActiveRecursively(realsenseUI, false);
    }

    void Update()
    {
        while (mainThreadActions.TryDequeue(out var action))
        {
            action();
        }

        ProcessImageQueue(ref textureLogitech1, imageQueueLogitech1, logitech1Image);
        ProcessImageQueue(ref textureLogitech2, imageQueueLogitech2, logitech2Image);
        ProcessImageQueue(ref textureSiyi, imageQueueSiyi, siyiImage);
        ProcessImageQueue(ref textureZed, imageQueueZed, zedImage);
        ProcessImageQueue(ref textureRealsense, imageQueueRealsense, realsenseImage);

        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("camera_ui_listener_node");

            subLogitech1 = ros2Node.CreateSubscription<Bool>(topicLogitech1, (msg) => EnqueueUIAction(msg, logitech1UI));
            subLogitech2 = ros2Node.CreateSubscription<Bool>(topicLogitech2, (msg) => EnqueueUIAction(msg, logitech2UI));
            subSiyi = ros2Node.CreateSubscription<Bool>(topicSiyi, (msg) => EnqueueUIAction(msg, siyiUI));
            subZed = ros2Node.CreateSubscription<Bool>(topicZed, (msg) => EnqueueUIAction(msg, zedUI));
            subRealsense = ros2Node.CreateSubscription<Bool>(topicRealsense, (msg) => EnqueueUIAction(msg, realsenseUI));

            imageSubLogitech1 = ros2Node.CreateSubscription<CompressedImage>(imageTopicLogitech1, (msg) => EnqueueImage(msg, imageQueueLogitech1));
            imageSubLogitech2 = ros2Node.CreateSubscription<CompressedImage>(imageTopicLogitech2, (msg) => EnqueueImage(msg, imageQueueLogitech2));
            imageSubSiyi = ros2Node.CreateSubscription<CompressedImage>(imageTopicSiyi, (msg) => EnqueueImage(msg, imageQueueSiyi));
            imageSubZed = ros2Node.CreateSubscription<CompressedImage>(imageTopicZed, (msg) => EnqueueImage(msg, imageQueueZed));
            imageSubRealsense = ros2Node.CreateSubscription<CompressedImage>(imageTopicRealsense, (msg) => EnqueueImage(msg, imageQueueRealsense));
        }
    }

    private void EnqueueUIAction(Bool msg, GameObject uiElement)
    {
        mainThreadActions.Enqueue(() =>
        {
            bool newState = msg.Data;
            SetActiveRecursively(uiElement, newState);
            Debug.Log($"UI Element '{uiElement.name}' set to: {(newState ? "Active" : "Inactive")}");
        });
    }

    private void SetActiveRecursively(GameObject obj, bool state)
    {
        if (obj == null) return;

        obj.SetActive(state);
        foreach (Transform child in obj.transform)
        {
            SetActiveRecursively(child.gameObject, state);
        }
    }

    private void EnqueueImage(CompressedImage msg, ConcurrentQueue<byte[]> imageQueue)
    {
        while (imageQueue.Count >= 1) { imageQueue.TryDequeue(out _); }
        imageQueue.Enqueue(msg.Data);
    }

    private void ProcessImageQueue(ref Texture2D texture, ConcurrentQueue<byte[]> imageQueue, RawImage display)
    {
        if (display == null) return;

        if (imageQueue.TryDequeue(out byte[] imageData))
        {
            if (texture == null)
            {
                texture = new Texture2D(2, 2);
            }

            if (texture.LoadImage(imageData))
            {
                display.texture = texture;
            }
        }
    }
}
