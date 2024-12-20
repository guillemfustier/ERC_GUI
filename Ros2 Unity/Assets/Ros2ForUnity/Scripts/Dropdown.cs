using ROS2;
using std_msgs.msg;
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using TMPro;
using System.Collections.Concurrent;

public class DropdownDevices : MonoBehaviour
{
    public TMP_Dropdown devicesDropdown;

    [Header("UI Configuration")]
    public GameObject prefabLogitech1;
    public GameObject prefabLogitech2;
    public GameObject prefabSiyi;
    public GameObject prefabZed;
    public GameObject prefabRealsense;

    public Transform uiParent;

    public GameObject zedUI;
    public GameObject realsenseUI;
    public GameObject siyiUI;
    public GameObject logitech1UI;
    public GameObject logitech2UI;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    private Dictionary<string, string> topicToDeviceName = new Dictionary<string, string>
    {
        {"/gui_cam_logitech_1", "Logitech Cam 1"},
        {"/gui_cam_logitech_2", "Logitech Cam 2"},
        {"/gui_cam_siyi", "Siyi Cam"},
        {"/gui_cam_zed", "Zed Cam"},
        {"/gui_cam_realsense", "Realsense Cam"}
    };

    private Dictionary<string, bool> deviceStates = new Dictionary<string, bool>();
    private Dictionary<string, ISubscription<Bool>> subscriptions = new Dictionary<string, ISubscription<Bool>>();

    private ConcurrentQueue<System.Action> mainThreadActions = new ConcurrentQueue<System.Action>();

    private IPublisher<Int32> nCamsPub;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        devicesDropdown.ClearOptions();

        devicesDropdown.options.Add(new TMP_Dropdown.OptionData("Cámaras"));
        devicesDropdown.onValueChanged.AddListener(OnDropdownValueChanged);

        foreach (var kvp in topicToDeviceName)
        {
            deviceStates[kvp.Value] = false;
        }

        InitializeDeviceStates();
        UpdateDropdown();
    }

    void Update()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("dropdown_listener_node");

            foreach (var topic in topicToDeviceName.Keys)
            {
                subscriptions[topic] = ros2Node.CreateSubscription<Bool>(topic, (msg) => OnDeviceMsgReceived(topic, msg));
            }

            nCamsPub = ros2Node.CreatePublisher<Int32>("/n_cams");
        }

        while (mainThreadActions.TryDequeue(out var action))
        {
            action();
        }
    }

    private void InitializeDeviceStates()
    {
        zedUI.SetActive(false);
        realsenseUI.SetActive(false);
        siyiUI.SetActive(false);
        logitech1UI.SetActive(false);
        logitech2UI.SetActive(false);
    }

    private void OnDeviceMsgReceived(string topic, Bool msg)
    {
        string deviceName = topicToDeviceName[topic];
        bool newState = msg.Data;

        Debug.Log($"Mensaje recibido en {topic}: {newState}");

        deviceStates[deviceName] = newState;

        mainThreadActions.Enqueue(() => UpdateDropdown());
    }

    private void UpdateDropdown()
    {
        devicesDropdown.ClearOptions();
        devicesDropdown.options.Add(new TMP_Dropdown.OptionData("Cámaras"));
        foreach (var kvp in topicToDeviceName)
        {
            if (!deviceStates[kvp.Value])
            {
                devicesDropdown.options.Add(new TMP_Dropdown.OptionData(kvp.Value));
            }
        }
        devicesDropdown.RefreshShownValue();
    }

    private void OnDropdownValueChanged(int index)
    {
        if (index == 0) return;

        string selectedOption = devicesDropdown.options[index].text;
        Debug.Log("Opción seleccionada: " + selectedOption);

        foreach (var kvp in topicToDeviceName)
        {
            if (kvp.Value == selectedOption)
            {
                var pub = ros2Node.CreatePublisher<Bool>(kvp.Key);
                Bool msg = new Bool { Data = true };
                pub.Publish(msg);
                Debug.Log($"Enviado True al topic: {kvp.Key}");

                ActivateCameraUI(kvp.Value);

                PublishNCams();

                break;
            }
        }
    }

    private void ActivateCameraUI(string cameraName)
    {
        switch (cameraName)
        {
            case "Logitech Cam 1":
                logitech1UI.SetActive(true);
                break;
            case "Logitech Cam 2":
                logitech2UI.SetActive(true);
                break;
            case "Siyi Cam":
                siyiUI.SetActive(true);
                break;
            case "Zed Cam":
                zedUI.SetActive(true);
                break;
            case "Realsense Cam":
                realsenseUI.SetActive(true);
                break;
        }
    }

    private void PublishNCams()
    {
        if (nCamsPub != null)
        {
            int activeCount = CountActiveCameras();
            Int32 nCamsMsg = new Int32();
            nCamsMsg.Data = activeCount;
            nCamsPub.Publish(nCamsMsg);
            Debug.Log($"Publicado n_cams={activeCount} desde DropdownDevices");
        }
    }

    private int CountActiveCameras()
    {
        int count = 0;
        if (zedUI.activeSelf) count++;
        if (realsenseUI.activeSelf) count++;
        if (siyiUI.activeSelf) count++;
        if (logitech1UI.activeSelf) count++;
        if (logitech2UI.activeSelf) count++;

        return count;
    }
}
