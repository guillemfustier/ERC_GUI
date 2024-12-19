using UnityEngine;
using UnityEngine.UI;
using System;
using ROS2;

public class botones : MonoBehaviour
{

    [Header("UI References - Cámaras")]
    public RawImage logitech1RawImage;
    public Button logitech1Button;

    public RawImage logitech2RawImage;
    public Button logitech2Button;

    public RawImage siyiRawImage;
    public Button siyiButton;

    public RawImage zedRawImage;
    public Button zedButton;

    [Header("UI References - Lidares")]
    public RawImage lidar2DRawImage;
    public Button lidar2DButton;

    public RawImage lidar4DRawImage;
    public Button lidar4DButton;

    [Header("UI References - RealSense")]
    public RawImage realsenseRawImage;
    public Button realsenseButton;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    // Publishers para cada dispositivo
    private IPublisher<std_msgs.msg.Bool> logitech1Pub;
    private IPublisher<std_msgs.msg.Bool> logitech2Pub;
    private IPublisher<std_msgs.msg.Bool> siyiPub;
    private IPublisher<std_msgs.msg.Bool> zedPub;
    private IPublisher<std_msgs.msg.Bool> lidar2DPub;
    private IPublisher<std_msgs.msg.Bool> lidar4DPub;
    private IPublisher<std_msgs.msg.Bool> realsensePub;

    // Publisher para /n_cams
    private IPublisher<std_msgs.msg.Int32> nCamsPub;

    // Estados iniciales (todos activos/true)
    private bool logitech1State = true;
    private bool logitech2State = true;
    private bool siyiState = true;
    private bool zedState = true;
    private bool lidar2DState = true;
    private bool lidar4DState = true;
    private bool realsenseState = true;

    // Contador inicial de dispositivos activos
    private int nCamsValue = 5;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("gui_botonera_node");

            // Crear publicadores
            logitech1Pub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_logitech_1");
            logitech2Pub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_logitech_2");
            siyiPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_siyi");
            zedPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_zed");
            lidar2DPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_lidar_2d");
            lidar4DPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_lidar_4d");
            realsensePub = ros2Node.CreatePublisher<std_msgs.msg.Bool>("/gui_cam_realsense");

            nCamsPub = ros2Node.CreatePublisher<std_msgs.msg.Int32>("/n_cams");

            // Publicar el valor inicial de n_cams
            PublishNCams();
        }
    }

    // Método genérico para togglear un dispositivo
    // deviceState: el estado bool del dispositivo
    // setDeviceState: acción para actualizar la variable de estado
    // pub: publisher del dispositivo
    // button: el botón del dispositivo
    // rawImage: el RawImage del dispositivo
    private void ToggleDevice(ref bool deviceState, IPublisher<std_msgs.msg.Bool> pub, Button button, RawImage rawImage)
    {
        // Cambiar de estado
        deviceState = !deviceState;

        // Publicar el nuevo estado
        std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
        msg.Data = deviceState;
        pub.Publish(msg);

        // Mostrar/ocultar UI
        button.gameObject.SetActive(deviceState);
        rawImage.gameObject.SetActive(deviceState);

        // Actualizar n_camsValue según el nuevo estado
        if (deviceState)
        {
            // Si acabamos de pasar a true, sumamos 1
            nCamsValue++;
        }
        else
        {
            // Si acabamos de pasar a false, restamos 1
            nCamsValue--;
        }

        // Publicar el nuevo valor de n_cams
        PublishNCams();
    }

    private void PublishNCams()
    {
        if (nCamsPub != null)
        {
            std_msgs.msg.Int32 nCamsMsg = new std_msgs.msg.Int32();
            nCamsMsg.Data = nCamsValue;
            nCamsPub.Publish(nCamsMsg);
        }
    }

    public void ToggleLogitech1()
    {
        if (logitech1Pub == null) return;
        ToggleDevice(ref logitech1State, logitech1Pub, logitech1Button, logitech1RawImage);
    }

    public void ToggleLogitech2()
    {
        if (logitech2Pub == null) return;
        ToggleDevice(ref logitech2State, logitech2Pub, logitech2Button, logitech2RawImage);
    }

    public void ToggleSiyi()
    {
        if (siyiPub == null) return;
        ToggleDevice(ref siyiState, siyiPub, siyiButton, siyiRawImage);
    }

    public void ToggleZed()
    {
        if (zedPub == null) return;
        ToggleDevice(ref zedState, zedPub, zedButton, zedRawImage);
    }

    public void ToggleLidar2D()
    {
        if (lidar2DPub == null) return;
        ToggleDevice(ref lidar2DState, lidar2DPub, lidar2DButton, lidar2DRawImage);
    }

    public void ToggleLidar4D()
    {
        if (lidar4DPub == null) return;
        ToggleDevice(ref lidar4DState, lidar4DPub, lidar4DButton, lidar4DRawImage);
    }

    public void ToggleRealsense()
    {
        if (realsensePub == null) return;
        ToggleDevice(ref realsenseState, realsensePub, realsenseButton, realsenseRawImage);
    }
}
