using UnityEngine;
using ROS2;
using std_msgs.msg;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;

public class Cam_relocate : MonoBehaviour
{
    [Header("ROS2 Configuration")]
    public string topicNCams = "/n_cams";

    [Header("UI References")]
    public RectTransform zedUI, realsenseUI, siyiUI, logitech1UI, logitech2UI;
    private List<RectTransform> allCameras;

    [Header("Canvas Reference")]
    public RectTransform canvasRect; // Referencia al RectTransform del Canvas

    [Header("Position and Scale Configurations")]
    public CameraConfig[] configsFor1;
    public CameraConfig[] configsFor2;
    public CameraConfig[] configsFor3;
    public CameraConfig[] configsFor4;
    public CameraConfig[] configsFor5;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<Int32> nCamsSubscription;

    private ConcurrentQueue<int> nCamsQueue = new ConcurrentQueue<int>();

    // Ranking de prioridad
    private readonly Dictionary<string, int> cameraPriority = new Dictionary<string, int>
    {
        {"Zed", 1},
        {"Realsense", 2},
        {"Siyi", 3},
        {"Logitech1", 4},
        {"Logitech2", 5}
    };

    private int previousNCams = -1; // Rastrea el número previo de cámaras activas

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();

        // Inicializar lista de cámaras
        allCameras = new List<RectTransform> { zedUI, realsenseUI, siyiUI, logitech1UI, logitech2UI };

        // Aplicar configuración inicial
        int activeCameras = CountActiveCameras();
        previousNCams = activeCameras; 
        AdjustCameras(activeCameras);
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("cam_relocate_node");
            nCamsSubscription = ros2Node.CreateSubscription<Int32>(topicNCams, OnNCamsReceived);
        }

        while (nCamsQueue.TryDequeue(out int nCams))
        {
            // En lugar de verificar si es mayor, siempre volveremos a recolocar con un delay
            Invoke(nameof(AdjustCamerasWithDelay), 0.1f);

            previousNCams = nCams;
        }
    }

    private void OnNCamsReceived(Int32 msg)
    {
        nCamsQueue.Enqueue(msg.Data);
    }

    private void AdjustCamerasWithDelay()
    {
        AdjustCameras(CountActiveCameras());
    }

    private void AdjustCameras(int nCams)
    {
        Debug.Log($"Reubicando cámaras según n_cams: {nCams}");

        var activeCameras = allCameras
            .Where(camera => camera.gameObject.activeSelf)
            .OrderBy(camera => cameraPriority[camera.name.Replace("UI", "")])
            .ToList();

        CameraConfig[] currentConfig = GetCurrentConfig(nCams);

        if (currentConfig == null)
        {
            Debug.LogWarning($"No hay configuración definida para {nCams} cámaras.");
            return;
        }

        if (currentConfig.Length < activeCameras.Count)
        {
            Debug.LogError($"Las configuraciones para {nCams} cámaras no coinciden con la cantidad de cámaras activas.");
            return;
        }

        for (int i = 0; i < currentConfig.Length && i < activeCameras.Count; i++)
        {
            if (activeCameras[i] != null)
            {
                Vector2 adjustedPosition = ConvertToGlobalPosition(currentConfig[i].position);
                activeCameras[i].anchoredPosition = adjustedPosition;
                activeCameras[i].localScale = currentConfig[i].scale;

                Debug.Log($"Cámara '{activeCameras[i].name}' ajustada a Posición: {adjustedPosition}, Escala: {currentConfig[i].scale}");
            }
        }
    }

    private CameraConfig[] GetCurrentConfig(int nCams)
    {
        return nCams switch
        {
            1 => configsFor1,
            2 => configsFor2,
            3 => configsFor3,
            4 => configsFor4,
            5 => configsFor5,
            _ => null,
        };
    }

    private int CountActiveCameras()
    {
        return allCameras.Count(camera => camera.gameObject.activeSelf);
    }

    private Vector2 ConvertToGlobalPosition(Vector2 position)
    {
        float x = -canvasRect.rect.width / 2 + position.x; 
        float y = canvasRect.rect.height / 2 - position.y; 
        return new Vector2(x, y);
    }

    [System.Serializable]
    public class CameraConfig
    {
        public Vector2 position; 
        public Vector3 scale;    
    }
}
