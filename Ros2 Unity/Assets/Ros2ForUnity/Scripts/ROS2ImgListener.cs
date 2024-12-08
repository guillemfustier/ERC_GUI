using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Concurrent;

public class CompressedImageSubscriber : MonoBehaviour
{
    [Header("UI Configuration")]
    public RawImage Display1;
    public RawImage Display2;

    [Header("ROS2 Configuration")]
    public string topicName_cam1 = "/camara_logitech_1/image_raw/compressed";
    public string topicName_cam2 = "/camara_logitech_2/image_raw/compressed";

    [Header("Optimization")]
    public int maxQueueSize = 1; // Máximo de mensajes en la cola

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    // Colas independientes para cada cámara
    private ConcurrentQueue<byte[]> imageQueue_cam1 = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueue_cam2 = new ConcurrentQueue<byte[]>();

    private ConcurrentQueue<Action> mainThreadQueue = new ConcurrentQueue<Action>(); // Para operaciones en el hilo principal

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        // Crear el nodo una sola vez
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("compressed_image_listener_node");
            Camera1Subscriber();
            Camera2Subscriber();
        }

        // Ejecutar acciones en el hilo principal
        while (mainThreadQueue.TryDequeue(out Action action))
        {
            action.Invoke();
        }
    }

    void Camera1Subscriber()
    {
        ros2Node.CreateSubscription<CompressedImage>(
            topicName_cam1,
            msg => ImagenRecibida(msg, imageQueue_cam1, Display1)
        );
        Debug.Log("Suscripción creada para cámara 1");
    }

    void Camera2Subscriber()
    {
        ros2Node.CreateSubscription<CompressedImage>(
            topicName_cam2,
            msg => ImagenRecibida(msg, imageQueue_cam2, Display2)
        );
        Debug.Log("Suscripción creada para cámara 2");
    }

    private void ImagenRecibida(CompressedImage msg, ConcurrentQueue<byte[]> queue, RawImage display)
    {
        // Mantener la cola dentro del tamaño máximo
        while (queue.Count >= maxQueueSize)
        {
            queue.TryDequeue(out _); // Descartar mensajes antiguos
        }

        // Encolar los datos de la imagen recibida
        queue.Enqueue(msg.Data);

        // Procesar la imagen más reciente en el hilo principal
        if (queue.TryDequeue(out byte[] imageData))
        {
            mainThreadQueue.Enqueue(() => ProcessImage(imageData, display));
        }
    }

    private void ProcessImage(byte[] imageData, RawImage rawImageDisplay)
    {
        // Crear la textura si no existe
        Texture2D texture = rawImageDisplay.texture as Texture2D;
        if (texture == null)
        {
            texture = new Texture2D(2, 2); // Ajusta la resolución según sea necesario
            rawImageDisplay.texture = texture;
        }

        // Cargar los datos en la textura
        texture.LoadImage(imageData);
        texture.Apply();
    }
}
