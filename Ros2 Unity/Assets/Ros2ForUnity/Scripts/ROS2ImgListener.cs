using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Concurrent;

public class CompressedImageSubscriber : MonoBehaviour
{
    [Header("UI Configuration")]
    public RawImage rawImageDisplay;

    [Header("ROS2 Configuration")]
    public string topicName_cam1 = "/camara_logitech_1/image_raw/compressed";

    [Header("Optimization")]
    public int maxQueueSize = 1; // Máximo de mensajes en la cola

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<CompressedImage> imageSubscription;

    // Cola concurrente expuesta para modificar en Unity
    public ConcurrentQueue<byte[]> imageQueue = new ConcurrentQueue<byte[]>();

    private Texture2D texture;
    private bool isProcessing = false; // Controla si se está procesando una imagen actualmente

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        // Crear el nodo y suscribirse al tópico si aún no se ha hecho
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("compressed_image_listener_node");

            // Crear la suscripción sin configuración específica de QoS
            imageSubscription = ros2Node.CreateSubscription<CompressedImage>(
                topicName_cam1,
                ImagenRecibida
            );
        }

        // Procesar la imagen más reciente si no se está procesando otra
        if (!isProcessing && imageQueue.TryDequeue(out byte[] imageData))
        {
            isProcessing = true;
            ProcessImage(imageData);
        }
    }

    private void ImagenRecibida(CompressedImage msg)
    {
        // Mantener la cola dentro del tamaño máximo
        while (imageQueue.Count >= maxQueueSize)
        {
            imageQueue.TryDequeue(out _); // Descarta mensajes antiguos
        }

        // Encolar los datos de la imagen recibida
        imageQueue.Enqueue(msg.Data);
    }

    private void ProcessImage(byte[] imageData)
    {
        // Crear la textura solo una vez
        if (texture == null)
        {
            texture = new Texture2D(2, 2); // Ajusta la resolución según sea necesario
        }

        // Cargar los datos en la textura
        texture.LoadImage(imageData);
        texture.Apply();

        // Asignar la textura al RawImage
        rawImageDisplay.texture = texture;

        isProcessing = false; // Marcar como terminado el procesamiento
    }
}
