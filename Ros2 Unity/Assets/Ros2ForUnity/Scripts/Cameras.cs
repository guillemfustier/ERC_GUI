using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Concurrent;

public class Cameras : MonoBehaviour
{
    [Header("UI Configuration")]
    public RawImage rawImageDisplay1;
    public RawImage rawImageDisplay2;
    public RawImage rawImageDisplay3;
    public RawImage rawImageDisplay4;
    public RawImage rawImageDisplay5;

    [Header("ROS2 Configuration")]
    public string topicName1 = "/camara_logitech_1/image_raw/compressed";
    public string topicName2 = "/camara_logitech_2/image_raw/compressed";
    public string topicName3 = "/camara_logitech_3/image_raw/compressed";
    public string topicName4 = "/camara_logitech_4/image_raw/compressed";
    public string topicName5 = "/camara_logitech_5/image_raw/compressed";

    [Header("Optimization")]
    public int maxQueueSize = 1; // Máximo de mensajes en la cola

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private bool subscriptionsCreated = false;

    // Suscripciones a cada cámara
    private ISubscription<CompressedImage> imageSubscription1;
    private ISubscription<CompressedImage> imageSubscription2;
    private ISubscription<CompressedImage> imageSubscription3;
    private ISubscription<CompressedImage> imageSubscription4;
    private ISubscription<CompressedImage> imageSubscription5;

    // Colas separadas para cada cámara
    private ConcurrentQueue<byte[]> imageQueue1 = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueue2 = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueue3 = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueue4 = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> imageQueue5 = new ConcurrentQueue<byte[]>();

    // Texturas separadas para cada cámara
    private Texture2D texture1;
    private Texture2D texture2;
    private Texture2D texture3;
    private Texture2D texture4;
    private Texture2D texture5;

    // Flags de procesamiento
    private bool isProcessing1 = false; 
    private bool isProcessing2 = false;
    private bool isProcessing3 = false; 
    private bool isProcessing4 = false;
    private bool isProcessing5 = false; 

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
        }

        // Crear las suscripciones cuando tengamos el nodo creado y no se hayan creado aún
        if (ros2Node != null && !subscriptionsCreated)
        {
            Camera1Subscriber();
            Camera2Subscriber();
            Camera3Subscriber();
            Camera4Subscriber();
            Camera5Subscriber();

            subscriptionsCreated = true;
        }

        // Procesar la cola de la cámara 1
        if (!isProcessing1 && imageQueue1.TryDequeue(out byte[] imageData1))
        {
            isProcessing1 = true;
            ProcessImage(imageData1, ref texture1, rawImageDisplay1, () => { isProcessing1 = false; });
        }

        // Procesar la cola de la cámara 2
        if (!isProcessing2 && imageQueue2.TryDequeue(out byte[] imageData2))
        {
            isProcessing2 = true;
            ProcessImage(imageData2, ref texture2, rawImageDisplay2, () => { isProcessing2 = false; });
        }

        // Procesar la cola de la cámara 3
        if (!isProcessing3 && imageQueue3.TryDequeue(out byte[] imageData3))
        {
            isProcessing3 = true;
            ProcessImage(imageData3, ref texture3, rawImageDisplay3, () => { isProcessing3 = false; });
        }

        // Procesar la cola de la cámara 4
        if (!isProcessing4 && imageQueue4.TryDequeue(out byte[] imageData4))
        {
            isProcessing4 = true;
            ProcessImage(imageData4, ref texture4, rawImageDisplay4, () => { isProcessing4 = false; });
        }

        // Procesar la cola de la cámara 5
        if (!isProcessing5 && imageQueue5.TryDequeue(out byte[] imageData5))
        {
            isProcessing5 = true;
            ProcessImage(imageData5, ref texture5, rawImageDisplay5, () => { isProcessing5 = false; });
        }
    }

    void Camera1Subscriber()
    {
        imageSubscription1 = ros2Node.CreateSubscription<CompressedImage>(
            topicName1,
            ImagenRecibidaCam1
        );
    }

    void Camera2Subscriber()
    {
        imageSubscription2 = ros2Node.CreateSubscription<CompressedImage>(
            topicName2,
            ImagenRecibidaCam2
        );
    }

    void Camera3Subscriber()
    {
        imageSubscription3 = ros2Node.CreateSubscription<CompressedImage>(
            topicName3,
            ImagenRecibidaCam3
        );
    }

    void Camera4Subscriber()
    {
        imageSubscription4 = ros2Node.CreateSubscription<CompressedImage>(
            topicName4,
            ImagenRecibidaCam4
        );
    }

    void Camera5Subscriber()
    {
        imageSubscription5 = ros2Node.CreateSubscription<CompressedImage>(
            topicName5,
            ImagenRecibidaCam5
        );
    }

    private void ImagenRecibidaCam1(CompressedImage msg)
    {
        while (imageQueue1.Count >= maxQueueSize) { imageQueue1.TryDequeue(out _); }
        imageQueue1.Enqueue(msg.Data);
    }

    private void ImagenRecibidaCam2(CompressedImage msg)
    {
        while (imageQueue2.Count >= maxQueueSize) { imageQueue2.TryDequeue(out _); }
        imageQueue2.Enqueue(msg.Data);
    }

    private void ImagenRecibidaCam3(CompressedImage msg)
    {
        while (imageQueue3.Count >= maxQueueSize) { imageQueue3.TryDequeue(out _); }
        imageQueue3.Enqueue(msg.Data);
    }

    private void ImagenRecibidaCam4(CompressedImage msg)
    {
        while (imageQueue4.Count >= maxQueueSize) { imageQueue4.TryDequeue(out _); }
        imageQueue4.Enqueue(msg.Data);
    }

    private void ImagenRecibidaCam5(CompressedImage msg)
    {
        while (imageQueue5.Count >= maxQueueSize) { imageQueue5.TryDequeue(out _); }
        imageQueue5.Enqueue(msg.Data);
    }

    private void ProcessImage(byte[] imageData, ref Texture2D texture, RawImage display, Action onComplete)
    {
        // Crear la textura si no existe
        if (texture == null)
        {
            texture = new Texture2D(2, 2); 
        }

        // Cargar los datos en la textura
        texture.LoadImage(imageData);
        texture.Apply();

        // Asignar la textura al RawImage correspondiente
        display.texture = texture;

        onComplete?.Invoke();
    }
}
