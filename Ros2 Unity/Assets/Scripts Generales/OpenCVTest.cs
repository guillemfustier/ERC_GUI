/*
using System;
using System.IO;
using UnityEngine;
using UnityEngine.UI; // Para trabajar con UI
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.CvEnum;

public class DisplayImageOnCanvas : MonoBehaviour
{
    public RawImage rawImage; // Referencia al componente UI Image
    public string imagePath = "image.png"; // Ruta de la imagens

    void Start()
    {
        // Cargar y procesar la imagen con OpenCV
        Mat img = CvInvoke.Imread(imagePath, ImreadModes.Color); // Cargar la imagen en color

        if (img.IsEmpty)
        {
            Debug.LogError("No se pudo cargar la imagen desde " + imagePath);
            return;
        }

        // Convertir la imagen a Texture2D
        Texture2D texture = ConvertMatToTexture2D(img);

        // Mostrar la imagen en el UI
        if (rawImage != null && texture != null)
        {
            rawImage.texture = texture;
        }
    }

    // Método para convertir un Mat de OpenCV a Texture2D
    private Texture2D ConvertMatToTexture2D(Mat mat)
    {
        // Asegúrate de que el Mat es compatible con Texture2D
        if (mat.NumberOfChannels == 3)
        {
            CvInvoke.CvtColor(mat, mat, ColorConversion.Bgr2Rgb); // Convertir de BGR a RGB
        }

        Texture2D texture = new Texture2D(mat.Width, mat.Height, TextureFormat.RGB24, false);
        byte[] imageData = mat.ToImage<Bgr, byte>().ToBitmap().ToByteArray(); // Convertir a byte array
        texture.LoadImage(imageData); // Cargar los datos en la textura

        return texture;
    }
}
*/