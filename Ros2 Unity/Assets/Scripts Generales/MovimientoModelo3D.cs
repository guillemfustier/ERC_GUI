using UnityEngine;
using UnityEngine.UI;

public class WheelRotation : MonoBehaviour
{
    // Variables públicas para configurar las ruedas
    public Transform[] leftWheels; // Array para las ruedas izquierdas
    public Transform[] rightWheels; // Array para las ruedas derechas
    public float rotationSpeed = 1000f; // Velocidad de rotación de las ruedas
    public float steeringFactor = 0.5f; // Factor para ajustar la diferencia de rotación al girar

    // Sliders para controlar la rotación de los objetos
    public Slider leftSlider; // Slider para el primer objeto
    public Slider rightSlider; // Slider para el segundo objeto
    public Transform leftObject; // Objeto controlado por el primer slider
    public Transform rightObject; // Objeto controlado por el segundo slider

    private float PosicionOriginal_ZedHorizontal = 90f;
    private float PosicionOriginal_ZedVertical = -195.542f;

    void Start()
    {
        // Configuramos los rangos de los sliders
        if (leftSlider != null)
        {
            leftSlider.minValue = -90f;
            leftSlider.maxValue = 90f;
        }

        if (rightSlider != null)
        {
            rightSlider.minValue = -90f;
            rightSlider.maxValue = 90f;
        }
    }

    void Update()
    {
        // Capturamos los inputs vertical y horizontal
        float verticalInput = Input.GetAxis("Vertical");
        float horizontalInput = Input.GetAxis("Horizontal");

        // Si hay ruedas asignadas, aplicamos la rotación
        if ((leftWheels != null && leftWheels.Length > 0) && (rightWheels != null && rightWheels.Length > 0))
        {
            foreach (Transform leftWheel in leftWheels)
            {
                // Ruedas izquierdas giran ligeramente más rápido si giramos a la derecha
                float adjustedSpeed = rotationSpeed * (1 + horizontalInput * steeringFactor);
                leftWheel.Rotate(Vector3.right * (verticalInput + Mathf.Abs(horizontalInput)) * adjustedSpeed * Time.deltaTime);
            }

            foreach (Transform rightWheel in rightWheels)
            {
                // Ruedas derechas giran ligeramente más rápido si giramos a la izquierda
                float adjustedSpeed = rotationSpeed * (1 - horizontalInput * steeringFactor);
                rightWheel.Rotate(Vector3.right * (verticalInput + Mathf.Abs(horizontalInput)) * adjustedSpeed * Time.deltaTime);
            }
        }

        // Actualizamos la rotación de los objetos según los valores de los sliders
        if (leftObject != null && leftSlider != null)
        {
            leftObject.localRotation = Quaternion.Euler(0f, PosicionOriginal_ZedHorizontal+leftSlider.value, 0f);
        }

        if (rightObject != null && rightSlider != null)
        {
            rightObject.localRotation = Quaternion.Euler(rightSlider.value, PosicionOriginal_ZedVertical, 0f);
        }
    }
}
