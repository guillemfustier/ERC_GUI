using UnityEngine;
using UnityEngine.UI;
using TMPro; // Importa TextMeshPro
using ROS2;
using std_msgs.msg;

public class TerminalInput : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<String> terminalInputPublisher;

    [Header("UI Configuration")]
    public TMP_InputField terminalInputField; // Campo de entrada de texto
    public Button sendButton; // Botón para enviar el texto

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        sendButton.onClick.AddListener(OnSendButtonClicked); // Asigna el evento de clic
    }

    void Update()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("terminal_input_node");
            terminalInputPublisher = ros2Node.CreatePublisher<String>("/gui/terminal_input");
        }
    }

    public void OnSendButtonClicked()
    {
        if (terminalInputPublisher != null && !string.IsNullOrWhiteSpace(terminalInputField.text))
        {
            String msg = new String
            {
                Data = terminalInputField.text
            };

            terminalInputPublisher.Publish(msg);
            Debug.Log($"Mensaje enviado: {msg.Data}");

            // Limpia el campo de texto después de enviar
            terminalInputField.text = string.Empty;
        }
    }
}
