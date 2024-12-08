using UnityEngine;

namespace ROS2
{

public class CamerasMenu : MonoBehaviour
{
    // Start is called before the first frame update
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.String> enable_publisher;
    public string camara = "camara1";

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("UpdateParametersNode");
                enable_publisher = ros2Node.CreatePublisher<std_msgs.msg.String>("/program/update_arguments");
            }
        }
    }

    public void EnableTrue(string camara)
    {
        std_msgs.msg.String msg = new std_msgs.msg.String();
        msg.Data = $"/gui/logitech_cameras,enable_{camara},true";
        enable_publisher.Publish(msg);
    }

    public void EnableFalse(string camara)
    {
        std_msgs.msg.String msg = new std_msgs.msg.String();
        msg.Data = $"/gui/logitech_cameras,enable_{camara},false";
        enable_publisher.Publish(msg);
    }
}

}  // namespace ROS2
