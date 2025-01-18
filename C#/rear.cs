using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class UDPCommunication : MonoBehaviour
{
    UdpClient udpClient;
    string ipAddress = "192.168.0.141"; 
    int port = 55555; 
    public WheelCollider rearWheel;
    private float lastWheelRPM = 0;
    private bool isWheelStopped = false; 

    void Start()
    {
        udpClient = new UdpClient();
        InvokeRepeating("SendWheelData", 0f, 0.02f); 
    }

    void SendWheelData()
    {
        float wheelRPM = -rearWheel.rpm; 

        if (Mathf.Abs(wheelRPM - lastWheelRPM) > 1f) 
        {
            lastWheelRPM = wheelRPM;
            string message = $"wheel:{wheelRPM:F2}"; 
            byte[] data = Encoding.UTF8.GetBytes(message);
            try
            {
                udpClient.Send(data, data.Length, ipAddress, port);
                Debug.Log($"Wheel RPM sent: {wheelRPM}");
                isWheelStopped = false;
            }
            catch (SocketException e)
            {
                Debug.LogError("SocketException: " + e.Message);
            }
        }
        else if (Mathf.Abs(wheelRPM) <= 0.1f && !isWheelStopped) 
        {
            SendStopMessage();
            isWheelStopped = true; 
        }
    }

        void SendStopMessage()
    {
        string stopMessage = "wheel:stop"; 
        byte[] data = Encoding.UTF8.GetBytes(stopMessage);
        try
        {
            udpClient.Send(data, data.Length, ipAddress, port);
            Debug.Log("Wheel stopped, stop message sent.");
        }
        catch (SocketException e)
        {
            Debug.LogError("SocketException: " + e.Message);
        }
    }

    void OnApplicationPause(bool isPaused)
    {
        if (isPaused)
        {
            Debug.Log("Application Paused. Sending stop message.");
            SendStopMessage(); 
        }
    }

    
    void OnApplicationQuit()
    {
        Debug.Log("Application Quit. Sending stop message.");
        SendStopMessage(); 
        udpClient.Close(); 
    }
}
